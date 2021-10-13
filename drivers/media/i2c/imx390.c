// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX390 Camera Driver
 *
 * Copyright (c) 2021 Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/v4l2-common.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "imx390_regs.h"

struct imx390_priv {
	struct device			*dev;

	struct regmap			*regmap;
	struct clk			*clk;
	struct gpio_desc		*xclr_gpio;

	unsigned long			clk_rate;

	struct v4l2_subdev		subdev;
	struct media_pad		pad;

	bool				streaming;
};

static const struct v4l2_area imx390_framesizes[] = {
	{
		.width		= IMX390_OUT_WIDTH,
		.height		= IMX390_OUT_HEIGHT,
	},
};

static const u32 imx390_mbus_formats[] = {
	MEDIA_BUS_FMT_SRGGB12_1X12,
};

static inline struct imx390_priv *to_imx390(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx390_priv, subdev);
}

/* -----------------------------------------------------------------------------
 * Read/Write Helpers
 */

static int imx390_read(struct imx390_priv *priv, u16 reg, u8 *val)
{
	unsigned int v;
	int ret;

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to read register 0x%04x: %d\n", reg,
			ret);
		return ret;
	}

	*val = v;

	return 0;
}

static int imx390_write(struct imx390_priv *priv, u16 reg, u8 val, int *err)
{
	int ret;

	if (err && *err)
		return *err;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret < 0) {
		dev_err(priv->dev, "Cannot write register 0x%02x: %d!\n", reg,
			ret);

		if (err)
			*err = ret;
	}

	return ret;
}

static int imx390_write_array(struct imx390_priv *priv,
			      const struct imx390_reg *regs,
			      unsigned int nr_regs)
{
	unsigned int i;
	int ret;

	for (i = 0; i < nr_regs; i++) {
		ret = imx390_write(priv, regs[i].reg, regs[i].val, NULL);
		if (ret)
			return ret;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 */

static int imx390_set_ae_params(struct imx390_priv *priv)
{
	u16 exposure = 3670; // TODO: hardcoded exposure for now
	unsigned int i;
	u32 sp1h_again = 0;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(imx390_gains_table); ++i) {
		if (exposure < imx390_gains_table[i][0]) {
			sp1h_again = imx390_gains_table[i][1];
			break;
		}
	}

	if (sp1h_again == 0)
		return -EINVAL;

	imx390_write(priv, IMX390_REG_CAT0_REG_HOLD, 1, &ret);
	imx390_write(priv, IMX390_REG_CAT0_AGAIN_SP1H_LOW,
		     sp1h_again & 0xff, &ret);
	imx390_write(priv, IMX390_REG_CAT0_AGAIN_SP1H_HIGH,
		     sp1h_again >> 8, &ret);
	imx390_write(priv, IMX390_REG_CAT0_REG_HOLD, 0, &ret);

	return ret;
}

static int imx390_configure(struct imx390_priv *priv)
{
	int ret;

	ret = imx390_write_array(
		priv, imx390_regs_linear_1920x1080_config,
		ARRAY_SIZE(imx390_regs_linear_1920x1080_config));
	if (ret < 0)
		return ret;

	msleep(100);

	ret = imx390_set_ae_params(priv);
	if (ret < 0)
		return ret;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static int imx390_start_stream(struct imx390_priv *priv)
{
	int ret;

	priv->streaming = true;

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0)
		goto err;

	ret = imx390_configure(priv);
	if (ret < 0)
		goto err;

	/* set active */
	ret = imx390_write(priv, IMX390_REG_CAT0_STANDBY, 0, NULL);
	if (ret < 0)
		goto err;

	/* No communication is possible for a while after exiting standby */
	msleep(20);

	return 0;

err:
	/*
	 * In case of error, turn the power off synchronously as the
	 * device likely has no other chance to recover.
	 */
	pm_runtime_put_sync(priv->dev);
	priv->streaming = false;

	return ret;
}

static void imx390_stop_stream(struct imx390_priv *priv)
{
	/* set standby */
	imx390_write(priv, IMX390_REG_CAT0_STANDBY, 1, NULL);
	/* No communication is possible for a while after entering standby */
	usleep_range(10000, 20000);

	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_put_autosuspend(priv->dev);

	priv->streaming = false;
}

static int imx390_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx390_priv *priv = to_imx390(sd);

	if (enable)
		return imx390_start_stream(priv);

	imx390_stop_stream(priv);

	return 0;
}

static void imx390_init_formats(struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_state_get_stream_format(state, 0, 0);
	format->code = imx390_mbus_formats[0];
	format->width = imx390_framesizes[0].width;
	format->height = imx390_framesizes[0].height;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SMPTE170M;

	if (state->routing.routes[1].flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE) {
		format = v4l2_state_get_stream_format(state, 0, 1);
		format->code = MEDIA_BUS_FMT_METADATA_16;
		format->width = imx390_framesizes[0].width;
		format->height = IMX390_METADATA_BEFORE_HEIGHT;
		format->field = V4L2_FIELD_NONE;
		format->colorspace = V4L2_COLORSPACE_DEFAULT;
	}
}

static int _imx390_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       bool enable_embedded_data)
{
	struct v4l2_subdev_route routes[] = {
		{
			.source_pad = 0,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_IMMUTABLE |
				 V4L2_SUBDEV_ROUTE_FL_SOURCE |
				 V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.source_pad = 0,
			.source_stream = 1,
			.flags = V4L2_SUBDEV_ROUTE_FL_SOURCE,
		}
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	int ret;

	if (enable_embedded_data)
		routes[1].flags |= V4L2_SUBDEV_ROUTE_FL_ACTIVE;

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret)
		return ret;

	imx390_init_formats(state);

	return 0;
}

static int imx390_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	int ret;

	v4l2_subdev_lock_state(state);

	ret = _imx390_set_routing(sd, state, false);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int imx390_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(imx390_mbus_formats))
		return -EINVAL;

	code->code = imx390_mbus_formats[code->index];

	return 0;
}

static int imx390_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx390_mbus_formats); ++i) {
		if (imx390_mbus_formats[i] == fse->code)
			break;
	}

	if (i == ARRAY_SIZE(imx390_mbus_formats))
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(imx390_framesizes))
		return -EINVAL;

	fse->min_width = imx390_framesizes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->max_height = imx390_framesizes[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static int imx390_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx390_priv *priv = to_imx390(sd);
	struct v4l2_mbus_framefmt *format;
	const struct v4l2_area *fsize;
	unsigned int i;
	u32 code;
	int ret = 0;

	if (fmt->pad != 0)
		return -EINVAL;

	/* metadata stream */
	if (fmt->stream == 1)
		return v4l2_subdev_get_fmt(sd, state, fmt);

	if (fmt->stream != 0)
		return -EINVAL;

	/*
	 * Validate the media bus code, defaulting to the first one if the
	 * requested code isn't supported.
	 */
	for (i = 0; i < ARRAY_SIZE(imx390_mbus_formats); ++i) {
		if (imx390_mbus_formats[i] == fmt->format.code) {
			code = fmt->format.code;
			break;
		}
	}

	if (i == ARRAY_SIZE(imx390_mbus_formats))
		code = imx390_mbus_formats[0];

	/* Find the nearest supported frame size. */
	fsize = v4l2_find_nearest_size(imx390_framesizes,
				       ARRAY_SIZE(imx390_framesizes), width,
				       height, fmt->format.width,
				       fmt->format.height);

	v4l2_subdev_lock_state(state);

	/* Update the stored format and return it. */
	format = v4l2_state_get_stream_format(state, fmt->pad, fmt->stream);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming) {
		ret = -EBUSY;
		goto done;
	}

	format->code = code;
	format->width = fsize->width;
	format->height = fsize->height;

	fmt->format = *format;

	/* update metadata width */
	format = v4l2_state_get_stream_format(state, 0, 1);
	if (format)
		format->width = fmt->format.width;

done:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int imx390_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u32 bpp;
	int ret = 0;

	if (pad != 0)
		return -EINVAL;

	state = v4l2_subdev_lock_active_state(sd);

	fmt = v4l2_state_get_stream_format(state, 0, 0);
	if (!fmt) {
		ret = -EPIPE;
		goto out;
	}

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	/* pixel stream */

	bpp = 12;

	fd->entry[fd->num_entries].stream = 0;

	fd->entry[fd->num_entries].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
	fd->entry[fd->num_entries].length = fmt->width * fmt->height * bpp / 8;
	fd->entry[fd->num_entries].pixelcode = fmt->code;
	fd->entry[fd->num_entries].bus.csi2.vc = 0;
	fd->entry[fd->num_entries].bus.csi2.dt = 0x2c; /* SRGGB12 */

	fd->num_entries++;

	/* meta stream */
	if (state->routing.routes[1].flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE) {
		fd->entry[fd->num_entries].stream = 1;

		fd->entry[fd->num_entries].flags =
			V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
		fd->entry[fd->num_entries].length =
			fmt->width * IMX390_METADATA_BEFORE_HEIGHT * bpp / 8;
		fd->entry[fd->num_entries].pixelcode = fmt->code;
		fd->entry[fd->num_entries].bus.csi2.vc = 0;
		fd->entry[fd->num_entries].bus.csi2.dt = 0x12; /* Metadata */

		fd->num_entries++;
	}

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int imx390_set_routing(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      enum v4l2_subdev_format_whence which,
			      struct v4l2_subdev_krouting *routing)
{
	bool enable_embedded;
	int ret;

	if (routing->num_routes == 0 || routing->num_routes > 2)
		return -EINVAL;

	/*
	 * The only thing that can be changed is whether the metadata stream
	 * is active or not.
	 */
	enable_embedded =
		routing->num_routes == 2 &&
		(routing->routes[1].flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE);

	v4l2_subdev_lock_state(state);

	ret = _imx390_set_routing(sd, state, enable_embedded);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static const struct v4l2_subdev_video_ops imx390_subdev_video_ops = {
	.s_stream	= imx390_s_stream,
};

static const struct v4l2_subdev_pad_ops imx390_subdev_pad_ops = {
	.init_cfg		= imx390_init_cfg,
	.enum_mbus_code		= imx390_enum_mbus_code,
	.enum_frame_size	= imx390_enum_frame_sizes,
	.get_fmt		= v4l2_subdev_get_fmt,
	.set_fmt		= imx390_set_fmt,
	.set_routing		= imx390_set_routing,
	.get_frame_desc		= imx390_get_frame_desc,
};

static const struct v4l2_subdev_ops imx390_subdev_ops = {
	.video	= &imx390_subdev_video_ops,
	.pad	= &imx390_subdev_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Power Management
 */

static int imx390_power_on(struct imx390_priv *priv)
{
	int ret;

	ret = clk_prepare_enable(priv->clk);
	if (ret < 0)
		return ret;

	if (priv->xclr_gpio) {
		gpiod_set_value_cansleep(priv->xclr_gpio, 1);
		/* Keep the XCLR pin on Low for 100 µs or longer */
		usleep_range(100, 1000);
		gpiod_set_value_cansleep(priv->xclr_gpio, 0);
		/* It takes max 30 ms for the sensor to be ready */
		msleep(30);
	}

	return 0;
}

static void imx390_power_off(struct imx390_priv *priv)
{
	clk_disable_unprepare(priv->clk);
}

static int imx390_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx390_priv *priv = to_imx390(subdev);
	int ret;

	ret = imx390_power_on(priv);
	if (ret < 0)
		return ret;

	return 0;
}

static int imx390_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx390_priv *priv = to_imx390(subdev);

	imx390_power_off(priv);

	return 0;
}

static const struct dev_pm_ops imx390_pm_ops = {
	SET_RUNTIME_PM_OPS(imx390_runtime_suspend, imx390_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * I2C Driver, Probe & Remove
 */

static int imx390_detect(struct imx390_priv *priv)
{
	int ret;
	u8 id;

	pm_runtime_get_sync(priv->dev);

	ret = imx390_read(priv, IMX390_REG_VERSION_ROM_VERSION, &id);
	if (ret)
		goto err;

	if (id != 0x15) {
		dev_err(priv->dev, "Unknown Chip ID 0x%02x\n", id);
		ret = -ENODEV;
		goto err;
	}

	pm_runtime_put(priv->dev);

	dev_dbg(priv->dev, "Chip ID 0x%02x\n", id);

	return 0;

err:
	pm_runtime_put(priv->dev);
	return ret;
}

static const struct regmap_config imx390_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static int imx390_probe(struct i2c_client *client)
{
	struct imx390_priv *priv;
	struct v4l2_subdev *sd;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &client->dev;

	/* Acquire resources: regmap, GPIOs and clock. The GPIOs are optional. */
	priv->regmap = devm_regmap_init_i2c(client, &imx390_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->xclr_gpio =
		devm_gpiod_get_optional(priv->dev, "xclr", GPIOD_OUT_LOW);
	if (IS_ERR(priv->xclr_gpio))
		return PTR_ERR(priv->xclr_gpio);

	priv->clk = devm_clk_get(priv->dev, "inck");
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	priv->clk_rate = clk_get_rate(priv->clk);
	dev_dbg(priv->dev, "inck rate: %lu Hz\n", priv->clk_rate);

	if (priv->clk_rate < 6000000 || priv->clk_rate > 27000000)
		return -EINVAL;

	ret = imx390_power_on(priv);
	if (ret)
		return ret;

	ret = imx390_detect(priv);
	imx390_power_off(priv);
	if (ret)
		return ret;

	/* Initialize the subdev and its controls. */
	sd = &priv->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx390_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS |
		     V4L2_SUBDEV_FL_MULTIPLEXED;

	/* Initialize the media entity. */
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &priv->pad);
	if (ret < 0)
		return ret;

	pm_runtime_enable(priv->dev);
	pm_runtime_set_autosuspend_delay(priv->dev, 1000);
	pm_runtime_use_autosuspend(priv->dev);

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_media_cleanup;

	/* Finally, register the subdev. */
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0)
		goto err_free_state;

	return 0;

err_free_state:
	v4l2_subdev_cleanup(&priv->subdev);
err_media_cleanup:
	media_entity_cleanup(&priv->subdev.entity);
	return ret;
}

static int imx390_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx390_priv *priv = to_imx390(sd);

	v4l2_async_unregister_subdev(&priv->subdev);

	pm_runtime_disable(priv->dev);

	v4l2_subdev_cleanup(&priv->subdev);

	media_entity_cleanup(&priv->subdev.entity);

	return 0;
}

static const struct of_device_id imx390_dt_id[] = {
	{ .compatible = "sony,imx390" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx390_dt_id);

static struct i2c_driver imx390_i2c_driver = {
	.driver = {
		.name = "imx390",
		.of_match_table = of_match_ptr(imx390_dt_id),
		.pm = &imx390_pm_ops,
	},
	.probe_new = imx390_probe,
	.remove = imx390_remove,
};

module_i2c_driver(imx390_i2c_driver);

MODULE_DESCRIPTION("Camera Sensor Driver for Sony IMX390");
MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>");
MODULE_LICENSE("GPL v2");
