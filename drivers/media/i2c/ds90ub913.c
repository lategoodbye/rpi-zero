// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Texas Instruments DS90UB913 video serializer
 *
 * Based on a driver from Luca Ceresoli <luca@lucaceresoli.net>
 *
 * Copyright (c) 2019 Luca Ceresoli <luca@lucaceresoli.net>
 * Copyright (c) 2021 Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include <media/v4l2-subdev.h>

#define UB913_PAD_SINK			0
#define UB913_PAD_SOURCE		1

/*
 * UB913 has 4 gpios, but gpios 3 and 4 are reserved for external oscillator
 * mode. Thus we only support 2 gpios for now.
 */
#define UB913_NUM_GPIOS			2

#define UB913_REG_RESET_CTL			0x01
#define UB913_REG_RESET_CTL_DIGITAL_RESET_1	BIT(1)
#define UB913_REG_RESET_CTL_DIGITAL_RESET_0	BIT(0)

#define UB913_REG_GENERAL_CFG			0x03
#define UB913_REG_MODE_SEL			0x05

#define UB913_REG_GPIO_CFG(n)			(0x0d + (n))
#define UB913_REG_GPIO_CFG_ENABLE(n)		BIT(0 + (n) * 4)
#define UB913_REG_GPIO_CFG_DIR_INPUT(n)		BIT(1 + (n) * 4)
#define UB913_REG_GPIO_CFG_REMOTE_EN(n)		BIT(2 + (n) * 4)
#define UB913_REG_GPIO_CFG_OUT_VAL(n)		BIT(3 + (n) * 4)
#define UB913_REG_GPIO_CFG_MASK(n)		(0xf << ((n) * 4))

#define UB913_REG_SCL_HIGH_TIME			0x11
#define UB913_REG_SCL_LOW_TIME			0x12

struct ub913_data {
	struct i2c_client	*client;
	struct regmap		*regmap;

	u32			gpio_func[UB913_NUM_GPIOS];

	struct gpio_chip	gpio_chip;
	char			gpio_chip_name[64];

	struct v4l2_subdev	sd;
	struct media_pad	pads[2];

	struct v4l2_async_notifier	notifier;

	struct v4l2_subdev	*source_sd;

	bool			streaming;

	struct device_node	*tx_ep_np;
};

static inline struct ub913_data *sd_to_ub913(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ub913_data, sd);
}

static int ub913_read(const struct ub913_data *priv, u8 reg, u8 *val)
{
	unsigned int v;
	int ret;

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret < 0) {
		dev_err(&priv->client->dev,
			"Cannot read register 0x%02x: %d!\n", reg, ret);
		return ret;
	}

	*val = v;
	return 0;
}

static int ub913_write(const struct ub913_data *priv, u8 reg, u8 val)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret < 0)
		dev_err(&priv->client->dev,
			"Cannot write register 0x%02x: %d!\n", reg, ret);

	return ret;
}

/*
 * GPIO chip
 */
static int ub913_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int ub913_gpio_direction_out(struct gpio_chip *gc, unsigned int offset,
				    int value)
{
	struct ub913_data *priv = gpiochip_get_data(gc);
	unsigned int reg_idx;
	unsigned int field_idx;
	int ret;

	reg_idx = offset / 2;
	field_idx = offset % 2;

	ret = regmap_update_bits(
		priv->regmap, UB913_REG_GPIO_CFG(reg_idx),
		UB913_REG_GPIO_CFG_MASK(field_idx),
		UB913_REG_GPIO_CFG_ENABLE(field_idx) |
			(value ? UB913_REG_GPIO_CFG_OUT_VAL(field_idx) : 0));

	return ret;
}

static void ub913_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	ub913_gpio_direction_out(gc, offset, value);
}

static int ub913_gpio_of_xlate(struct gpio_chip *gc,
			       const struct of_phandle_args *gpiospec,
			       u32 *flags)
{
	if (flags)
		*flags = gpiospec->args[1];

	return gpiospec->args[0];
}

static int ub913_gpiochip_probe(struct ub913_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct gpio_chip *gc = &priv->gpio_chip;
	int ret;

	/* Initialize GPIOs 0 and 1 to local control, tri-state */
	ub913_write(priv, UB913_REG_GPIO_CFG(0), 0);

	scnprintf(priv->gpio_chip_name, sizeof(priv->gpio_chip_name), "%s",
		  dev_name(dev));

	gc->label = priv->gpio_chip_name;
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->base = -1;
	gc->can_sleep = 1;
	gc->ngpio = UB913_NUM_GPIOS;
	gc->get_direction = ub913_gpio_get_direction;
	gc->direction_output = ub913_gpio_direction_out;
	gc->set = ub913_gpio_set;
	gc->of_xlate = ub913_gpio_of_xlate;
	gc->of_node = priv->client->dev.of_node;
	gc->of_gpio_n_cells = 2;

	ret = gpiochip_add_data(gc, priv);
	if (ret) {
		dev_err(dev, "Failed to add GPIOs: %d\n", ret);
		return ret;
	}

	return 0;
}

static void ub913_gpiochip_remove(struct ub913_data *priv)
{
	gpiochip_remove(&priv->gpio_chip);
}

/*
 * Reset via registers (useful from remote).
 * Note: the procedure is undocumented, but this one seems to work.
 */
static void ub913_soft_reset(struct ub913_data *priv)
{
	struct device *dev = &priv->client->dev;
	int retries;

	ub913_write(priv, UB913_REG_RESET_CTL,
		    UB913_REG_RESET_CTL_DIGITAL_RESET_0);

	usleep_range(10000, 30000);

	retries = 10;
	while (retries-- > 0) {
		int ret;
		u8 v;

		ret = ub913_read(priv, UB913_REG_RESET_CTL, &v);

		if (ret >= 0 &&
		    (v & UB913_REG_RESET_CTL_DIGITAL_RESET_0) == 0) {
			dev_dbg(dev, "reset done\n");
			break;
		}

		usleep_range(1000, 3000);
	}

	if (retries == 0)
		dev_err(dev, "reset timeout\n");
}

static int ub913_parse_dt(struct ub913_data *priv)
{
	struct device_node *np = priv->client->dev.of_node;
	struct device *dev = &priv->client->dev;
	int ret;

	if (!np) {
		dev_err(dev, "OF: no device tree node!\n");
		return -ENOENT;
	}

	/* optional, if absent all GPIO pins are unused */
	ret = of_property_read_u32_array(np, "gpio-functions", priv->gpio_func,
					 ARRAY_SIZE(priv->gpio_func));
	if (ret && ret != -EINVAL)
		dev_err(dev, "DT: invalid gpio-functions property (%d)", ret);

	return 0;
}

static const struct regmap_config ub913_regmap_config = {
	.name = "ds90ub913",
	.reg_bits = 8,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_DEFAULT,
	.val_format_endian = REGMAP_ENDIAN_DEFAULT,
};

/*
 * V4L2
 */

static int ub913_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ub913_data *priv = sd_to_ub913(sd);
	int ret;

	priv->streaming = enable;

	ret = v4l2_subdev_call(priv->source_sd, video, s_stream, enable);
	if (ret && enable)
		priv->streaming = false;

	return ret;
}

static const struct v4l2_subdev_video_ops ub913_video_ops = {
	.s_stream = ub913_s_stream,
};

static int _ub913_set_routing(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_krouting *routing)
{
	const struct v4l2_mbus_framefmt format = {
		.width = 640,
		.height = 480,
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_LIM_RANGE,
		.xfer_func = V4L2_XFER_FUNC_SRGB,
	};
	int ret;

	/*
	 * Note: we can only support up to V4L2_FRAME_DESC_ENTRY_MAX, until
	 * frame desc is made dynamically allocated.
	 */

	if (routing->num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
		return -EINVAL;

	ret = v4l2_subdev_routing_validate_1_to_1(routing);
	if (ret)
		return ret;

	v4l2_subdev_lock_state(state);

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);

	v4l2_subdev_unlock_state(state);

	if (ret)
		return ret;

	return 0;
}

static int ub913_set_routing(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     enum v4l2_subdev_format_whence which,
			     struct v4l2_subdev_krouting *routing)
{
	struct ub913_data *priv = sd_to_ub913(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming)
		return -EBUSY;

	return _ub913_set_routing(sd, state, routing);
}

static int ub913_get_source_frame_desc(struct ub913_data *priv,
				       struct v4l2_mbus_frame_desc *desc)
{
	struct media_pad *pad;
	int ret;

	pad = media_entity_remote_pad(&priv->pads[UB913_PAD_SINK]);
	if (!pad)
		return -EPIPE;

	ret = v4l2_subdev_call(priv->source_sd, pad, get_frame_desc, pad->index,
			       desc);
	if (ret)
		return ret;

	return 0;
}

static int ub913_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_frame_desc *fd)
{
	struct ub913_data *priv = sd_to_ub913(sd);
	const struct v4l2_subdev_krouting *routing;
	struct v4l2_mbus_frame_desc source_fd;
	struct v4l2_subdev_route *route;
	struct v4l2_subdev_state *state;
	int ret = 0;

	if (pad != 1) /* first tx pad */
		return -EINVAL;

	ret = ub913_get_source_frame_desc(priv, &source_fd);
	if (ret)
		return ret;

	state = v4l2_subdev_lock_active_state(sd);

	routing = &state->routing;

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_PARALLEL;

	for_each_active_route(routing, route) {
		unsigned int j;

		if (route->source_pad != pad)
			continue;

		for (j = 0; j < source_fd.num_entries; ++j)
			if (source_fd.entry[j].stream == route->sink_stream)
				break;

		if (j == source_fd.num_entries) {
			dev_err(&priv->client->dev,
				"Failed to find stream from source frame desc\n");
			ret = -EPIPE;
			goto out;
		}

		fd->entry[fd->num_entries].stream = route->source_stream;

		fd->entry[fd->num_entries].flags =
			V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
		fd->entry[fd->num_entries].length = source_fd.entry[j].length;
		fd->entry[fd->num_entries].pixelcode =
			source_fd.entry[j].pixelcode;

		fd->num_entries++;
	}

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ub913_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *format)
{
	struct ub913_data *priv = sd_to_ub913(sd);
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming)
		return -EBUSY;

	/* No transcoding, source and sink formats must match. */
	if (format->pad == 1)
		return v4l2_subdev_get_fmt(sd, state, format);

	v4l2_subdev_lock_state(state);

	/* Set sink format */
	fmt = v4l2_state_get_stream_format(state, format->pad, format->stream);
	if (!fmt) {
		ret = -EINVAL;
		goto out;
	}

	*fmt = format->format;

	/* Propagate to source format */
	fmt = v4l2_subdev_state_get_opposite_stream_format(state, format->pad,
						    format->stream);
	if (!fmt) {
		ret = -EINVAL;
		goto out;
	}

	*fmt = format->format;

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ub913_init_cfg(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = 0,
			.sink_stream = 0,
			.source_pad = 1,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	return _ub913_set_routing(sd, state, &routing);
}

static const struct v4l2_subdev_pad_ops ub913_pad_ops = {
	.set_routing	= ub913_set_routing,
	.get_frame_desc	= ub913_get_frame_desc,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ub913_set_fmt,
	.init_cfg = ub913_init_cfg,
};

static const struct v4l2_subdev_ops ub913_subdev_ops = {
	.video = &ub913_video_ops,
	.pad = &ub913_pad_ops,
};

static const struct media_entity_operations ub913_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ub913_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *source_subdev,
			      struct v4l2_async_subdev *asd)
{
	struct ub913_data *priv = sd_to_ub913(notifier->sd);
	struct device *dev = &priv->client->dev;
	unsigned int src_pad;
	int ret;

	dev_dbg(dev, "Bind %s\n", source_subdev->name);

	ret = media_entity_get_fwnode_pad(&source_subdev->entity,
					  source_subdev->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev, "Failed to find pad for %s\n",
			source_subdev->name);
		return ret;
	}

	priv->source_sd = source_subdev;
	src_pad = ret;

	ret = media_create_pad_link(&source_subdev->entity, src_pad,
				    &priv->sd.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev, "Unable to link %s:%u -> %s:0\n",
			source_subdev->name, src_pad, priv->sd.name);
		return ret;
	}

	dev_dbg(dev, "Bound %s:%u\n", source_subdev->name, src_pad);

	dev_dbg(dev, "All subdevs bound\n");

	return 0;
}

static void ub913_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *source_subdev,
				struct v4l2_async_subdev *asd)
{
	struct ub913_data *priv = sd_to_ub913(notifier->sd);
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unbind %s\n", source_subdev->name);
}

static const struct v4l2_async_notifier_operations ub913_notify_ops = {
	.bound = ub913_notify_bound,
	.unbind = ub913_notify_unbind,
};

static int ub913_v4l2_notifier_register(struct ub913_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct v4l2_async_subdev *asd;
	struct device_node *ep_node;
	int ret;

	dev_dbg(dev, "register async notif\n");

	ep_node = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	if (!ep_node) {
		dev_err(dev, "No graph endpoint\n");
		return -ENODEV;
	}

	v4l2_async_notifier_init(&priv->notifier);

	asd = v4l2_async_notifier_add_fwnode_remote_subdev(
		&priv->notifier, of_fwnode_handle(ep_node),
		struct v4l2_async_subdev);

	of_node_put(ep_node);

	if (IS_ERR(asd)) {
		dev_err(dev, "Failed to add subdev: %ld", PTR_ERR(asd));
		v4l2_async_notifier_cleanup(&priv->notifier);
		return PTR_ERR(asd);
	}

	priv->notifier.ops = &ub913_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&priv->sd, &priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_notifier_cleanup(&priv->notifier);
		return ret;
	}

	return 0;
}

static void ub913_v4l2_notifier_unregister(struct ub913_data *priv)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unregister async notif\n");

	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
}

static int ub913_i2c_init(struct ub913_data *priv)
{
	/* i2c fast mode */
	u32 scl_high = 900; /* ns */
	u32 scl_low = 1600; /* ns */
	u32 ref = 24000000; /* TODO: get refclock properly */
	int ret = 0;

	scl_high = div64_u64((u64)scl_high * ref, 1000000000);
	scl_low = div64_u64((u64)scl_low * ref, 1000000000);

	ret = ub913_write(priv, UB913_REG_SCL_HIGH_TIME, scl_high);
	if (ret)
		return ret;

	ret = ub913_write(priv, UB913_REG_SCL_LOW_TIME, scl_low);
	if (ret)
		return ret;

	return 0;
}

static int ub913_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ub913_data *priv;
	int ret;

	dev_dbg(dev, "probing, addr 0x%02x\n", client->addr);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	priv->regmap = devm_regmap_init_i2c(client, &ub913_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Failed to init regmap\n");
		return PTR_ERR(priv->regmap);
	}

	ret = ub913_parse_dt(priv);
	if (ret)
		return ret;

	ub913_soft_reset(priv);

	ret = ub913_i2c_init(priv);
	if (ret) {
		dev_err(dev, "i2c init failed: %d\n", ret);
		return ret;
	}

	ret = ub913_gpiochip_probe(priv);
	if (ret) {
		dev_err(dev, "Failed to init gpiochip\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&priv->sd, priv->client, &ub913_subdev_ops);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_MULTIPLEXED;
	priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->sd.entity.ops = &ub913_entity_ops;

	priv->pads[0].flags = MEDIA_PAD_FL_SINK;
	priv->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->sd.entity, 2, priv->pads);
	if (ret) {
		dev_err(dev, "Failed to init pads\n");
		goto err_gpiochip_remove;
	}

	priv->tx_ep_np = of_graph_get_endpoint_by_regs(dev->of_node, 1, 0);
	if (priv->tx_ep_np)
		priv->sd.fwnode = of_fwnode_handle(priv->tx_ep_np);

	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret)
		goto err_entity_cleanup;

	ret = ub913_v4l2_notifier_register(priv);
	if (ret) {
		dev_err(dev, "v4l2 subdev notifier register failed: %d\n", ret);
		goto err_free_state;
	}

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret) {
		dev_err(dev, "v4l2_async_register_subdev error: %d\n", ret);
		goto err_unreg_notif;
	}

	dev_dbg(dev, "Successfully probed\n");

	return 0;

err_unreg_notif:
	ub913_v4l2_notifier_unregister(priv);
err_free_state:
	v4l2_subdev_cleanup(&priv->sd);
err_entity_cleanup:
	if (priv->tx_ep_np)
		of_node_put(priv->tx_ep_np);

	media_entity_cleanup(&priv->sd.entity);
err_gpiochip_remove:
	ub913_gpiochip_remove(priv);

	return ret;
}

static int ub913_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ub913_data *priv = sd_to_ub913(sd);

	dev_dbg(&client->dev, "Removing\n");

	ub913_v4l2_notifier_unregister(priv);
	v4l2_async_unregister_subdev(&priv->sd);

	v4l2_subdev_cleanup(&priv->sd);

	if (priv->tx_ep_np)
		of_node_put(priv->tx_ep_np);

	media_entity_cleanup(&priv->sd.entity);

	ub913_gpiochip_remove(priv);

	return 0;
}

static const struct i2c_device_id ub913_id[] = { { "ds90ub913a-q1", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, ub913_id);

#ifdef CONFIG_OF
static const struct of_device_id ub913_dt_ids[] = {
	{ .compatible = "ti,ds90ub913a-q1", },
	{}
};
MODULE_DEVICE_TABLE(of, ub913_dt_ids);
#endif

static struct i2c_driver ds90ub913_driver = {
	.probe_new	= ub913_probe,
	.remove		= ub913_remove,
	.id_table	= ub913_id,
	.driver = {
		.name	= "ds90ub913a",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ub913_dt_ids),
	},
};

module_i2c_driver(ds90ub913_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Texas Instruments DS90UB913 serializer driver");
MODULE_AUTHOR("Luca Ceresoli <luca@lucaceresoli.net>");
MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>");
