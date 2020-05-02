// SPDX-License-Identifier: GPL-2.0+
/*
 * Raspberry Pi driver for firmware controlled clocks
 *
 * Even though clk-bcm2835 provides an interface to the hardware registers for
 * the system clocks we've had to factor out 'pllb' as the firmware 'owns' it.
 * We're not allowed to change it directly as we might race with the
 * over-temperature and under-voltage protections provided by the firmware.
 *
 * Copyright (C) 2019 Nicolas Saenz Julienne <nsaenzjulienne@suse.de>
 */

#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <soc/bcm2835/raspberrypi-firmware.h>

#define RPI_FIRMWARE_ARM_CLK_ID		0x00000003

#define RPI_FIRMWARE_STATE_ENABLE_BIT	BIT(0)
#define RPI_FIRMWARE_STATE_WAIT_BIT	BIT(1)

/*
 * Even though the firmware interface alters 'pllb' the frequencies are
 * provided as per 'pllb_arm'. We need to scale before passing them trough.
 */
#define RPI_FIRMWARE_PLLB_ARM_DIV_RATE	2

#define A2W_PLL_FRAC_BITS		20

#define NUM_FW_CLKS			16

struct raspberrypi_clk {
	struct device *dev;
	struct rpi_firmware *firmware;
	struct platform_device *cpufreq;
};

struct raspberrypi_clk_data {
	struct clk_hw hw;
	unsigned int id;

	unsigned long min_rate;
	unsigned long max_rate;

	struct raspberrypi_clk *rpi;
};

/*
 * Structure of the message passed to Raspberry Pi's firmware in order to
 * change clock rates. The 'disable_turbo' option is only available to the ARM
 * clock (pllb) which we enable by default as turbo mode will alter multiple
 * clocks at once.
 *
 * Even though we're able to access the clock registers directly we're bound to
 * use the firmware interface as the firmware ultimately takes care of
 * mitigating overheating/undervoltage situations and we would be changing
 * frequencies behind his back.
 *
 * For more information on the firmware interface check:
 * https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 */
struct raspberrypi_firmware_prop {
	__le32 id;
	__le32 val;
	__le32 disable_turbo;
} __packed;

static int raspberrypi_clock_property(struct rpi_firmware *firmware,
				      const struct raspberrypi_clk_data *data,
				      u32 tag, u32 *val)
{
	struct raspberrypi_firmware_prop msg = {
		.id = cpu_to_le32(data->id),
		.val = cpu_to_le32(*val),
		.disable_turbo = cpu_to_le32(1),
	};
	int ret;

	ret = rpi_firmware_property(firmware, tag, &msg, sizeof(msg));
	if (ret)
		return ret;

	*val = le32_to_cpu(msg.val);

	return 0;
}

static int raspberrypi_fw_is_prepared(struct clk_hw *hw)
{
	struct raspberrypi_clk_data *data =
		container_of(hw, struct raspberrypi_clk_data, hw);
	struct raspberrypi_clk *rpi = data->rpi;
	u32 val = 0;
	int ret;

	ret = raspberrypi_clock_property(rpi->firmware, data,
					 RPI_FIRMWARE_GET_CLOCK_STATE, &val);
	if (ret)
		return 0;

	return !!(val & RPI_FIRMWARE_STATE_ENABLE_BIT);
}


static unsigned long raspberrypi_fw_get_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct raspberrypi_clk_data *data =
		container_of(hw, struct raspberrypi_clk_data, hw);
	struct raspberrypi_clk *rpi = data->rpi;
	u32 val = 0;
	int ret;

	ret = raspberrypi_clock_property(rpi->firmware, data,
					 RPI_FIRMWARE_GET_CLOCK_RATE, &val);
	if (ret)
		return ret;

	return val;
}

static unsigned long raspberrypi_fw_pll_get_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	return raspberrypi_fw_get_rate(hw, parent_rate) *
		RPI_FIRMWARE_PLLB_ARM_DIV_RATE;
}

static int raspberrypi_fw_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct raspberrypi_clk_data *data =
		container_of(hw, struct raspberrypi_clk_data, hw);
	struct raspberrypi_clk *rpi = data->rpi;
	u32 _rate = rate;
	int ret;

	ret = raspberrypi_clock_property(rpi->firmware, data,
					 RPI_FIRMWARE_SET_CLOCK_RATE, &_rate);
	if (ret)
		dev_err_ratelimited(rpi->dev, "Failed to change %s frequency: %d",
				    clk_hw_get_name(hw), ret);

	return ret;
}

static int raspberrypi_fw_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long parent_rate)
{
	u32 new_rate = rate / RPI_FIRMWARE_PLLB_ARM_DIV_RATE;

	return raspberrypi_fw_set_rate(hw, new_rate, parent_rate);
}

/*
 * Sadly there is no firmware rate rounding interface. We borrowed it from
 * clk-bcm2835.
 */
static int raspberrypi_pll_determine_rate(struct clk_hw *hw,
					  struct clk_rate_request *req)
{
	struct raspberrypi_clk_data *data =
		container_of(hw, struct raspberrypi_clk_data, hw);
	u64 div, final_rate;
	u32 ndiv, fdiv;

	/* We can't use req->rate directly as it would overflow */
	final_rate = clamp(req->rate, data->min_rate, data->max_rate);

	div = (u64)final_rate << A2W_PLL_FRAC_BITS;
	do_div(div, req->best_parent_rate);

	ndiv = div >> A2W_PLL_FRAC_BITS;
	fdiv = div & ((1 << A2W_PLL_FRAC_BITS) - 1);

	final_rate = ((u64)req->best_parent_rate *
					((ndiv << A2W_PLL_FRAC_BITS) + fdiv));

	req->rate = final_rate >> A2W_PLL_FRAC_BITS;

	return 0;
}

static const struct clk_ops raspberrypi_firmware_pll_clk_ops = {
	.is_prepared = raspberrypi_fw_is_prepared,
	.recalc_rate = raspberrypi_fw_pll_get_rate,
	.set_rate = raspberrypi_fw_pll_set_rate,
	.determine_rate = raspberrypi_pll_determine_rate,
};

static struct clk_hw *raspberrypi_register_pllb(struct raspberrypi_clk *rpi)
{
	struct raspberrypi_clk_data *data;
	struct clk_init_data init = {};
	u32 min_rate = 0, max_rate = 0;
	int ret;

	data = devm_kzalloc(rpi->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);
	data->rpi = rpi;
	data->id = RPI_FIRMWARE_ARM_CLK_ID;

	/* All of the PLLs derive from the external oscillator. */
	init.parent_names = (const char *[]){ "osc" };
	init.num_parents = 1;
	init.name = "pllb";
	init.ops = &raspberrypi_firmware_pll_clk_ops;
	init.flags = CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED;

	/* Get min & max rates set by the firmware */
	ret = raspberrypi_clock_property(rpi->firmware, data,
					 RPI_FIRMWARE_GET_MIN_CLOCK_RATE,
					 &min_rate);
	if (ret) {
		dev_err(rpi->dev, "Failed to get %s min freq: %d\n",
			init.name, ret);
		return ERR_PTR(ret);
	}

	ret = raspberrypi_clock_property(rpi->firmware, data,
					 RPI_FIRMWARE_GET_MAX_CLOCK_RATE,
					 &max_rate);
	if (ret) {
		dev_err(rpi->dev, "Failed to get %s max freq: %d\n",
			init.name, ret);
		return ERR_PTR(ret);
	}

	if (!min_rate || !max_rate) {
		dev_err(rpi->dev, "Unexpected frequency range: min %u, max %u\n",
			min_rate, max_rate);
		return ERR_PTR(-EINVAL);
	}

	dev_info(rpi->dev, "CPU frequency range: min %u, max %u\n",
		 min_rate, max_rate);

	data->min_rate = min_rate * RPI_FIRMWARE_PLLB_ARM_DIV_RATE;
	data->max_rate = max_rate * RPI_FIRMWARE_PLLB_ARM_DIV_RATE;

	data->hw.init = &init;

	ret = devm_clk_hw_register(rpi->dev, &data->hw);
	if (ret)
		return ERR_PTR(ret);

	return &data->hw;
}

static struct clk_fixed_factor raspberrypi_clk_pllb_arm = {
	.mult = 1,
	.div = 2,
	.hw.init = &(struct clk_init_data) {
		.name		= "pllb_arm",
		.parent_names	= (const char *[]){ "pllb" },
		.num_parents	= 1,
		.ops		= &clk_fixed_factor_ops,
		.flags		= CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,
	},
};

static struct clk_hw *raspberrypi_register_pllb_arm(struct raspberrypi_clk *rpi)
{
	int ret;

	ret = devm_clk_hw_register(rpi->dev, &raspberrypi_clk_pllb_arm.hw);
	if (ret) {
		dev_err(rpi->dev, "Failed to initialize pllb_arm\n");
		return ERR_PTR(ret);
	}

	ret = devm_clk_hw_register_clkdev(rpi->dev,
					  &raspberrypi_clk_pllb_arm.hw,
					  NULL, "cpu0");
	if (ret) {
		dev_err(rpi->dev, "Failed to initialize clkdev\n");
		return ERR_PTR(ret);
	}

	return &raspberrypi_clk_pllb_arm.hw;
}

static long raspberrypi_fw_dumb_round_rate(struct clk_hw *hw,
					   unsigned long rate,
					   unsigned long *parent_rate)
{
	/*
	 * The firmware will do the rounding but that isn't part of
	 * the interface with the firmware, so we just do our best
	 * here.
	 */
	return rate;
}

static const struct clk_ops raspberrypi_firmware_clk_ops = {
	.is_prepared	= raspberrypi_fw_is_prepared,
	.recalc_rate	= raspberrypi_fw_get_rate,
	.round_rate	= raspberrypi_fw_dumb_round_rate,
	.set_rate	= raspberrypi_fw_set_rate,
};

static struct clk_hw *raspberrypi_clk_register(struct raspberrypi_clk *rpi,
					       unsigned int parent,
					       unsigned int id)
{
	struct raspberrypi_clk_data *data;
	struct clk_init_data init = {};
	int ret;

	if (id == RPI_FIRMWARE_ARM_CLK_ID) {
		struct clk_hw *hw;

		hw = raspberrypi_register_pllb(rpi);
		if (IS_ERR(hw)) {
			dev_err(rpi->dev, "Failed to initialize pllb, %ld\n",
				PTR_ERR(hw));
			return hw;
		}

		return raspberrypi_register_pllb_arm(rpi);
	}

	data = devm_kzalloc(rpi->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);
	data->rpi = rpi;
	data->id = id;

	init.name = devm_kasprintf(rpi->dev, GFP_KERNEL, "fw-clk-%u", id);
	init.ops = &raspberrypi_firmware_clk_ops;
	init.flags = CLK_GET_RATE_NOCACHE;

	data->hw.init = &init;

	ret = devm_clk_hw_register(rpi->dev, &data->hw);
	if (ret)
		return ERR_PTR(ret);

	return &data->hw;
}

static int raspberrypi_discover_clocks(struct raspberrypi_clk *rpi,
				       struct clk_hw_onecell_data *data)
{
	struct rpi_firmware_get_clocks_response *clks;
	int ret;

	clks = devm_kcalloc(rpi->dev, sizeof(*clks), NUM_FW_CLKS, GFP_KERNEL);
	if (!clks)
		return -ENOMEM;

	ret = rpi_firmware_property(rpi->firmware, RPI_FIRMWARE_GET_CLOCKS,
				    clks, sizeof(*clks) * NUM_FW_CLKS);
	if (ret)
		return ret;

	while (clks->id) {
		struct clk_hw *hw;

		hw = raspberrypi_clk_register(rpi, clks->parent, clks->id);
		if (IS_ERR(hw))
			return PTR_ERR(hw);

		data->hws[clks->id] = hw;
		data->num = clks->id + 1;
		clks++;
	}

	return 0;
}

static int raspberrypi_clk_probe(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *clk_data;
	struct device_node *firmware_node;
	struct device *dev = &pdev->dev;
	struct rpi_firmware *firmware;
	struct raspberrypi_clk *rpi;
	int ret;

	/*
	 * We can be probed either through the an old-fashioned
	 * platform device registration or through a DT node that is a
	 * child of the firmware node. Handle both cases.
	 */
	if (dev->of_node)
		firmware_node = of_get_parent(dev->of_node);
	else
		firmware_node = of_find_compatible_node(NULL, NULL,
							"raspberrypi,bcm2835-firmware");
	if (!firmware_node) {
		dev_err(dev, "Missing firmware node\n");
		return -ENOENT;
	}

	firmware = rpi_firmware_get(firmware_node);
	if (!firmware)
		return -EPROBE_DEFER;

	rpi = devm_kzalloc(dev, sizeof(*rpi), GFP_KERNEL);
	if (!rpi)
		return -ENOMEM;

	rpi->dev = dev;
	rpi->firmware = firmware;
	platform_set_drvdata(pdev, rpi);

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, NUM_FW_CLKS),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	ret = raspberrypi_discover_clocks(rpi, clk_data);
	if (ret)
		return ret;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					  clk_data);
	if (ret)
		return ret;

	rpi->cpufreq = platform_device_register_data(dev, "raspberrypi-cpufreq",
						     -1, NULL, 0);

	return 0;
}

static int raspberrypi_clk_remove(struct platform_device *pdev)
{
	struct raspberrypi_clk *rpi = platform_get_drvdata(pdev);

	platform_device_unregister(rpi->cpufreq);

	return 0;
}

static const struct of_device_id raspberrypi_clk_match[] = {
	{ .compatible = "raspberrypi,firmware-clocks" },
	{ },
};
MODULE_DEVICE_TABLE(of, raspberrypi_clk_match);

static struct platform_driver raspberrypi_clk_driver = {
	.driver = {
		.name = "raspberrypi-clk",
		.of_match_table = raspberrypi_clk_match,
	},
	.probe          = raspberrypi_clk_probe,
	.remove		= raspberrypi_clk_remove,
};
module_platform_driver(raspberrypi_clk_driver);

MODULE_AUTHOR("Nicolas Saenz Julienne <nsaenzjulienne@suse.de>");
MODULE_DESCRIPTION("Raspberry Pi firmware clock driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:raspberrypi-clk");
