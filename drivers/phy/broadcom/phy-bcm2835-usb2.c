/*
 * BCM2835 USB2 Phy Driver
 *
 * Copyright (C) 2017 Stefan Wahren <stefan.wahren@i2se.com>
 *
 * Based on phy-bcm-kona-usb2.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>


static int bcm2835_usb_phy_power_on(struct phy *gphy)
{
	return 0;
}

static int bcm2835_usb_phy_power_off(struct phy *gphy)
{
	return 0;
}

static const struct phy_ops ops = {
	.power_on	= bcm2835_usb_phy_power_on,
	.power_off	= bcm2835_usb_phy_power_off,
	.owner		= THIS_MODULE,
};

static int bcm2835_usb2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy *gphy;
	struct phy_provider *phy_provider;

	gphy = devm_phy_create(dev, NULL, &ops);
	if (IS_ERR(gphy))
		return PTR_ERR(gphy);

	/* The BCM2835 PHY supports an 8-bit wide UTMI interface */
	phy_set_bus_width(gphy, 8);

	phy_provider = devm_of_phy_provider_register(dev,
			of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id bcm2835_usb2_dt_ids[] = {
	{ .compatible = "brcm,bcm2835-phy" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, bcm2835_usb2_dt_ids);

static struct platform_driver bcm2835_usb2_driver = {
	.probe		= bcm2835_usb2_probe,
	.driver		= {
		.name	= "bcm2835-usb2",
		.of_match_table = bcm2835_usb2_dt_ids,
	},
};

module_platform_driver(bcm2835_usb2_driver);

MODULE_ALIAS("platform:bcm2835-usb2");
MODULE_AUTHOR("Stefan Wahren <stefan.wahren@i2se.com>");
MODULE_DESCRIPTION("BCM2835 USB 2.0 PHY driver");
MODULE_LICENSE("GPL v2");
