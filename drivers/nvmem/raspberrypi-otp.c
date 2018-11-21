// SPDX-License-Identifier: GPL-2.0+
/*
 * Raspberry Pi Customer OTP driver
 *
 * Copyright (C) 2018 Stefan Wahren <stefan.wahren@i2se.com>
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <soc/bcm2835/raspberrypi-firmware.h>

#define CUSTOMER_CELLS 8

struct rpi_otp {
	struct nvmem_device *nvmem;
	struct rpi_firmware *fw;
};

/*
 * Packet definition used by RPI_FIRMWARE_GET_CUSTOMER_OTP
 */
struct rpi_customer_otp_packet {
	u32 index;
	u32 length;
	u8 cells[CUSTOMER_CELLS * 4];
};

static int rpi_otp_read(void *context, unsigned int offset, void *val,
			size_t bytes)
{
	struct rpi_customer_otp_packet packet;
	struct rpi_otp *otp = context;
	int ret;

	packet.index = 0;
	packet.length = CUSTOMER_CELLS;
	memset(packet.cells, 0xff, sizeof(packet.cells));

	ret = rpi_firmware_property(otp->fw, RPI_FIRMWARE_GET_CUSTOMER_OTP,
				    &packet, sizeof(packet));

	if (ret)
		return ret;

	/* Request rejected by firmware */
	if (packet.index)
		return -EIO;

	memcpy(val, &packet.cells[offset], bytes);

	return 0;
}

static struct nvmem_config ocotp_config = {
	.name = "rpi-customer-otp",
	.read_only = true,
	.size = CUSTOMER_CELLS * 4,
	.stride = 4,
	.word_size = 4,
	.reg_read = rpi_otp_read,
};

static int rpi_otp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *fw_node;
	struct rpi_otp *otp;

	otp = devm_kzalloc(dev, sizeof(*otp), GFP_KERNEL);
	if (!otp)
		return -ENOMEM;

	fw_node = of_get_parent(dev->of_node);
	if (!fw_node) {
		dev_err(dev, "Missing firmware node\n");
		return -ENOENT;
	}

	otp->fw = rpi_firmware_get(fw_node);
	of_node_put(fw_node);
	if (!otp->fw)
		return -EPROBE_DEFER;

	ocotp_config.priv = otp;
	ocotp_config.dev = dev;
	otp->nvmem = devm_nvmem_register(dev, &ocotp_config);

	return PTR_ERR_OR_ZERO(otp->nvmem);
}

static const struct of_device_id rpi_otp_of_match[] = {
	{ .compatible = "raspberrypi,bcm2835-customer-otp", },
	{ /* sentinel */},
};
MODULE_DEVICE_TABLE(of, rpi_otp_of_match);

static struct platform_driver rpi_otp_driver = {
	.probe = rpi_otp_probe,
	.driver = {
		.name = "rpi-customer-otp",
		.of_match_table = rpi_otp_of_match,
	},
};
module_platform_driver(rpi_otp_driver);

MODULE_AUTHOR("Stefan Wahren <stefan.wahren@i2se.com>");
MODULE_DESCRIPTION("Raspberry Pi Customer OTP driver");
MODULE_LICENSE("GPL");
