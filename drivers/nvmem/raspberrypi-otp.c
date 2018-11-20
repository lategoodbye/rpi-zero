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

struct rpi_customer_otp_cell {
	u32 index;
	u32 length;
	u32 val;
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

static int rpi_otp_write(void *context, unsigned int offset, void *val,
			  size_t bytes)
{
	struct rpi_customer_otp_cell packet;
	struct rpi_otp *otp = context;
	unsigned int bytes_written;
	u32 *buf = val;
	int ret;

	if (offset % 4)
		return -EINVAL;

	if (bytes % 4)
		return -EINVAL;

	for (bytes_written = 0; bytes_written < bytes; bytes_written += 4) {
		packet.index = offset / 4;
		packet.length = 1;
		packet.val = *buf;
		buf++;

		ret = rpi_firmware_property(otp->fw, RPI_FIRMWARE_SET_CUSTOMER_OTP,
					    &packet, sizeof(packet));

		pr_info("%s: row = %u, val = %08X, ret = %d\n", __func__, packet.index, packet.val, ret);

		if (ret)
			return ret;
	}

	return bytes;
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
	u32 val;

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

	rpi_otp_read(otp, 0, &val, sizeof(val));
	if (!val) {
		dev_info(dev, "Writing to OTP\n");
		val = 0x12345678;
		rpi_otp_write(otp, 0, &val, sizeof(val));
		val = 0x23456789;
		rpi_otp_write(otp, 4, &val, sizeof(val));
		val = 0x3456789A;
		rpi_otp_write(otp, 8, &val, sizeof(val));
		val = 0x456789AB;
		rpi_otp_write(otp, 12, &val, sizeof(val));
		val = 0x56789ABC;
		rpi_otp_write(otp, 16, &val, sizeof(val));
		val = 0x6789ABCD;
		rpi_otp_write(otp, 20, &val, sizeof(val));
		val = 0x789ABCDE;
		rpi_otp_write(otp, 24, &val, sizeof(val));
		val = 0x89ABCDEF;
		rpi_otp_write(otp, 28, &val, sizeof(val));
	}

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
