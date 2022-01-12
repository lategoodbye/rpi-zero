// SPDX-License-Identifier: GPL-2.0-only
/*
 * BCM283x / BCM271x Unicam Capture Driver
 *
 * Copyright (C) 2017-2020 - Raspberry Pi (Trading) Ltd.
 *
 * Dave Stevenson <dave.stevenson@raspberrypi.com>
 *
 * Based on TI am437x driver by
 *   Benoit Parrot <bparrot@ti.com>
 *   Lad, Prabhakar <prabhakar.csengg@gmail.com>
 *
 * and TI CAL camera interface driver by
 *    Benoit Parrot <bparrot@ti.com>
 *
 *
 * There are two camera drivers in the kernel for BCM283x - this one
 * and bcm2835-camera (currently in staging).
 *
 * This driver directly controls the Unicam peripheral - there is no
 * involvement with the VideoCore firmware. Unicam receives CSI-2 or
 * CCP2 data and writes it into SDRAM.
 * The only potential processing options are to repack Bayer data into an
 * alternate format, and applying windowing.
 * The repacking does not shift the data, so can repack V4L2_PIX_FMT_Sxxxx10P
 * to V4L2_PIX_FMT_Sxxxx10, or V4L2_PIX_FMT_Sxxxx12P to V4L2_PIX_FMT_Sxxxx12,
 * but not generically up to V4L2_PIX_FMT_Sxxxx16. The driver will add both
 * formats where the relevant formats are defined, and will automatically
 * configure the repacking as required.
 * Support for windowing may be added later.
 *
 * It should be possible to connect this driver to any sensor with a
 * suitable output interface and V4L2 subdevice driver.
 *
 * bcm2835-camera uses the VideoCore firmware to control the sensor,
 * Unicam, ISP, and all tuner control loops. Fully processed frames are
 * delivered to the driver by the firmware. It only has sensor drivers
 * for Omnivision OV5647, and Sony IMX219 sensors.
 *
 * The two drivers are mutually exclusive for the same Unicam instance.
 * The VideoCore firmware checks the device tree configuration during boot.
 * If it finds device tree nodes called csi0 or csi1 it will block the
 * firmware from accessing the peripheral, and bcm2835-camera will
 * not be able to stream data.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>

#include "bcm2835-unicam-regs.h"

#define UNICAM_MODULE_NAME	"unicam"

/*
 * Unicam must request a minimum of 250Mhz from the VPU clock.
 * Otherwise the input FIFOs overrun and cause image corruption.
 */
#define UNICAM_MIN_VPU_CLOCK_RATE (250 * 1000 * 1000)

/*
 * Stride is a 16 bit register, but also has to be a multiple of 32.
 */
#define UNICAM_BPL_ALIGNMENT		32
#define UNICAM_MAX_BYTESPERLINE		((1 << 16) - UNICAM_BPL_ALIGNMENT)
/*
 * Max width is therefore determined by the max stride divided by
 * the number of bits per pixel. Take 32bpp as a
 * worst case.
 * No imposed limit on the height, so adopt a square image for want
 * of anything better.
 */
#define UNICAM_MAX_WIDTH		(UNICAM_MAX_BYTESPERLINE / 4)
#define UNICAM_MAX_HEIGHT		UNICAM_MAX_WIDTH
/* Define a nominal minimum image size */
#define UNICAM_MIN_WIDTH		16
#define UNICAM_MIN_HEIGHT		16
/* Default size of the embedded buffer */
#define UNICAM_EMBEDDED_SIZE		16384

/*
 * Size of the dummy buffer. Can be any size really, but the DMA
 * allocation works in units of page sizes.
 */
#define UNICAM_DUMMY_BUF_SIZE		PAGE_SIZE

#define UNICAM_SD_PAD_SINK		0
#define UNICAM_SD_PAD_SOURCE_IMAGE	1
#define UNICAM_SD_PAD_SOURCE_METADATA	2
#define UNICAM_SD_NUM_PADS		(1 + UNICAM_SD_PAD_SOURCE_METADATA)

static inline bool unicam_sd_pad_is_sink(u32 pad)
{
	/* Camera RX has 1 sink pad, and N source pads */
	return pad == 0;
}

static inline bool unicam_sd_pad_is_source(u32 pad)
{
	/* Camera RX has 1 sink pad, and N source pads */
	return pad != UNICAM_SD_PAD_SINK;
}

enum unicam_node_type {
	UNICAM_IMAGE_NODE,
	UNICAM_METADATA_NODE,
	UNICAM_MAX_NODES
};

/* -----------------------------------------------------------------------------
 * Structure definitions
 */

/*
 * struct unicam_fmt - Unicam media bus format information
 * @fourcc: V4L2 pixel format FCC identifier. 0 if n/a.
 * @unpacked_fourcc: V4L2 pixel format FCC identifier if the data is expanded
 * out to 16bpp. 0 if n/a.
 * @code: V4L2 media bus format code.
 * @depth: Bits per pixel as delivered from the source.
 * @csi_dt: CSI data type.
 * @metadata_fmt: This format only applies to the metadata pad.
 */
struct unicam_fmt {
	u32	fourcc;
	u32	unpacked_fourcc;
	u32	code;
	u8	depth;
	u8	csi_dt;
	u8	metadata_fmt:1;
};

static const struct unicam_fmt unicam_image_formats[] = {
	/* YUV Formats */
	{
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.code		= MEDIA_BUS_FMT_YUYV8_1X16,
		.depth		= 16,
		.csi_dt		= 0x1e,
	}, {
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.code		= MEDIA_BUS_FMT_UYVY8_1X16,
		.depth		= 16,
		.csi_dt		= 0x1e,
	}, {
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.code		= MEDIA_BUS_FMT_YVYU8_1X16,
		.depth		= 16,
		.csi_dt		= 0x1e,
	}, {
		.fourcc		= V4L2_PIX_FMT_VYUY,
		.code		= MEDIA_BUS_FMT_VYUY8_1X16,
		.depth		= 16,
		.csi_dt		= 0x1e,
	}, {
	/* RGB Formats */
		.fourcc		= V4L2_PIX_FMT_RGB565, /* gggbbbbb rrrrrggg */
		.code		= MEDIA_BUS_FMT_RGB565_1X16,
		.depth		= 16,
		.csi_dt		= 0x22,
	}, {
		.fourcc		= V4L2_PIX_FMT_RGB24, /* rgb */
		.code		= MEDIA_BUS_FMT_RGB888_1X24,
		.depth		= 24,
		.csi_dt		= 0x24,
	}, {
		.fourcc		= V4L2_PIX_FMT_BGR24, /* bgr */
		.code		= MEDIA_BUS_FMT_BGR888_1X24,
		.depth		= 24,
		.csi_dt		= 0x24,
	}, {
	/* Bayer Formats */
		.fourcc		= V4L2_PIX_FMT_SBGGR8,
		.code		= MEDIA_BUS_FMT_SBGGR8_1X8,
		.depth		= 8,
		.csi_dt		= 0x2a,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG8,
		.code		= MEDIA_BUS_FMT_SGBRG8_1X8,
		.depth		= 8,
		.csi_dt		= 0x2a,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG8,
		.code		= MEDIA_BUS_FMT_SGRBG8_1X8,
		.depth		= 8,
		.csi_dt		= 0x2a,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB8,
		.code		= MEDIA_BUS_FMT_SRGGB8_1X8,
		.depth		= 8,
		.csi_dt		= 0x2a,
	}, {
		.fourcc		= V4L2_PIX_FMT_SBGGR10P,
		.unpacked_fourcc = V4L2_PIX_FMT_SBGGR10,
		.code		= MEDIA_BUS_FMT_SBGGR10_1X10,
		.depth		= 10,
		.csi_dt		= 0x2b,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG10P,
		.unpacked_fourcc = V4L2_PIX_FMT_SGBRG10,
		.code		= MEDIA_BUS_FMT_SGBRG10_1X10,
		.depth		= 10,
		.csi_dt		= 0x2b,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG10P,
		.unpacked_fourcc = V4L2_PIX_FMT_SGRBG10,
		.code		= MEDIA_BUS_FMT_SGRBG10_1X10,
		.depth		= 10,
		.csi_dt		= 0x2b,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB10P,
		.unpacked_fourcc = V4L2_PIX_FMT_SRGGB10,
		.code		= MEDIA_BUS_FMT_SRGGB10_1X10,
		.depth		= 10,
		.csi_dt		= 0x2b,
	}, {
		.fourcc		= V4L2_PIX_FMT_SBGGR12P,
		.unpacked_fourcc = V4L2_PIX_FMT_SBGGR12,
		.code		= MEDIA_BUS_FMT_SBGGR12_1X12,
		.depth		= 12,
		.csi_dt		= 0x2c,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG12P,
		.unpacked_fourcc = V4L2_PIX_FMT_SGBRG12,
		.code		= MEDIA_BUS_FMT_SGBRG12_1X12,
		.depth		= 12,
		.csi_dt		= 0x2c,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG12P,
		.unpacked_fourcc = V4L2_PIX_FMT_SGRBG12,
		.code		= MEDIA_BUS_FMT_SGRBG12_1X12,
		.depth		= 12,
		.csi_dt		= 0x2c,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB12P,
		.unpacked_fourcc = V4L2_PIX_FMT_SRGGB12,
		.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
		.depth		= 12,
		.csi_dt		= 0x2c,
	}, {
		.fourcc		= V4L2_PIX_FMT_SBGGR14P,
		.unpacked_fourcc = V4L2_PIX_FMT_SBGGR14,
		.code		= MEDIA_BUS_FMT_SBGGR14_1X14,
		.depth		= 14,
		.csi_dt		= 0x2d,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG14P,
		.unpacked_fourcc = V4L2_PIX_FMT_SGBRG14,
		.code		= MEDIA_BUS_FMT_SGBRG14_1X14,
		.depth		= 14,
		.csi_dt		= 0x2d,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG14P,
		.unpacked_fourcc = V4L2_PIX_FMT_SGRBG14,
		.code		= MEDIA_BUS_FMT_SGRBG14_1X14,
		.depth		= 14,
		.csi_dt		= 0x2d,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB14P,
		.unpacked_fourcc = V4L2_PIX_FMT_SRGGB14,
		.code		= MEDIA_BUS_FMT_SRGGB14_1X14,
		.depth		= 14,
		.csi_dt		= 0x2d,
	}, {
	/* 16 bit Bayer formats could be supported. */

	/* Greyscale formats */
		.fourcc		= V4L2_PIX_FMT_GREY,
		.code		= MEDIA_BUS_FMT_Y8_1X8,
		.depth		= 8,
		.csi_dt		= 0x2a,
	}, {
		.fourcc		= V4L2_PIX_FMT_Y10P,
		.unpacked_fourcc = V4L2_PIX_FMT_Y10,
		.code		= MEDIA_BUS_FMT_Y10_1X10,
		.depth		= 10,
		.csi_dt		= 0x2b,
	}, {
		.fourcc		= V4L2_PIX_FMT_Y12P,
		.unpacked_fourcc = V4L2_PIX_FMT_Y12,
		.code		= MEDIA_BUS_FMT_Y12_1X12,
		.depth		= 12,
		.csi_dt		= 0x2c,
	}, {
		.fourcc		= V4L2_PIX_FMT_Y14P,
		.unpacked_fourcc = V4L2_PIX_FMT_Y14,
		.code		= MEDIA_BUS_FMT_Y14_1X14,
		.depth		= 14,
		.csi_dt		= 0x2d,
	},
};

static const struct unicam_fmt unicam_meta_formats[] = {
	{
		.fourcc		= V4L2_META_FMT_8,
		.code		= MEDIA_BUS_FMT_METADATA_8,
		.depth		= 8,
		.metadata_fmt	= 1,
	}
};

static const struct v4l2_mbus_framefmt default_image_format = {
	.width = 640,
	.height = 480,
	.code = MEDIA_BUS_FMT_UYVY8_1X16,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SRGB,
	.ycbcr_enc = V4L2_YCBCR_ENC_601,
	.quantization = V4L2_QUANTIZATION_LIM_RANGE,
	.xfer_func = V4L2_XFER_FUNC_SRGB,
	.flags = 0,
};

struct unicam_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static inline struct unicam_buffer *to_unicam_buffer(struct vb2_buffer *vb)
{
	return container_of(vb, struct unicam_buffer, vb.vb2_buf);
}

struct unicam_node {
	bool registered;
	bool streaming;
	unsigned int id;

	/* Pointer to the current v4l2_buffer */
	struct unicam_buffer *cur_frm;
	/* Pointer to the next v4l2_buffer */
	struct unicam_buffer *next_frm;
	/* video capture */
	const struct unicam_fmt *fmt;
	/* Used to store current pixel format */
	struct v4l2_format v_fmt;
	/* Buffer queue used in video-buf */
	struct vb2_queue buffer_queue;
	/* Queue of filled frames */
	struct list_head dma_queue;
	/* IRQ lock for DMA queue */
	spinlock_t dma_queue_lock;
	/* lock used to access this structure */
	struct mutex lock;
	/* Identifies video device for this channel */
	struct video_device video_dev;
	/* Pointer to the parent handle */
	struct unicam_device *dev;
	struct media_pad pad;
	/*
	 * Dummy buffer intended to be used by unicam
	 * if we have no other queued buffers to swap to.
	 */
	void *dummy_buf_cpu_addr;
	dma_addr_t dummy_buf_dma_addr;
};

struct unicam_device {
	struct kref kref;

	/* peripheral base address */
	void __iomem *base;
	/* clock gating base address */
	void __iomem *clk_gate_base;
	/* lp clock handle */
	struct clk *clock;
	/* vpu clock handle */
	struct clk *vpu_clock;
	/* V4l2 device */
	struct v4l2_device v4l2_dev;
	struct media_device mdev;

	/* parent device */
	struct device *dev;
	/* subdevice async Notifier */
	struct v4l2_async_notifier notifier;
	unsigned int sequence;

	/* Sensor node */
	struct {
		struct v4l2_subdev *subdev;
		struct media_pad *pad;
	} sensor;

	/* Internal subdev */
	struct {
		struct v4l2_subdev sd;
		struct media_pad pads[UNICAM_SD_NUM_PADS];
		bool streaming;
	} subdev;

	enum v4l2_mbus_type bus_type;
	/*
	 * Stores bus.mipi_csi2.flags for CSI2 sensors, or
	 * bus.mipi_csi1.strobe for CCP2.
	 */
	unsigned int bus_flags;
	unsigned int max_data_lanes;
	unsigned int active_data_lanes;

	struct media_pipeline pipe;

	struct unicam_node node[UNICAM_MAX_NODES];
};

static inline struct unicam_device *
notifier_to_unicam_device(struct v4l2_async_notifier *notifier)
{
	return container_of(notifier, struct unicam_device, notifier);
}

static inline struct unicam_device *
sd_to_unicam_device(struct v4l2_subdev *sd)
{
	return container_of(sd, struct unicam_device, subdev.sd);
}

static void unicam_release(struct kref *kref)
{
	struct unicam_device *unicam =
		container_of(kref, struct unicam_device, kref);

	if (unicam->mdev.dev)
		media_device_cleanup(&unicam->mdev);

	kfree(unicam);
}

static void unicam_put(struct unicam_device *unicam)
{
	kref_put(&unicam->kref, unicam_release);
}

static struct unicam_device *unicam_get(struct unicam_device *unicam)
{
	kref_get(&unicam->kref);
	return unicam;
}

static void unicam_node_release(struct video_device *vdev)
{
	struct unicam_node *node = video_get_drvdata(vdev);

	unicam_put(node->dev);
}

/* -----------------------------------------------------------------------------
 * Misc helper functions
 */

static inline bool is_metadata_node(struct unicam_node *node)
{
	return node->video_dev.device_caps & V4L2_CAP_META_CAPTURE;
}

static inline bool is_image_node(struct unicam_node *node)
{
	return node->video_dev.device_caps & V4L2_CAP_VIDEO_CAPTURE;
}

/* Hardware access */
static inline void unicam_clk_write(struct unicam_device *unicam, u32 val)
{
	/* Pass the CM_PASSWORD along with the value. */
	writel(val | 0x5a000000, unicam->clk_gate_base);
}

static inline u32 unicam_reg_read(struct unicam_device *unicam, u32 offset)
{
	return readl(unicam->base + offset);
}

static inline void unicam_reg_write(struct unicam_device *unicam, u32 offset, u32 val)
{
	writel(val, unicam->base + offset);
}

static inline int unicam_get_field(u32 value, u32 mask)
{
	return (value & mask) >> __ffs(mask);
}

static inline void unicam_set_field(u32 *valp, u32 field, u32 mask)
{
	u32 val = *valp;

	val &= ~mask;
	val |= (field << __ffs(mask)) & mask;
	*valp = val;
}

static inline void unicam_reg_write_field(struct unicam_device *unicam, u32 offset,
					  u32 field, u32 mask)
{
	u32 val = unicam_reg_read(unicam, offset);

	unicam_set_field(&val, field, mask);
	unicam_reg_write(unicam, offset, val);
}

/* -----------------------------------------------------------------------------
 * Format data table and helper functions
 */

/* Format setup functions */
static const struct unicam_fmt *unicam_find_format_by_code(u32 code, u32 pad)
{
	const struct unicam_fmt *formats;
	unsigned int num_formats;
	unsigned int i;

	if (pad == UNICAM_SD_PAD_SOURCE_IMAGE) {
		formats = unicam_image_formats;
		num_formats = ARRAY_SIZE(unicam_image_formats);
	} else {
		formats = unicam_meta_formats;
		num_formats = ARRAY_SIZE(unicam_meta_formats);
	}

	for (i = 0; i < num_formats; i++) {
		if (formats[i].code == code)
			return &formats[i];
	}

	return NULL;
}

static const struct unicam_fmt *unicam_find_format_by_fourcc(u32 fourcc, u32 pad)
{
	const struct unicam_fmt *formats;
	unsigned int num_formats;
	unsigned int i;

	if (pad == UNICAM_SD_PAD_SOURCE_IMAGE) {
		formats = unicam_image_formats;
		num_formats = ARRAY_SIZE(unicam_image_formats);
	} else {
		formats = unicam_meta_formats;
		num_formats = ARRAY_SIZE(unicam_meta_formats);
	}

	for (i = 0; i < num_formats; ++i) {
		if (formats[i].fourcc == fourcc)
			return &formats[i];
	}

	return NULL;
}

static int unicam_calc_format_size_bpl(struct unicam_device *unicam,
				       const struct unicam_fmt *fmt,
				       struct v4l2_format *f)
{
	u32 min_bpl;

	v4l_bound_align_image(&f->fmt.pix.width, UNICAM_MIN_WIDTH,
			      UNICAM_MAX_WIDTH, 2,
			      &f->fmt.pix.height, UNICAM_MIN_HEIGHT, 
			      UNICAM_MAX_HEIGHT, 0, 0);

	/* Unpacking always goes to 16bpp */
	if (f->fmt.pix.pixelformat == fmt->unpacked_fourcc)
		min_bpl = f->fmt.pix.width * 2;
	else
		min_bpl = f->fmt.pix.width * fmt->depth / 8;
	min_bpl = ALIGN(min_bpl, UNICAM_BPL_ALIGNMENT);

	f->fmt.pix.bytesperline = ALIGN(f->fmt.pix.bytesperline,
					UNICAM_BPL_ALIGNMENT);
	f->fmt.pix.bytesperline = clamp_t(unsigned int,
					  f->fmt.pix.bytesperline,
					  min_bpl,
					  UNICAM_MAX_BYTESPERLINE);

	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;

	dev_dbg(unicam->dev, "%s: fourcc: %08X size: %ux%u bpl:%u img_size:%u\n",
		__func__,
		f->fmt.pix.pixelformat,
		f->fmt.pix.width, f->fmt.pix.height,
		f->fmt.pix.bytesperline, f->fmt.pix.sizeimage);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Hardware handling
 */

static void unicam_wr_dma_addr(struct unicam_node *node, dma_addr_t dmaaddr,
			       unsigned int buffer_size)
{
	dma_addr_t endaddr = dmaaddr + buffer_size;

	if (node->id == UNICAM_IMAGE_NODE) {
		unicam_reg_write(node->dev, UNICAM_IBSA0, dmaaddr);
		unicam_reg_write(node->dev, UNICAM_IBEA0, endaddr);
	} else {
		unicam_reg_write(node->dev, UNICAM_DBSA0, dmaaddr);
		unicam_reg_write(node->dev, UNICAM_DBEA0, endaddr);
	}
}

static unsigned int unicam_get_lines_done(struct unicam_device *unicam)
{
	dma_addr_t start_addr, cur_addr;
	struct unicam_node *node = &unicam->node[UNICAM_IMAGE_NODE];
	unsigned int stride = node->v_fmt.fmt.pix.bytesperline;
	struct unicam_buffer *frm = node->cur_frm;

	if (!frm)
		return 0;

	start_addr = vb2_dma_contig_plane_dma_addr(&frm->vb.vb2_buf, 0);
	cur_addr = unicam_reg_read(unicam, UNICAM_IBWP);
	return (unsigned int)(cur_addr - start_addr) / stride;
}

static void unicam_schedule_next_buffer(struct unicam_node *node)
{
	struct unicam_buffer *buf;
	unsigned int size;
	dma_addr_t addr;

	buf = list_first_entry(&node->dma_queue, struct unicam_buffer, list);
	node->next_frm = buf;
	list_del(&buf->list);

	addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
	if (is_image_node(node))
		size = node->v_fmt.fmt.pix.sizeimage;
	else
		size = node->v_fmt.fmt.meta.buffersize;

	unicam_wr_dma_addr(node, addr, size);
}

static void unicam_schedule_dummy_buffer(struct unicam_node *node)
{
	struct unicam_device *unicam = node->dev;
	int node_id = is_image_node(node) ? UNICAM_IMAGE_NODE : UNICAM_METADATA_NODE;

	dev_dbg(unicam->dev, "Scheduling dummy buffer for node %d\n", node_id);

	unicam_wr_dma_addr(node, node->dummy_buf_dma_addr,
			   UNICAM_DUMMY_BUF_SIZE);

	node->next_frm = NULL;
}

static void unicam_process_buffer_complete(struct unicam_node *node,
					   unsigned int sequence)
{
	node->cur_frm->vb.field = node->v_fmt.fmt.pix.field;
	node->cur_frm->vb.sequence = sequence;

	vb2_buffer_done(&node->cur_frm->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static void unicam_queue_event_sof(struct unicam_device *unicam)
{
	struct unicam_node *node = &unicam->node[UNICAM_IMAGE_NODE];

	struct v4l2_event event = {
		.type = V4L2_EVENT_FRAME_SYNC,
		.u.frame_sync.frame_sequence = unicam->sequence,
	};

	v4l2_event_queue(&node->video_dev, &event);
}

/*
 * unicam_isr : ISR handler for unicam capture
 * @irq: irq number
 * @dev_id: dev_id ptr
 *
 * It changes status of the captured buffer, takes next buffer from the queue
 * and sets its address in unicam registers
 */
static irqreturn_t unicam_isr(int irq, void *dev)
{
	struct unicam_device *unicam = dev;
	unsigned int lines_done = unicam_get_lines_done(dev);
	unsigned int sequence = unicam->sequence;
	unsigned int i;
	u32 ista, sta;
	bool fe;
	u64 ts;

	sta = unicam_reg_read(unicam, UNICAM_STA);
	/* Write value back to clear the interrupts */
	unicam_reg_write(unicam, UNICAM_STA, sta);

	ista = unicam_reg_read(unicam, UNICAM_ISTA);
	/* Write value back to clear the interrupts */
	unicam_reg_write(unicam, UNICAM_ISTA, ista);

	dev_dbg(unicam->dev, "ISR: ISTA: 0x%X, STA: 0x%X, sequence %d, lines done %d",
		ista, sta, sequence, lines_done);

	if (!(sta & (UNICAM_IS | UNICAM_PI0)))
		return IRQ_HANDLED;

	/*
	 * Look for either the Frame End interrupt or the Packet Capture status
	 * to signal a frame end.
	 */
	fe = ista & UNICAM_FEI || sta & UNICAM_PI0;

	/*
	 * We must run the frame end handler first. If we have a valid next_frm
	 * and we get a simultaneout FE + FS interrupt, running the FS handler
	 * first would null out the next_frm ptr and we would have lost the
	 * buffer forever.
	 */
	if (fe) {
		/*
		 * Ensure we have swapped buffers already as we can't
		 * stop the peripheral. If no buffer is available, use a
		 * dummy buffer to dump out frames until we get a new buffer
		 * to use.
		 */
		for (i = 0; i < ARRAY_SIZE(unicam->node); i++) {
			if (!unicam->node[i].streaming)
				continue;

			/*
			 * If cur_frm == next_frm, it means we have not had
			 * a chance to swap buffers, likely due to having
			 * multiple interrupts occurring simultaneously (like FE
			 * + FS + LS). In this case, we cannot signal the buffer
			 * as complete, as the HW will reuse that buffer.
			 */
			if (unicam->node[i].cur_frm &&
			    unicam->node[i].cur_frm != unicam->node[i].next_frm)
				unicam_process_buffer_complete(&unicam->node[i],
							       sequence);
			unicam->node[i].cur_frm = unicam->node[i].next_frm;
		}
		unicam->sequence++;
	}

	if (ista & UNICAM_FSI) {
		/*
		 * Timestamp is to be when the first data byte was captured,
		 * aka frame start.
		 */
		ts = ktime_get_ns();
		for (i = 0; i < ARRAY_SIZE(unicam->node); i++) {
			if (!unicam->node[i].streaming)
				continue;

			if (unicam->node[i].cur_frm)
				unicam->node[i].cur_frm->vb.vb2_buf.timestamp =
								ts;
			else
				dev_dbg(unicam->v4l2_dev.dev,
					"ISR: [%d] Dropping frame, buffer not available at FS\n",
					i);
			/*
			 * Set the next frame output to go to a dummy frame
			 * if we have not managed to obtain another frame
			 * from the queue.
			 */
			unicam_schedule_dummy_buffer(&unicam->node[i]);
		}

		unicam_queue_event_sof(unicam);
	}

	/*
	 * Cannot swap buffer at frame end, there may be a race condition
	 * where the HW does not actually swap it if the new frame has
	 * already started.
	 */
	if (ista & (UNICAM_FSI | UNICAM_LCI) && !fe) {
		for (i = 0; i < ARRAY_SIZE(unicam->node); i++) {
			if (!unicam->node[i].streaming)
				continue;

			spin_lock(&unicam->node[i].dma_queue_lock);
			if (!list_empty(&unicam->node[i].dma_queue) &&
			    !unicam->node[i].next_frm)
				unicam_schedule_next_buffer(&unicam->node[i]);
			spin_unlock(&unicam->node[i].dma_queue_lock);
		}
	}

	if (unicam_reg_read(unicam, UNICAM_ICTL) & UNICAM_FCM) {
		/* Switch out of trigger mode if selected */
		unicam_reg_write_field(unicam, UNICAM_ICTL, 1, UNICAM_TFC);
		unicam_reg_write_field(unicam, UNICAM_ICTL, 0, UNICAM_FCM);
	}
	return IRQ_HANDLED;
}

static void unicam_set_packing_config(struct unicam_device *unicam)
{
	struct unicam_node *node = &unicam->node[UNICAM_IMAGE_NODE];
	u32 pack, unpack;
	u32 val;

	if (node->v_fmt.fmt.pix.pixelformat ==
	    node->fmt->fourcc) {
		unpack = UNICAM_PUM_NONE;
		pack = UNICAM_PPM_NONE;
	} else {
		switch (node->fmt->depth) {
		case 8:
			unpack = UNICAM_PUM_UNPACK8;
			break;
		case 10:
			unpack = UNICAM_PUM_UNPACK10;
			break;
		case 12:
			unpack = UNICAM_PUM_UNPACK12;
			break;
		case 14:
			unpack = UNICAM_PUM_UNPACK14;
			break;
		case 16:
			unpack = UNICAM_PUM_UNPACK16;
			break;
		default:
			unpack = UNICAM_PUM_NONE;
			break;
		}

		/* Repacking is always to 16bpp */
		pack = UNICAM_PPM_PACK16;
	}

	val = 0;
	unicam_set_field(&val, unpack, UNICAM_PUM_MASK);
	unicam_set_field(&val, pack, UNICAM_PPM_MASK);
	unicam_reg_write(unicam, UNICAM_IPIPE, val);
}

static void unicam_cfg_image_id(struct unicam_device *unicam)
{
	struct unicam_node *node = &unicam->node[UNICAM_IMAGE_NODE];

	if (unicam->bus_type == V4L2_MBUS_CSI2_DPHY) {
		/* CSI2 mode, hardcode VC 0 for now. */
		unicam_reg_write(unicam, UNICAM_IDI0,
				 (0 << 6) | node->fmt->csi_dt);
	} else {
		/* CCP2 mode */
		unicam_reg_write(unicam, UNICAM_IDI0,
				 0x80 | node->fmt->csi_dt);
	}
}

static void unicam_enable_ed(struct unicam_device *unicam)
{
	u32 val = unicam_reg_read(unicam, UNICAM_DCS);

	unicam_set_field(&val, 2, UNICAM_EDL_MASK);
	/* Do not wrap at the end of the embedded data buffer */
	unicam_set_field(&val, 0, UNICAM_DBOB);

	unicam_reg_write(unicam, UNICAM_DCS, val);
}

static void unicam_start_rx(struct unicam_device *unicam, dma_addr_t *addr)
{
	struct unicam_node *node = &unicam->node[UNICAM_IMAGE_NODE];
	int line_int_freq = node->v_fmt.fmt.pix.height >> 2;
	unsigned int size, i;
	u32 val;

	if (line_int_freq < 128)
		line_int_freq = 128;

	/* Enable lane clocks */
	val = 1;
	for (i = 0; i < unicam->active_data_lanes; i++)
		val = val << 2 | 1;
	unicam_clk_write(unicam, val);

	/* Basic init */
	unicam_reg_write(unicam, UNICAM_CTRL, UNICAM_MEM);

	/* Enable analogue control, and leave in reset. */
	val = UNICAM_AR;
	unicam_set_field(&val, 7, UNICAM_CTATADJ_MASK);
	unicam_set_field(&val, 7, UNICAM_PTATADJ_MASK);
	unicam_reg_write(unicam, UNICAM_ANA, val);
	usleep_range(1000, 2000);

	/* Come out of reset */
	unicam_reg_write_field(unicam, UNICAM_ANA, 0, UNICAM_AR);

	/* Peripheral reset */
	unicam_reg_write_field(unicam, UNICAM_CTRL, 1, UNICAM_CPR);
	unicam_reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_CPR);

	unicam_reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_CPE);

	/* Enable Rx control. */
	val = unicam_reg_read(unicam, UNICAM_CTRL);
	if (unicam->bus_type == V4L2_MBUS_CSI2_DPHY) {
		unicam_set_field(&val, UNICAM_CPM_CSI2, UNICAM_CPM_MASK);
		unicam_set_field(&val, UNICAM_DCM_STROBE, UNICAM_DCM_MASK);
	} else {
		unicam_set_field(&val, UNICAM_CPM_CCP2, UNICAM_CPM_MASK);
		unicam_set_field(&val, unicam->bus_flags, UNICAM_DCM_MASK);
	}
	/* Packet framer timeout */
	unicam_set_field(&val, 0xf, UNICAM_PFT_MASK);
	unicam_set_field(&val, 128, UNICAM_OET_MASK);
	unicam_reg_write(unicam, UNICAM_CTRL, val);

	unicam_reg_write(unicam, UNICAM_IHWIN, 0);
	unicam_reg_write(unicam, UNICAM_IVWIN, 0);

	/* AXI bus access QoS setup */
	val = unicam_reg_read(unicam, UNICAM_PRI);
	unicam_set_field(&val, 0, UNICAM_BL_MASK);
	unicam_set_field(&val, 0, UNICAM_BS_MASK);
	unicam_set_field(&val, 0xe, UNICAM_PP_MASK);
	unicam_set_field(&val, 8, UNICAM_NP_MASK);
	unicam_set_field(&val, 2, UNICAM_PT_MASK);
	unicam_set_field(&val, 1, UNICAM_PE);
	unicam_reg_write(unicam, UNICAM_PRI, val);

	unicam_reg_write_field(unicam, UNICAM_ANA, 0, UNICAM_DDL);

	/* Always start in trigger frame capture mode (UNICAM_FCM set) */
	val = UNICAM_FSIE | UNICAM_FEIE | UNICAM_FCM | UNICAM_IBOB;
	unicam_set_field(&val, line_int_freq, UNICAM_LCIE_MASK);
	unicam_reg_write(unicam, UNICAM_ICTL, val);
	unicam_reg_write(unicam, UNICAM_STA, UNICAM_STA_MASK_ALL);
	unicam_reg_write(unicam, UNICAM_ISTA, UNICAM_ISTA_MASK_ALL);

	/* tclk_term_en */
	unicam_reg_write_field(unicam, UNICAM_CLT, 2, UNICAM_CLT1_MASK);
	/* tclk_settle */
	unicam_reg_write_field(unicam, UNICAM_CLT, 6, UNICAM_CLT2_MASK);
	/* td_term_en */
	unicam_reg_write_field(unicam, UNICAM_DLT, 2, UNICAM_DLT1_MASK);
	/* ths_settle */
	unicam_reg_write_field(unicam, UNICAM_DLT, 6, UNICAM_DLT2_MASK);
	/* trx_enable */
	unicam_reg_write_field(unicam, UNICAM_DLT, 0, UNICAM_DLT3_MASK);

	unicam_reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_SOE);

	/* Packet compare setup - required to avoid missing frame ends */
	val = 0;
	unicam_set_field(&val, 1, UNICAM_PCE);
	unicam_set_field(&val, 1, UNICAM_GI);
	unicam_set_field(&val, 1, UNICAM_CPH);
	unicam_set_field(&val, 0, UNICAM_PCVC_MASK);
	unicam_set_field(&val, 1, UNICAM_PCDT_MASK);
	unicam_reg_write(unicam, UNICAM_CMP0, val);

	/* Enable clock lane and set up terminations */
	val = 0;
	if (unicam->bus_type == V4L2_MBUS_CSI2_DPHY) {
		/* CSI2 */
		unicam_set_field(&val, 1, UNICAM_CLE);
		unicam_set_field(&val, 1, UNICAM_CLLPE);
		if (unicam->bus_flags & V4L2_MBUS_CSI2_CONTINUOUS_CLOCK) {
			unicam_set_field(&val, 1, UNICAM_CLTRE);
			unicam_set_field(&val, 1, UNICAM_CLHSE);
		}
	} else {
		/* CCP2 */
		unicam_set_field(&val, 1, UNICAM_CLE);
		unicam_set_field(&val, 1, UNICAM_CLHSE);
		unicam_set_field(&val, 1, UNICAM_CLTRE);
	}
	unicam_reg_write(unicam, UNICAM_CLK, val);

	/*
	 * Enable required data lanes with appropriate terminations.
	 * The same value needs to be written to UNICAM_DATn registers for
	 * the active lanes, and 0 for inactive ones.
	 */
	val = 0;
	if (unicam->bus_type == V4L2_MBUS_CSI2_DPHY) {
		/* CSI2 */
		unicam_set_field(&val, 1, UNICAM_DLE);
		unicam_set_field(&val, 1, UNICAM_DLLPE);
		if (unicam->bus_flags & V4L2_MBUS_CSI2_CONTINUOUS_CLOCK) {
			unicam_set_field(&val, 1, UNICAM_DLTRE);
			unicam_set_field(&val, 1, UNICAM_DLHSE);
		}
	} else {
		/* CCP2 */
		unicam_set_field(&val, 1, UNICAM_DLE);
		unicam_set_field(&val, 1, UNICAM_DLHSE);
		unicam_set_field(&val, 1, UNICAM_DLTRE);
	}
	unicam_reg_write(unicam, UNICAM_DAT0, val);

	if (unicam->active_data_lanes == 1)
		val = 0;
	unicam_reg_write(unicam, UNICAM_DAT1, val);

	if (unicam->max_data_lanes > 2) {
		/*
		 * Registers UNICAM_DAT2 and UNICAM_DAT3 only valid if the
		 * instance supports more than 2 data lanes.
		 */
		if (unicam->active_data_lanes == 2)
			val = 0;
		unicam_reg_write(unicam, UNICAM_DAT2, val);

		if (unicam->active_data_lanes == 3)
			val = 0;
		unicam_reg_write(unicam, UNICAM_DAT3, val);
	}

	unicam_reg_write(unicam, UNICAM_IBLS,
			 node->v_fmt.fmt.pix.bytesperline);
	size = node->v_fmt.fmt.pix.sizeimage;
	unicam_wr_dma_addr(&unicam->node[UNICAM_IMAGE_NODE],
			   addr[UNICAM_IMAGE_NODE], size);
	unicam_set_packing_config(unicam);
	unicam_cfg_image_id(unicam);

	val = unicam_reg_read(unicam, UNICAM_MISC);
	unicam_set_field(&val, 1, UNICAM_FL0);
	unicam_set_field(&val, 1, UNICAM_FL1);
	unicam_reg_write(unicam, UNICAM_MISC, val);

	/* Enable peripheral */
	unicam_reg_write_field(unicam, UNICAM_CTRL, 1, UNICAM_CPE);

	/* Load image pointers */
	unicam_reg_write_field(unicam, UNICAM_ICTL, 1, UNICAM_LIP_MASK);

	/*
	 * Enable trigger only for the first frame to
	 * sync correctly to the FS from the source.
	 */
	unicam_reg_write_field(unicam, UNICAM_ICTL, 1, UNICAM_TFC);
}

static void unicam_start_metadata(struct unicam_device *unicam, dma_addr_t *addr)
{
	struct unicam_node *node = &unicam->node[UNICAM_METADATA_NODE];
	unsigned int size;

	dev_dbg(unicam->dev, "enable metadata dma\n");
	size = node->v_fmt.fmt.meta.buffersize;
	unicam_enable_ed(unicam);
	unicam_wr_dma_addr(node, addr[UNICAM_METADATA_NODE], size);
	unicam_reg_write_field(unicam, UNICAM_DCS, 1, UNICAM_LDP);
}

static void unicam_disable(struct unicam_device *unicam)
{
	/* Analogue lane control disable */
	unicam_reg_write_field(unicam, UNICAM_ANA, 1, UNICAM_DDL);

	/* Stop the output engine */
	unicam_reg_write_field(unicam, UNICAM_CTRL, 1, UNICAM_SOE);

	/* Disable the data lanes. */
	unicam_reg_write(unicam, UNICAM_DAT0, 0);
	unicam_reg_write(unicam, UNICAM_DAT1, 0);

	if (unicam->max_data_lanes > 2) {
		unicam_reg_write(unicam, UNICAM_DAT2, 0);
		unicam_reg_write(unicam, UNICAM_DAT3, 0);
	}

	/* Peripheral reset */
	unicam_reg_write_field(unicam, UNICAM_CTRL, 1, UNICAM_CPR);
	usleep_range(50, 100);
	unicam_reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_CPR);

	/* Disable peripheral */
	unicam_reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_CPE);

	/* Clear ED setup */
	unicam_reg_write(unicam, UNICAM_DCS, 0);

	/* Disable all lane clocks */
	unicam_clk_write(unicam, 0);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev Operations
 */

static int __unicam_subdev_set_routing(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_krouting *routing)
{
	int ret;

	ret = v4l2_subdev_routing_validate_1_to_1(routing);
	if (ret)
		return ret;

	v4l2_subdev_lock_state(state);

	ret = v4l2_subdev_set_routing(sd, state, routing);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int unicam_subdev_set_routing(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     enum v4l2_subdev_format_whence which,
				     struct v4l2_subdev_krouting *routing)
{
	struct unicam_device *unicam = sd_to_unicam_device(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && unicam->subdev.streaming)
		return -EBUSY;

	return __unicam_subdev_set_routing(sd, state, routing);
}

static int unicam_subdev_init_cfg(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = UNICAM_SD_PAD_SINK,
			.sink_stream = 0,
			.source_pad = UNICAM_SD_PAD_SOURCE_IMAGE,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	/* Initialize routing to single route to the fist source pad */
	return __unicam_subdev_set_routing(sd, state, &routing);
}

static int unicam_subdev_enum_mbus_code(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_subdev_mbus_code_enum *code)
{
	u32 pad, stream;
	int ret = -EINVAL;

	v4l2_subdev_lock_state(state);

	ret = v4l2_subdev_routing_find_opposite_end(&state->routing, code->pad,
						    code->stream, &pad,
						    &stream);
	if (ret)
		goto out;

	if (unicam_sd_pad_is_source(code->pad)) {
		/* No transcoding, source and sink codes must match. */
		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_state_get_stream_format(state, pad, stream);
		if (!fmt)
			goto out;

		if (code->index > 0)
			goto out;

		code->code = fmt->code;
	} else {
		const struct unicam_fmt *formats;
		unsigned int num_formats;

		if (pad == UNICAM_SD_PAD_SOURCE_IMAGE) {
			formats = unicam_image_formats;
			num_formats = ARRAY_SIZE(unicam_image_formats);
		} else {
			formats = unicam_meta_formats;
			num_formats = ARRAY_SIZE(unicam_meta_formats);
		}

		if (code->index >= num_formats)
			goto out;

		code->code = formats[code->index].code;
	}

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int unicam_sd_enable_streams(struct v4l2_subdev *sd, u32 pad,
				    u64 streams_mask)
{
	struct unicam_device *unicam = sd_to_unicam_device(sd);
	struct v4l2_subdev_state *state;
	u32 other_pad, other_stream;
	int ret;

	if (WARN_ON(streams_mask != 1))
		return -EINVAL;

	state = v4l2_subdev_lock_active_state(sd);

	ret = v4l2_subdev_routing_find_opposite_end(&state->routing, pad, 0,
						    &other_pad, &other_stream);

	v4l2_subdev_unlock_state(state);

	if (ret)
		return ret;

	unicam->sequence = 0;

	dev_dbg(unicam->dev, "Running with %u data lanes\n",
		unicam->active_data_lanes);

	ret = clk_set_min_rate(unicam->vpu_clock, UNICAM_MIN_VPU_CLOCK_RATE);
	if (ret) {
		dev_err(unicam->dev, "failed to set up VPU clock\n");
		return ret;
	}

	ret = clk_prepare_enable(unicam->vpu_clock);
	if (ret) {
		dev_err(unicam->dev, "Failed to enable VPU clock: %d\n", ret);
		goto err_vpu_clock;
	}

	ret = clk_set_rate(unicam->clock, 100 * 1000 * 1000);
	if (ret) {
		dev_err(unicam->dev, "failed to set up CSI clock\n");
		goto err_vpu_prepare;
	}

	ret = clk_prepare_enable(unicam->clock);
	if (ret) {
		dev_err(unicam->dev, "Failed to enable CSI clock: %d\n", ret);
		goto err_vpu_prepare;
	}

	ret = v4l2_subdev_call(unicam->sensor.subdev, video, enable_streams,
			       unicam->sensor.pad->index, BIT(other_stream));

	if (ret && ret == -ENOIOCTLCMD)
		ret = v4l2_subdev_call(unicam->sensor.subdev, video, s_stream, 1);

	if (ret) {
		dev_err(unicam->dev, "stream on failed in subdev\n");
		goto err_enable_stream;
	}

	unicam->subdev.streaming = true;

	return 0;

err_enable_stream:
	clk_disable_unprepare(unicam->clock);
err_vpu_prepare:
	clk_disable_unprepare(unicam->vpu_clock);
err_vpu_clock:
	if (clk_set_min_rate(unicam->vpu_clock, 0))
		dev_err(unicam->dev, "failed to reset the VPU clock\n");
	return ret;
}

static int unicam_sd_disable_streams(struct v4l2_subdev *sd, u32 pad,
				     u64 streams_mask)
{
	struct unicam_device *unicam = sd_to_unicam_device(sd);
	struct v4l2_subdev_state *state;
	u32 other_pad, other_stream;
	int ret;

	if (WARN_ON(streams_mask != 1))
		return -EINVAL;

	state = v4l2_subdev_lock_active_state(sd);

	ret = v4l2_subdev_routing_find_opposite_end(&state->routing, pad, 0,
						    &other_pad, &other_stream);

	v4l2_subdev_unlock_state(state);

	if (ret) {
		dev_err(unicam->dev, "disable streams failed: %d\n", ret);
		return ret;
	}

	ret = v4l2_subdev_call(unicam->sensor.subdev, video, disable_streams,
			       unicam->sensor.pad->index, BIT(other_stream));

	if (ret && ret == -ENOIOCTLCMD) {
		/* The sensor does not support disable streams. */
		if (unicam->node[UNICAM_IMAGE_NODE].streaming &&
		    other_pad == unicam->node[UNICAM_METADATA_NODE].pad.index) {
			/* We won't stop the sensor yet. */
			unicam->subdev.streaming = false;
			return 0;
		}
		/* There is no more the metadata node, we can stop. */
		ret = v4l2_subdev_call(unicam->sensor.subdev, video, s_stream, 0);
	}

	clk_disable_unprepare(unicam->clock);
	if (clk_set_min_rate(unicam->vpu_clock, 0))
		dev_err(unicam->dev, "failed to reset the VPU clock\n");
	clk_disable_unprepare(unicam->vpu_clock);

	unicam->subdev.streaming = false;

	return 0;
}

static int unicam_subdev_set_pad_format(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_subdev_format *format)
{
	struct unicam_device *unicam = sd_to_unicam_device(sd);
	struct v4l2_mbus_framefmt *sink_format, *source_format;
	const struct unicam_fmt *fmtinfo;
	int ret = 0;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && unicam->subdev.streaming)
		return -EBUSY;

	/* No transcoding, source and sink formats must match. */
	if (unicam_sd_pad_is_source(format->pad))
		return v4l2_subdev_get_fmt(sd, state, format);

	fmtinfo = unicam_find_format_by_code(format->format.code, format->pad);
	if (!fmtinfo)
		fmtinfo = &unicam_image_formats[0];

	if (format->pad == UNICAM_SD_PAD_SOURCE_IMAGE) {
		format->format.width = clamp_t(unsigned int,
					       format->format.width,
					       UNICAM_MIN_WIDTH,
					       UNICAM_MAX_WIDTH);
		format->format.height = clamp_t(unsigned int,
						format->format.height,
						UNICAM_MIN_HEIGHT,
						UNICAM_MAX_HEIGHT);
	}
	format->format.field = V4L2_FIELD_NONE;

	v4l2_subdev_lock_state(state);

	sink_format = v4l2_state_get_stream_format(state, format->pad,
						   format->stream);
	source_format = v4l2_subdev_state_get_opposite_stream_format(state,
								     format->pad,
								     format->stream);
	if (!sink_format || !source_format) {
		ret = -EINVAL;
		goto out;
	}

	*sink_format = format->format;
	*source_format = format->format;
out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int unicam_subdev_enum_frame_size(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *state,
					 struct v4l2_subdev_frame_size_enum *fse)
{
	const struct unicam_fmt *fmtinfo;
	u32 pad, stream;
	int ret = -EINVAL;

	if (fse->index > 0)
		return ret;

	v4l2_subdev_lock_state(state);

	ret = v4l2_subdev_routing_find_opposite_end(&state->routing, fse->pad,
						    fse->stream, &pad,
						    &stream);
	if (ret)
		goto out;

	if (unicam_sd_pad_is_source(fse->pad)) {
		/* No transcoding, source and sink formats must match. */
		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_state_get_stream_format(state, pad, stream);
		if (!fmt)
			goto out;

		if (fse->code != fmt->code)
			goto out;

		fse->min_width = fmt->width;
		fse->max_width = fmt->width;
		fse->min_height = fmt->height;
		fse->max_height = fmt->height;
	} else {
		fmtinfo = unicam_find_format_by_code(fse->code, pad);
		if (!fmtinfo)
			goto out;

		fse->min_width = UNICAM_MIN_WIDTH;
		fse->max_width = UNICAM_MAX_WIDTH;
		fse->min_height = UNICAM_MIN_HEIGHT;
		fse->max_height = UNICAM_MAX_HEIGHT;
	}

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static const struct v4l2_subdev_video_ops unicam_subdev_video_ops = {
	.enable_streams	= unicam_sd_enable_streams,
	.disable_streams = unicam_sd_disable_streams,
};

static const struct v4l2_subdev_pad_ops unicam_subdev_pad_ops = {
	.init_cfg		= unicam_subdev_init_cfg,
	.enum_mbus_code		= unicam_subdev_enum_mbus_code,
	.get_fmt		= v4l2_subdev_get_fmt,
	.set_fmt		= unicam_subdev_set_pad_format,
	.set_routing		= unicam_subdev_set_routing,
	.enum_frame_size	= unicam_subdev_enum_frame_size,
};

static const struct v4l2_subdev_ops unicam_subdev_ops = {
	.video		= &unicam_subdev_video_ops,
	.pad		= &unicam_subdev_pad_ops,
};

static const struct media_entity_operations unicam_subdev_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.has_route = v4l2_subdev_has_route,
};

static int unicam_init_and_register_subdev(struct unicam_device *unicam)
{
	struct v4l2_subdev *sd = &unicam->subdev.sd;
	int ret;

	v4l2_subdev_init(sd, &unicam_subdev_ops);
	v4l2_set_subdevdata(sd, unicam);
	unicam->subdev.sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	unicam->subdev.sd.entity.ops = &unicam_subdev_media_ops;
	unicam->subdev.sd.dev = unicam->dev;
	unicam->subdev.sd.owner = THIS_MODULE;
	unicam->subdev.sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_MULTIPLEXED;
	snprintf(unicam->subdev.sd.name, sizeof(unicam->subdev.sd.name), "unicam-subdev");

	unicam->subdev.pads[UNICAM_SD_PAD_SINK].flags = MEDIA_PAD_FL_SINK;

	unicam->subdev.pads[UNICAM_SD_PAD_SOURCE_IMAGE].flags = MEDIA_PAD_FL_SOURCE;
	unicam->subdev.pads[UNICAM_SD_PAD_SOURCE_METADATA].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity,
				     ARRAY_SIZE(unicam->subdev.pads), unicam->subdev.pads);
	if (ret) {
		dev_err(unicam->dev, "Could not init media entity");
		return ret;
	}

	ret = v4l2_subdev_init_finalize(sd);
	if (ret) {
		dev_err(unicam->dev, "Could not finalize init for subdev");
		return ret;
	}

	return v4l2_device_register_subdev(&unicam->v4l2_dev, sd);
}

/* -----------------------------------------------------------------------------
 * Videobuf2 Queue Operations
 */

static int unicam_queue_setup(struct vb2_queue *vq,
			      unsigned int *nbuffers,
			      unsigned int *nplanes,
			      unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct unicam_node *node = vb2_get_drv_priv(vq);
	struct unicam_device *unicam = node->dev;
	unsigned int size = is_image_node(node) ?
				node->v_fmt.fmt.pix.sizeimage :
				node->v_fmt.fmt.meta.buffersize;

	if (vq->num_buffers + *nbuffers < 3)
		*nbuffers = 3 - vq->num_buffers;

	if (*nplanes) {
		if (sizes[0] < size) {
			dev_dbg(unicam->dev, "sizes[0] %i < size %u\n", sizes[0],
				size);
			return -EINVAL;
		}
		size = sizes[0];
	}

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int unicam_buffer_prepare(struct vb2_buffer *vb)
{
	struct unicam_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct unicam_device *unicam = node->dev;
	struct unicam_buffer *buf = to_unicam_buffer(vb);
	unsigned long size;

	if (WARN_ON(!node->fmt))
		return -EINVAL;

	size = is_image_node(node) ? node->v_fmt.fmt.pix.sizeimage :
				     node->v_fmt.fmt.meta.buffersize;
	if (vb2_plane_size(vb, 0) < size) {
		dev_dbg(unicam->dev, "data will not fit into plane (%lu < %lu)\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, size);
	return 0;
}

static void unicam_buffer_queue(struct vb2_buffer *vb)
{
	struct unicam_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct unicam_buffer *buf = to_unicam_buffer(vb);

	spin_lock_irq(&node->dma_queue_lock);
	list_add_tail(&buf->list, &node->dma_queue);
	spin_unlock_irq(&node->dma_queue_lock);
}

static void unicam_return_buffers(struct unicam_node *node,
				  enum vb2_buffer_state state)
{
	struct unicam_buffer *buf, *tmp;

	list_for_each_entry_safe(buf, tmp, &node->dma_queue, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}

	if (node->cur_frm)
		vb2_buffer_done(&node->cur_frm->vb.vb2_buf,
				state);
	if (node->next_frm && node->cur_frm != node->next_frm)
		vb2_buffer_done(&node->next_frm->vb.vb2_buf,
				state);

	node->cur_frm = NULL;
	node->next_frm = NULL;
}

static int unicam_video_check_format(struct unicam_node *node)
{
	const struct v4l2_mbus_framefmt *format;
	struct unicam_device *unicam = node->dev;
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_active_state(&unicam->subdev.sd);

	format = v4l2_state_get_stream_format(state, UNICAM_SD_PAD_SOURCE_IMAGE, 0);
	if (!format) {
		ret = -EINVAL;
		goto out;
	}

	if (node->fmt->code != format->code ||
	    node->v_fmt.fmt.pix.height != format->height ||
	    node->v_fmt.fmt.pix.width != format->width ||
	    node->v_fmt.fmt.pix.field != format->field) {
		dev_dbg(unicam->dev, "(%u x %u) %08x %s != (%u x %u) %08x %s\n",
			node->v_fmt.fmt.pix.width, node->v_fmt.fmt.pix.height,
			node->fmt->code,
			v4l2_field_names[node->v_fmt.fmt.pix.field],
			format->width, format->height, format->code,
			v4l2_field_names[format->field]);
		ret = -EPIPE;
		goto out;
	}

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int unicam_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct unicam_node *node = vb2_get_drv_priv(vq);
	struct unicam_device *unicam = node->dev;
	dma_addr_t buffer_addr[UNICAM_MAX_NODES] = { 0 };
	struct unicam_buffer *buf;
	struct v4l2_subdev_state *state;
	unsigned long flags;
	unsigned int i;
	int ret;
	u32 pad, stream;
	u32 remote_pad_index = is_image_node(node) ? UNICAM_SD_PAD_SOURCE_IMAGE
						   : UNICAM_SD_PAD_SOURCE_METADATA;

	/* Look for the route for the given pad and stream. */
	state = v4l2_subdev_lock_active_state(&unicam->subdev.sd);

	ret = v4l2_subdev_routing_find_opposite_end(&state->routing,
						    remote_pad_index, 0,
						    &pad, &stream);
	if (ret)
		goto err_streaming;

	v4l2_subdev_unlock_state(state);

	dev_dbg(unicam->dev, "Starting stream 0 on pad %d on subdev %s\n",
		remote_pad_index, unicam->subdev.sd.name);

	/* The metadata node can't be started alone. */
	if (is_metadata_node(node)) {
		if (!unicam->node[UNICAM_IMAGE_NODE].streaming) {
			dev_err(unicam->dev,
				"Can't start metadata without image\n");
			return -EINVAL;
		}
		dev_dbg(unicam->dev, "starting metadata node\n");

		spin_lock_irqsave(&node->dma_queue_lock, flags);
		buf = list_first_entry(&node->dma_queue,
				       struct unicam_buffer, list);
		dev_dbg(unicam->dev, "buffer %d: %p\n", i, buf);
		node->cur_frm = buf;
		node->next_frm = buf;
		list_del(&buf->list);
		spin_unlock_irqrestore(&node->dma_queue_lock, flags);

		buffer_addr[UNICAM_METADATA_NODE] =
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		dev_dbg(unicam->dev, "buffer %d addr: %lld\n", i, buffer_addr[i]);

		unicam_start_metadata(unicam, buffer_addr);
		node->streaming = true;
		return 0;
	}

	ret = pm_runtime_get_sync(unicam->dev);
	if (ret < 0) {
		dev_err(unicam->dev, "pm_runtime_get_sync failed\n");
		goto err_streaming;
	}

	ret = media_pipeline_start(node->video_dev.entity.pads, &unicam->pipe);
	if (ret < 0) {
		dev_dbg(unicam->dev, "Failed to start media pipeline: %d\n", ret);
		goto err_pm_put;
	}

	ret = unicam_video_check_format(node);
	if (ret < 0) {
		dev_err(unicam->dev, "Video format is incorrect: %d\n", ret);
		goto error_pipeline;
	}

	dev_dbg(unicam->dev, "node %s\n", node->video_dev.name);

	spin_lock_irqsave(&node->dma_queue_lock, flags);
	buf = list_first_entry(&node->dma_queue,
			       struct unicam_buffer, list);
	dev_dbg(unicam->dev, "buffer %d: %p\n", i, buf);
	node->cur_frm = buf;
	node->next_frm = buf;
	list_del(&buf->list);
	spin_unlock_irqrestore(&node->dma_queue_lock, flags);

	buffer_addr[UNICAM_IMAGE_NODE] =
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);

	dev_dbg(unicam->dev, "Start unicam\n");
	unicam_start_rx(unicam, buffer_addr);

	dev_dbg(unicam->dev, "Enable stream\n");
	ret = v4l2_subdev_call(&unicam->subdev.sd, video, enable_streams,
			       remote_pad_index, BIT(0));
	if (ret < 0) {
		dev_err(unicam->dev, "stream on failed in subdev\n");
		goto error_pipeline;
	}

	node->streaming = true;

	return 0;

error_pipeline:
	media_pipeline_stop(node->video_dev.entity.pads);
err_pm_put:
	pm_runtime_put_sync(unicam->dev);
err_streaming:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static void unicam_stop_streaming(struct vb2_queue *vq)
{
	struct unicam_node *node = vb2_get_drv_priv(vq);
	struct unicam_device *unicam = node->dev;
	u32 remote_pad_index = is_image_node(node) ? UNICAM_SD_PAD_SOURCE_IMAGE
						   : UNICAM_SD_PAD_SOURCE_METADATA;

	node->streaming = false;

	v4l2_subdev_call(&unicam->subdev.sd, video, disable_streams,
			 remote_pad_index, BIT(0));

	/* We can stream only with the image node. */
	if (is_metadata_node(node)) {
		/*
		 * Allow the hardware to spin in the dummy buffer.
		 * This is only really needed if the embedded data pad is
		 * disabled before the image pad.
		 */
		unicam_wr_dma_addr(node, node->dummy_buf_dma_addr,
				   UNICAM_DUMMY_BUF_SIZE);
		goto dequeue_buffers;
	}

	unicam_disable(unicam);

	media_pipeline_stop(node->video_dev.entity.pads);
	pm_runtime_put(unicam->dev);

dequeue_buffers:
	/* Clear all queued buffers for the node */
	unicam_return_buffers(node, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops unicam_video_qops = {
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.queue_setup		= unicam_queue_setup,
	.buf_prepare		= unicam_buffer_prepare,
	.buf_queue		= unicam_buffer_queue,
	.start_streaming	= unicam_start_streaming,
	.stop_streaming		= unicam_stop_streaming,
};

/* -----------------------------------------------------------------------------
 *  V4L2 video device Operations
 */

static int unicam_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct unicam_node *node = video_drvdata(file);
	struct unicam_device *unicam = node->dev;

	strscpy(cap->driver, UNICAM_MODULE_NAME, sizeof(cap->driver));
	strscpy(cap->card, UNICAM_MODULE_NAME, sizeof(cap->card));

	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", dev_name(unicam->dev));

	cap->capabilities |= V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_META_CAPTURE;

	return 0;
}

static int unicam_log_status(struct file *file, void *fh)
{
	struct unicam_node *node = video_drvdata(file);
	struct unicam_device *unicam = node->dev;
	u32 reg;

	/* status for sub devices */
	v4l2_device_call_all(&unicam->v4l2_dev, 0, core, log_status);

	dev_info(unicam->dev, "-----Receiver status-----\n");
	dev_info(unicam->dev, "V4L2 width/height:   %ux%u\n",
		 node->v_fmt.fmt.pix.width, node->v_fmt.fmt.pix.height);
	dev_info(unicam->dev, "Mediabus format:     %08x\n", node->fmt->code);
	dev_info(unicam->dev, "V4L2 format:         %08x\n",
		 node->v_fmt.fmt.pix.pixelformat);
	reg = unicam_reg_read(unicam, UNICAM_IPIPE);
	dev_info(unicam->dev, "Unpacking/packing:   %u / %u\n",
		 unicam_get_field(reg, UNICAM_PUM_MASK),
		 unicam_get_field(reg, UNICAM_PPM_MASK));
	dev_info(unicam->dev, "----Live data----\n");
	dev_info(unicam->dev, "Programmed stride:   %4u\n",
		 unicam_reg_read(unicam, UNICAM_IBLS));
	dev_info(unicam->dev, "Detected resolution: %ux%u\n",
		 unicam_reg_read(unicam, UNICAM_IHSTA),
		 unicam_reg_read(unicam, UNICAM_IVSTA));
	dev_info(unicam->dev, "Write pointer:       %08x\n",
		 unicam_reg_read(unicam, UNICAM_IBWP));

	return 0;
}

static int unicam_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_FRAME_SYNC:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	}

	return v4l2_ctrl_subscribe_event(fh, sub);
}

static int unicam_enum_fmt_vid(struct file *file, void  *priv,
				   struct v4l2_fmtdesc *f)
{
	unsigned int i, index;

	for (i = 0, index = 0; i < ARRAY_SIZE(unicam_image_formats); i++) {
		if (f->mbus_code && unicam_image_formats[i].code != f->mbus_code)
			continue;

		if (index == f->index) {
			f->pixelformat = unicam_image_formats[i].fourcc;
			return 0;
		}

		index++;

		if (unicam_image_formats[i].unpacked_fourcc) {
			if (index == f->index) {
				f->pixelformat = unicam_image_formats[i].unpacked_fourcc;
				return 0;
			}
			index++;
		}
	}

	return -EINVAL;
}

static int unicam_g_fmt_vid(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct unicam_node *node = video_drvdata(file);

	*f = node->v_fmt;

	return 0;
}

static const struct unicam_fmt *
unicam_try_fmt(struct unicam_node *node, struct v4l2_format *f)
{
	struct v4l2_pix_format *v4l2_format = &f->fmt.pix;
	struct unicam_device *unicam = node->dev;
	const struct unicam_fmt *fmt;

	/*
	 * Default to the first format if the requested pixel format code isn't
	 * supported.
	 */
	fmt = unicam_find_format_by_fourcc(v4l2_format->pixelformat,
					   UNICAM_SD_PAD_SOURCE_IMAGE);
	if (!fmt) {
		fmt = &unicam_image_formats[0];
		v4l2_format->pixelformat = fmt->fourcc;
	}

	unicam_calc_format_size_bpl(unicam, fmt, f);

	if (v4l2_format->field == V4L2_FIELD_ANY)
		v4l2_format->field = V4L2_FIELD_NONE;

	dev_dbg(unicam->dev, "%s: %08x %ux%u (bytesperline %u sizeimage %u)\n",
		__func__, v4l2_format->pixelformat,
		v4l2_format->width, v4l2_format->height,
		v4l2_format->bytesperline, v4l2_format->sizeimage);

	return fmt;
}

static int unicam_try_fmt_vid(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct unicam_node *node = video_drvdata(file);

	unicam_try_fmt(node, f);
	return 0;
}

static int unicam_s_fmt_vid(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct unicam_node *node = video_drvdata(file);
	struct unicam_device *unicam = node->dev;

	if (vb2_is_busy(&node->buffer_queue)) {
		dev_dbg(unicam->dev, "%s device busy\n", __func__);
		return -EBUSY;
	}

	node->fmt = unicam_try_fmt(node, f);

	node->v_fmt = *f;

	return 0;
}

static int unicam_enum_framesizes(struct file *file, void *fh,
				  struct v4l2_frmsizeenum *fsize)
{
	struct unicam_node *node = video_drvdata(file);
	int ret = -EINVAL;

	if (fsize->index > 0)
		return ret;

	if (is_image_node(node)) {
		if (!unicam_find_format_by_fourcc(fsize->pixel_format,
						  UNICAM_SD_PAD_SOURCE_IMAGE))
			return ret;

		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise.min_width = UNICAM_MIN_WIDTH;
		fsize->stepwise.max_width = UNICAM_MAX_WIDTH;
		fsize->stepwise.step_width = 1;
		fsize->stepwise.min_height = UNICAM_MIN_HEIGHT;
		fsize->stepwise.max_height = UNICAM_MAX_HEIGHT;
		fsize->stepwise.step_height = 1;
	}
	else {
		if (!unicam_find_format_by_fourcc(fsize->pixel_format,
						  UNICAM_SD_PAD_SOURCE_METADATA))
			return ret;

		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = UNICAM_EMBEDDED_SIZE;
		fsize->discrete.height = 1;
	}

	return 0;
}

static int unicam_enum_fmt_meta(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	unsigned int i, index;

	for (i = 0, index = 0; i < ARRAY_SIZE(unicam_meta_formats); i++) {
		if (f->mbus_code && unicam_meta_formats[i].code != f->mbus_code)
			continue;
		if (!unicam_meta_formats[i].metadata_fmt)
			continue;

		if (index == f->index) {
			f->pixelformat = unicam_meta_formats[i].fourcc;
			f->type = V4L2_BUF_TYPE_META_CAPTURE;
			return 0;
		}
		index++;
	}

	return -EINVAL;
}

static int unicam_g_fmt_meta(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct unicam_node *node = video_drvdata(file);

	f->fmt.meta = node->v_fmt.fmt.meta;

	return 0;
}

static int unicam_try_fmt_meta(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	f->fmt.meta.dataformat = V4L2_META_FMT_8;
	f->fmt.meta.buffersize = UNICAM_EMBEDDED_SIZE;

	return 0;
}

static int unicam_s_fmt_meta(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct unicam_node *node = video_drvdata(file);

	unicam_try_fmt_meta(file, priv, f);

	node->v_fmt = *f;

	return 0;
}

static const struct v4l2_ioctl_ops unicam_ioctl_ops = {
	.vidioc_querycap      = unicam_querycap,
	.vidioc_enum_fmt_vid_cap  = unicam_enum_fmt_vid,
	.vidioc_g_fmt_vid_cap     = unicam_g_fmt_vid,
	.vidioc_try_fmt_vid_cap   = unicam_try_fmt_vid,
	.vidioc_s_fmt_vid_cap     = unicam_s_fmt_vid,

	.vidioc_enum_fmt_meta_cap	= unicam_enum_fmt_meta,
	.vidioc_g_fmt_meta_cap		= unicam_g_fmt_meta,
	.vidioc_try_fmt_meta_cap	= unicam_try_fmt_meta,
	.vidioc_s_fmt_meta_cap		= unicam_s_fmt_meta,

	.vidioc_enum_framesizes   = unicam_enum_framesizes,
	.vidioc_reqbufs       = vb2_ioctl_reqbufs,
	.vidioc_create_bufs   = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf   = vb2_ioctl_prepare_buf,
	.vidioc_querybuf      = vb2_ioctl_querybuf,
	.vidioc_qbuf          = vb2_ioctl_qbuf,
	.vidioc_dqbuf         = vb2_ioctl_dqbuf,
	.vidioc_expbuf        = vb2_ioctl_expbuf,
	.vidioc_streamon      = vb2_ioctl_streamon,
	.vidioc_streamoff     = vb2_ioctl_streamoff,

	.vidioc_log_status		= unicam_log_status,
	.vidioc_subscribe_event		= unicam_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* unicam capture driver file operations */
static const struct v4l2_file_operations unicam_fops = {
	.owner		= THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
};

static void unicam_set_default_format(struct unicam_node *node)
{
	if (is_image_node(node)) {
		struct v4l2_pix_format *pix_fmt = &node->v_fmt.fmt.pix;

		pix_fmt->width = default_image_format.width;
		pix_fmt->height = default_image_format.height;
		pix_fmt->field = default_image_format.field;
		pix_fmt->colorspace = default_image_format.colorspace;
		pix_fmt->ycbcr_enc = default_image_format.ycbcr_enc;
		pix_fmt->quantization = default_image_format.quantization;
		pix_fmt->xfer_func = default_image_format.xfer_func;
		pix_fmt->pixelformat = unicam_image_formats[0].fourcc;
		unicam_calc_format_size_bpl(node->dev, &unicam_image_formats[0],
					    &node->v_fmt);
		node->v_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		node->fmt = &unicam_image_formats[0];
	} else {
		const struct unicam_fmt *fmt;

		/* Fix this node format as embedded data. */
		fmt = &unicam_meta_formats[0];
		node->fmt = fmt;
		node->v_fmt.fmt.meta.dataformat = fmt->fourcc;
		node->v_fmt.fmt.meta.buffersize = UNICAM_EMBEDDED_SIZE;
		node->v_fmt.type = V4L2_BUF_TYPE_META_CAPTURE;
	}
}

static int unicam_register_node(struct unicam_device *unicam,
			 enum unicam_node_type type)
{
	struct video_device *vdev;
	struct vb2_queue *q;
	struct unicam_node *node = &unicam->node[type];
	int ret;
	u32 pad_index = type == UNICAM_IMAGE_NODE ? UNICAM_SD_PAD_SOURCE_IMAGE
						  : UNICAM_SD_PAD_SOURCE_METADATA;

	node->dev = unicam_get(unicam);
	node->id = type;

	spin_lock_init(&node->dma_queue_lock);
	mutex_init(&node->lock);

	INIT_LIST_HEAD(&node->dma_queue);

	/* Initialize the videobuf2 queue. */
	q = &node->buffer_queue;
	q->type = type == UNICAM_IMAGE_NODE ? V4L2_BUF_TYPE_VIDEO_CAPTURE
					    : V4L2_BUF_TYPE_META_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = node;
	q->ops = &unicam_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct unicam_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &node->lock;
	q->min_buffers_needed = 1;
	q->dev = unicam->dev;

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(unicam->dev, "vb2_queue_init() failed\n");
		goto err_unicam_put;
	}

	/* Initialize the video device. */
	vdev = &node->video_dev;
	vdev->release = unicam_node_release;
	vdev->fops = &unicam_fops;
	vdev->ioctl_ops = &unicam_ioctl_ops;
	vdev->v4l2_dev = &unicam->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->queue = q;
	vdev->lock = &node->lock;
	vdev->device_caps = type == UNICAM_IMAGE_NODE ?
			    V4L2_CAP_VIDEO_CAPTURE : V4L2_CAP_META_CAPTURE;
	vdev->device_caps |= V4L2_CAP_STREAMING | V4L2_CAP_IO_MC;

	/* Define the device names */
	snprintf(vdev->name, sizeof(vdev->name), "%s-%s", UNICAM_MODULE_NAME,
		 type == UNICAM_IMAGE_NODE ? "image" : "embedded");

	video_set_drvdata(vdev, node);
	if (type == UNICAM_IMAGE_NODE)
		vdev->entity.flags |= MEDIA_ENT_FL_DEFAULT;
	node->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, &node->pad);
	if (ret)
		goto err_unicam_put;

	node->dummy_buf_cpu_addr = dma_alloc_coherent(unicam->dev,
						      UNICAM_DUMMY_BUF_SIZE,
						      &node->dummy_buf_dma_addr,
						      GFP_KERNEL);
	if (!node->dummy_buf_cpu_addr) {
		dev_err(unicam->dev, "Unable to allocate dummy buffer.\n");
		ret = -ENOMEM;
		goto err_entity_cleanup;
	}

	unicam_set_default_format(node);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(unicam->dev, "Unable to register video device %s\n",
			vdev->name);
		goto err_dma_free;
	}

	node->registered = true;

	ret = media_create_pad_link(&unicam->subdev.sd.entity,
				    pad_index,
				    &node->video_dev.entity,
				    0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		/*
		 * No need for cleanup, the caller will unregister the
		 * video device, which will drop the reference on the
		 * device and trigger the cleanup.
		 */
		dev_err(unicam->dev, "Unable to create pad link for %s",
			unicam->sensor.subdev->name);
		return ret;
	}

	return 0;

err_dma_free:
	dma_free_coherent(unicam->dev, UNICAM_DUMMY_BUF_SIZE,
			  node->dummy_buf_cpu_addr,
			  node->dummy_buf_dma_addr);
err_entity_cleanup:
	media_entity_cleanup(&vdev->entity);
err_unicam_put:
	unicam_put(unicam);
	return ret;
}

static void unicam_unregister_nodes(struct unicam_device *unicam)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(unicam->node); i++) {
		struct unicam_node *node = &unicam->node[i];

		if (node->dummy_buf_cpu_addr)
			dma_free_coherent(unicam->dev, UNICAM_DUMMY_BUF_SIZE,
					  node->dummy_buf_cpu_addr,
					  node->dummy_buf_dma_addr);

		if (node->registered) {
			video_unregister_device(&node->video_dev);
			node->registered = false;
		}
	}
}

/* -----------------------------------------------------------------------------
 * V4L2 async notifier
 */

static int
unicam_async_bound(struct v4l2_async_notifier *notifier,
		   struct v4l2_subdev *subdev,
		   struct v4l2_async_subdev *asd)
{
	struct unicam_device *unicam = notifier_to_unicam_device(notifier);
	struct media_pad *sink = &unicam->subdev.sd.entity.pads[UNICAM_SD_PAD_SINK];

	unicam->sensor.subdev = subdev;
	dev_dbg(unicam->dev, "Using sensor %s for capture\n",
		subdev->name);

	unicam->sensor.pad = sink;

	return v4l2_create_fwnode_links_to_pad(subdev, sink, MEDIA_LNK_FL_ENABLED |
					       MEDIA_LNK_FL_IMMUTABLE);
}

static int unicam_async_complete(struct v4l2_async_notifier *notifier)
{
	struct unicam_device *unicam = notifier_to_unicam_device(notifier);
	int ret;

	ret = unicam_register_node(unicam, UNICAM_IMAGE_NODE);
	if (ret) {
		dev_err(unicam->dev, "Unable to register image video device.\n");
		goto unregister;
	}

	ret = unicam_register_node(unicam, UNICAM_METADATA_NODE);
	if (ret) {
		dev_err(unicam->dev, "Unable to register metadata video device.\n");
		goto unregister;
	}

	ret = v4l2_device_register_subdev_nodes(&unicam->v4l2_dev);
	if (ret) {
		dev_err(unicam->dev, "Unable to register subdev nodes.\n");
		goto unregister;
	}

	return 0;

unregister:
	unicam_unregister_nodes(unicam);
	unicam_put(unicam);

	return ret;
}

static const struct v4l2_async_notifier_operations unicam_async_ops = {
	.bound = unicam_async_bound,
	.complete = unicam_async_complete,
};

static int unicam_connect_of_subdevs(struct unicam_device *unicam)
{
	struct v4l2_fwnode_endpoint ep = { };
	struct fwnode_handle *ep_handle;
	struct v4l2_async_subdev *asd;
	unsigned int lane;
	int ret = -EINVAL;

	if (of_property_read_u32(unicam->dev->of_node, "brcm,num-data-lanes",
				 &unicam->max_data_lanes) < 0) {
		dev_err(unicam->dev, "DT property %s not set\n",
			"brcm,num-data-lanes");
		return -EINVAL;
	}

	/* Get the local endpoint and remote device. */
	ep_handle = fwnode_graph_get_endpoint_by_id(dev_fwnode(unicam->dev),
						    0, 0,
						    FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep_handle) {
		dev_err(unicam->dev, "No endpoint\n");
		return -ENODEV;
	}

	/* Parse the local endpoint and validate its configuration. */
	if (v4l2_fwnode_endpoint_alloc_parse(ep_handle, &ep)) {
		dev_err(unicam->dev, "could not parse endpoint\n");
		goto cleanup_exit;
	}

	dev_dbg(unicam->dev, "parsed local endpoint, bus_type %u\n",
		ep.bus_type);

	unicam->bus_type = ep.bus_type;

	switch (ep.bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		switch (ep.bus.mipi_csi2.num_data_lanes) {
		case 1:
		case 2:
		case 4:
			break;

		default:
			dev_err(unicam->dev, "%u data lanes not supported\n",
				ep.bus.mipi_csi2.num_data_lanes);
			goto cleanup_exit;
		}

		for (lane = 0; lane < ep.bus.mipi_csi2.num_data_lanes; lane++) {
			if (ep.bus.mipi_csi2.data_lanes[lane] != lane + 1) {
				dev_err(unicam->dev, "data lanes reordering not supported\n");
				goto cleanup_exit;
			}
		}

		if (ep.bus.mipi_csi2.num_data_lanes > unicam->max_data_lanes) {
			dev_err(unicam->dev, "endpoint requires %u data lanes when %u are supported\n",
				ep.bus.mipi_csi2.num_data_lanes,
				unicam->max_data_lanes);
		}

		unicam->active_data_lanes = ep.bus.mipi_csi2.num_data_lanes;
		unicam->bus_flags = ep.bus.mipi_csi2.flags;

		break;

	case V4L2_MBUS_CCP2:
		if (ep.bus.mipi_csi1.clock_lane != 0 ||
		    ep.bus.mipi_csi1.data_lane != 1) {
			dev_err(unicam->dev, "unsupported lanes configuration\n");
			goto cleanup_exit;
		}

		unicam->max_data_lanes = 1;
		unicam->active_data_lanes = 1;
		unicam->bus_flags = ep.bus.mipi_csi1.strobe;
		break;

	default:
		/* Unsupported bus type */
		dev_err(unicam->dev, "unsupported bus type %u\n",
			ep.bus_type);
		goto cleanup_exit;
	}

	dev_dbg(unicam->dev, "%s bus, %u data lanes, flags=0x%08x\n",
		unicam->bus_type == V4L2_MBUS_CSI2_DPHY ? "CSI-2" : "CCP2",
		unicam->active_data_lanes, unicam->bus_flags);

	/* Initialize and register the async notifier. */
	v4l2_async_nf_init(&unicam->notifier);

	asd = v4l2_async_nf_add_fwnode_remote(&unicam->notifier, ep_handle,
					      struct v4l2_async_subdev);

	fwnode_handle_put(ep_handle);
	ep_handle = NULL;

	if (IS_ERR(asd)) {
		ret = PTR_ERR(asd);
		goto cleanup_exit;
	}

	unicam->notifier.ops = &unicam_async_ops;

	ret = v4l2_async_nf_register(&unicam->v4l2_dev, &unicam->notifier);
	if (ret) {
		dev_err(unicam->dev, "Error registering device notifier: %d\n", ret);
		goto cleanup_exit;
	}

	return 0;

cleanup_exit:
	v4l2_fwnode_endpoint_free(&ep);
	fwnode_handle_put(ep_handle);

	return ret;
}

static int unicam_media_dev_init(struct unicam_device *unicam)
{
	int ret = 0;

	unicam->mdev.dev = unicam->dev;
	strscpy(unicam->mdev.model, UNICAM_MODULE_NAME,
		sizeof(unicam->mdev.model));
	strscpy(unicam->mdev.serial, "", sizeof(unicam->mdev.serial));
	snprintf(unicam->mdev.bus_info, sizeof(unicam->mdev.bus_info),
		 "platform:%s", dev_name(unicam->dev));
	unicam->mdev.hw_revision = 0;

	media_device_init(&unicam->mdev);

	unicam->v4l2_dev.mdev = &unicam->mdev;

	ret = v4l2_device_register(unicam->dev, &unicam->v4l2_dev);
	if (ret < 0) {
		dev_err(unicam->dev,
			"Unable to register v4l2 device\n");
		return ret;
	}

	ret = media_device_register(&unicam->mdev);
	if (ret < 0) {
		dev_err(unicam->dev,
			"Unable to register media-controller device\n");
		goto err_v4l2_unregister;
	}

	return 0;

err_v4l2_unregister:
	v4l2_device_unregister(&unicam->v4l2_dev);
	return ret;
}

static int unicam_probe(struct platform_device *pdev)
{
	struct unicam_device *unicam;
	int ret = 0;

	unicam = kzalloc(sizeof(*unicam), GFP_KERNEL);
	if (!unicam)
		return -ENOMEM;

	kref_init(&unicam->kref);
	unicam->dev = &pdev->dev;
	/* set the driver data in platform device */
	platform_set_drvdata(pdev, unicam);

	unicam->base = devm_platform_ioremap_resource_byname(pdev, "unicam");
	if (IS_ERR(unicam->base)) {
		ret = PTR_ERR(unicam->base);
		goto err_unicam_put;
	}

	unicam->clk_gate_base = devm_platform_ioremap_resource_byname(pdev, "cmi");
	if (IS_ERR(unicam->clk_gate_base)) {
		ret = PTR_ERR(unicam->clk_gate_base);
		goto err_unicam_put;
	}

	unicam->clock = devm_clk_get(&pdev->dev, "lp");
	if (IS_ERR(unicam->clock)) {
		dev_err(unicam->dev, "Failed to get lp clock\n");
		ret = PTR_ERR(unicam->clock);
		goto err_unicam_put;
	}

	unicam->vpu_clock = devm_clk_get(&pdev->dev, "vpu");
	if (IS_ERR(unicam->vpu_clock)) {
		dev_err(unicam->dev, "Failed to get vpu clock\n");
		ret = PTR_ERR(unicam->vpu_clock);
		goto err_unicam_put;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		ret = -EINVAL;
		goto err_unicam_put;
	}

	ret = devm_request_irq(&pdev->dev, ret, unicam_isr, 0,
			       "unicam_capture0", unicam);
	if (ret) {
		dev_err(&pdev->dev, "Unable to request interrupt\n");
		ret = -EINVAL;
		goto err_unicam_put;
	}

	ret = unicam_media_dev_init(unicam);
	if (ret) {
		dev_err(unicam->dev,
			"Unable to initialize media device\n");
		goto err_unicam_put;
	}

	ret = unicam_init_and_register_subdev(unicam);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register internal subdev\n");
		goto err_media_unregister;
	}

	ret = unicam_connect_of_subdevs(unicam);
	if (ret) {
		dev_err(&pdev->dev, "Failed to connect subdevs\n");
		goto err_subdev_unregister;
	}

	/* Enable the block power domain */
	pm_runtime_enable(&pdev->dev);

	return 0;

err_subdev_unregister:
	v4l2_subdev_cleanup(&unicam->subdev.sd);
err_media_unregister:
	media_device_unregister(&unicam->mdev);
err_unicam_put:
	unicam_put(unicam);

	return ret;
}

static int unicam_remove(struct platform_device *pdev)
{
	struct unicam_device *unicam = platform_get_drvdata(pdev);

	unicam_unregister_nodes(unicam);
	v4l2_device_unregister(&unicam->v4l2_dev);
	media_device_unregister(&unicam->mdev);
	v4l2_async_nf_unregister(&unicam->notifier);
	unicam_put(unicam);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id unicam_of_match[] = {
	{ .compatible = "brcm,bcm2835-unicam", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, unicam_of_match);

static struct platform_driver unicam_driver = {
	.probe		= unicam_probe,
	.remove		= unicam_remove,
	.driver = {
		.name	= UNICAM_MODULE_NAME,
		.of_match_table = of_match_ptr(unicam_of_match),
	},
};

module_platform_driver(unicam_driver);

MODULE_AUTHOR("Dave Stevenson <dave.stevenson@raspberrypi.com>");
MODULE_DESCRIPTION("BCM2835 Unicam driver");
MODULE_LICENSE("GPL");
