#ifndef _VC4_HDMI_H_
#define _VC4_HDMI_H_

#include <drm/drm_connector.h>
#include <media/cec.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>

#include "vc4_drv.h"

/* VC4 HDMI encoder KMS struct */
struct vc4_hdmi_encoder {
	struct vc4_encoder base;
	bool hdmi_monitor;
	bool limited_rgb_range;
};

static inline struct vc4_hdmi_encoder *
to_vc4_hdmi_encoder(struct drm_encoder *encoder)
{
	return container_of(encoder, struct vc4_hdmi_encoder, base.base);
}

struct vc4_hdmi;

struct vc4_hdmi_variant {
	/* Callback to get the resources (memory region, interrupts,
	 * clocks, etc) for that variant.
	 */
	int (*init_resources)(struct vc4_hdmi *vc4_hdmi);
};

/* HDMI audio information */
struct vc4_hdmi_audio {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu;
	struct snd_soc_dai_link_component codec;
	struct snd_soc_dai_link_component platform;
	int samplerate;
	int channels;
	struct snd_dmaengine_dai_dma_data dma_data;
	struct snd_pcm_substream *substream;
};

/* General HDMI hardware state. */
struct vc4_hdmi {
	struct platform_device *pdev;
	const struct vc4_hdmi_variant *variant;

	struct vc4_hdmi_encoder encoder;
	struct drm_connector connector;

	struct vc4_hdmi_audio audio;

	struct i2c_adapter *ddc;
	void __iomem *hdmicore_regs;
	void __iomem *hd_regs;
	int hpd_gpio;
	bool hpd_active_low;

	struct cec_adapter *cec_adap;
	struct cec_msg cec_rx_msg;
	bool cec_tx_ok;
	bool cec_irq_was_rx;

	struct clk *pixel_clock;
	struct clk *hsm_clock;

	struct debugfs_regset32 hdmi_regset;
	struct debugfs_regset32 hd_regset;
};

static inline struct vc4_hdmi *
connector_to_vc4_hdmi(struct drm_connector *connector)
{
	return container_of(connector, struct vc4_hdmi, connector);
}

static inline struct vc4_hdmi *
encoder_to_vc4_hdmi(struct drm_encoder *encoder)
{
	struct vc4_hdmi_encoder *_encoder = to_vc4_hdmi_encoder(encoder);

	return container_of(_encoder, struct vc4_hdmi, encoder);
}

#define HDMI_READ(offset) readl(vc4_hdmi->hdmicore_regs + offset)
#define HDMI_WRITE(offset, val) writel(val, vc4_hdmi->hdmicore_regs + offset)
#define HD_READ(offset) readl(vc4_hdmi->hd_regs + offset)
#define HD_WRITE(offset, val) writel(val, vc4_hdmi->hd_regs + offset)

#endif /* _VC4_HDMI_H_ */
