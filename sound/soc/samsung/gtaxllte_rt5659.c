// SPDX-License-Identifier: GPL-2.0
//
// Machine driver for the Samsung Galaxy Tab A 10.1 LTE (SM-T585, gtaxllte)
//
// One link as in the vendor universal7870-rt5659.c: I2S0 drives the S1402X
// mixer and the RT5659 codec together (multi-codec). The codec PLL1 runs from
// the 26 MHz PMU CLKOUT or BCLK1, as simple-card cannot set up a codec PLL.

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/rt5659.h"
#include "i2s.h"

#define GTAXLLTE_MCLK_RATE	26000000
#define GTAXLLTE_SYSCLK_48K	24576000
#define GTAXLLTE_BFS_48K	32
#define GTAXLLTE_RFS_48K	512

/* 0: PLL1 from MCLK; else PLL1 from BCLK1 at this many BCLKs per frame */
static unsigned int codec_bfs;
module_param(codec_bfs, uint, 0644);
MODULE_PARM_DESC(codec_bfs, "codec PLL1 source: 0 = MCLK, 32/64 = BCLK1 ratio");

static int gtaxllte_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *mixer_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 1);
	struct device *dev = rtd->card->dev;
	int ret;

	/* Only 48 kHz for now; 192 kHz would clock PLL1 from BCLK1 */
	if (params_rate(params) != 48000) {
		dev_err(dev, "unsupported rate %u\n", params_rate(params));
		return -EINVAL;
	}

	/*
	 * As vendor: CDCLK out at 512fs. The vendor RCLKSRC_1 call is left out:
	 * the DT assigns the codec clock as RCLK mux parent already, and
	 * switching RCLK source here re-gets op_clk and trips a refcount warning.
	 */
	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
				     GTAXLLTE_RFS_48K, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(dev, "failed to set I2S CDCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_BCLK,
				     GTAXLLTE_BFS_48K);
	if (ret < 0) {
		dev_err(dev, "failed to set I2S BFS: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_bclk_ratio(mixer_dai, GTAXLLTE_BFS_48K);
	if (ret < 0) {
		dev_err(dev, "failed to set mixer bclk ratio: %d\n", ret);
		return ret;
	}

	/*
	 * From the 26 MHz MCLK, as the vendor does at 48 kHz, PLL1 only reaches
	 * 24.5818 MHz (audible clicks); from BCLK1 it would be synchronous, but
	 * the codec-side BCLK ratio is not known yet (bring-up experiment).
	 */
	if (codec_bfs)
		ret = snd_soc_dai_set_pll(codec_dai, 0, RT5659_PLL1_S_BCLK1,
					  params_rate(params) * codec_bfs,
					  GTAXLLTE_SYSCLK_48K);
	else
		ret = snd_soc_dai_set_pll(codec_dai, 0, RT5659_PLL1_S_MCLK,
					  GTAXLLTE_MCLK_RATE, GTAXLLTE_SYSCLK_48K);
	if (ret < 0) {
		dev_err(dev, "failed to set codec PLL: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5659_SCLK_S_PLL1,
				     GTAXLLTE_SYSCLK_48K, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(dev, "failed to set codec sysclk: %d\n", ret);

	return ret;
}

/*
 * The 26 MHz MCLK is not synchronous to the I2S clocks, so like the vendor
 * (ASRC_2 = 0x1001 under Lineage) run the stereo DAC filter through the
 * I2S1 ASRC; rt5659 powers the ASRC widget itself once this field is set.
 */
static int gtaxllte_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *codec = snd_soc_rtd_to_codec(rtd, 1)->component;

	return snd_soc_component_update_bits(codec, RT5659_ASRC_2,
					     RT5659_DA_STO_T_MASK,
					     1 << RT5659_DA_STO_T_SFT);
}

static const struct snd_soc_ops gtaxllte_ops = {
	.hw_params = gtaxllte_hw_params,
};

static const struct snd_soc_dapm_widget gtaxllte_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const struct snd_kcontrol_new gtaxllte_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

SND_SOC_DAILINK_DEFS(primary,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link gtaxllte_dai_link = {
	.name = "Primary",
	.stream_name = "Primary",
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		   SND_SOC_DAIFMT_CBC_CFC,
	.init = gtaxllte_link_init,
	.ops = &gtaxllte_ops,
	SND_SOC_DAILINK_REG(primary),
};

static struct snd_soc_card gtaxllte_card = {
	.name = "gtaxllte",
	.owner = THIS_MODULE,
	.dai_link = &gtaxllte_dai_link,
	.num_links = 1,
	.dapm_widgets = gtaxllte_widgets,
	.num_dapm_widgets = ARRAY_SIZE(gtaxllte_widgets),
	.controls = gtaxllte_controls,
	.num_controls = ARRAY_SIZE(gtaxllte_controls),
};

static int gtaxllte_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &gtaxllte_card;
	struct snd_soc_dai_link *link = &gtaxllte_dai_link;
	struct device *dev = &pdev->dev;
	struct device_node *cpu, *codec;
	int ret;

	card->dev = dev;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret)
		return ret;

	ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
	if (ret)
		return ret;

	cpu = of_get_child_by_name(dev->of_node, "cpu");
	if (!cpu)
		return dev_err_probe(dev, -EINVAL, "missing cpu node\n");
	ret = snd_soc_of_get_dlc(cpu, NULL, link->cpus, 0);
	of_node_put(cpu);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse cpu dai\n");
	link->platforms->of_node = link->cpus->of_node;

	/* Mixer first, codec second: hw_params relies on that order */
	codec = of_get_child_by_name(dev->of_node, "codec");
	if (!codec) {
		ret = dev_err_probe(dev, -EINVAL, "missing codec node\n");
		goto put_cpu;
	}
	ret = snd_soc_of_get_dai_link_codecs(dev, codec, link);
	of_node_put(codec);
	if (ret) {
		dev_err_probe(dev, ret, "failed to parse codec dais\n");
		goto put_cpu;
	}
	if (link->num_codecs != 2) {
		ret = dev_err_probe(dev, -EINVAL, "need mixer and codec dais\n");
		goto put_codecs;
	}

	ret = devm_snd_soc_register_card(dev, card);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register card\n");
		goto put_codecs;
	}

	return 0;

put_codecs:
	snd_soc_of_put_dai_link_codecs(link);
put_cpu:
	of_node_put(link->cpus->of_node);
	return ret;
}

static void gtaxllte_remove(struct platform_device *pdev)
{
	snd_soc_of_put_dai_link_codecs(&gtaxllte_dai_link);
	of_node_put(gtaxllte_dai_link.cpus->of_node);
}

static const struct of_device_id gtaxllte_of_match[] = {
	{ .compatible = "samsung,gtaxllte-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, gtaxllte_of_match);

static struct platform_driver gtaxllte_driver = {
	.probe = gtaxllte_probe,
	.remove = gtaxllte_remove,
	.driver = {
		.name = "gtaxllte-audio",
		.of_match_table = gtaxllte_of_match,
		.pm = &snd_soc_pm_ops,
	},
};
module_platform_driver(gtaxllte_driver);

MODULE_DESCRIPTION("ASoC machine driver for Samsung SM-T585 (gtaxllte)");
MODULE_LICENSE("GPL");
