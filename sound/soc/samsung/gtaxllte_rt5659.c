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
#include <linux/slab.h>

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
 * (ASRC_2 = 0x1001 under Lineage) run the stereo DAC and ADC filters through
 * the I2S1 ASRC; rt5659 powers the ASRC widgets itself once these are set.
 */
static int gtaxllte_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *codec = snd_soc_rtd_to_codec(rtd, 1)->component;
	int ret;

	ret = snd_soc_component_update_bits(codec, RT5659_ASRC_2,
					    RT5659_DA_STO_T_MASK |
					    RT5659_AD_STO1_T_MASK,
					    1 << RT5659_DA_STO_T_SFT |
					    1 << RT5659_AD_STO1_T_SFT);
	if (ret < 0)
		return ret;

	/* DAC1 at -20 dB (122 of 175) instead of 0 dB, to spare the ears */
	ret = snd_soc_component_update_bits(codec, RT5659_DAC1_DIG_VOL,
					    0xffff, 0x7a7a);
	if (ret < 0)
		return ret;

	/*
	 * As the vendor init list: without this the combo jack's mic contact
	 * stays disconnected from IN1 and the headset mic records silence.
	 */
	ret = snd_soc_component_write(codec, RT5659_EJD_CTRL_1, 0x70c0);
	if (ret < 0)
		return ret;

	/*
	 * Headphones and main mic by default; there is no jack detection yet,
	 * so switch with "Speaker"/"Headphone"/"Main Mic"/"Headset Mic Switch".
	 */
	ret = snd_soc_dapm_disable_pin(snd_soc_card_to_dapm(rtd->card),
				       "Headset Mic");
	if (ret < 0)
		return ret;

	return snd_soc_dapm_disable_pin(snd_soc_card_to_dapm(rtd->card),
					"Speaker");
}

/*
 * Default routing as Lineage sets it for both headphones and speaker:
 * DAC1 -> Stereo DAC Mixer (L1 to left, R1 to right) -> DAC L1/R1
 * (STO_DAC_MIXER 0x2a8a, A_DAC_MUX 0x000c); the speaker stays silent with
 * the analog DAC fed from DAC1 directly. These are DAPM controls: DAPM reads
 * a mixer path's state only when the codec routes are added, so a plain
 * register write in the link init leaves the path disconnected. Set them
 * through the controls once the card exists, so DAPM updates its paths.
 */
static const struct {
	const char *name;
	bool is_enum;
	unsigned int val;
} gtaxllte_defaults[] = {
	{ "Stereo DAC MIXL DAC L1 Switch", false, 1 },
	{ "Stereo DAC MIXR DAC R1 Switch", false, 1 },
	{ "DAC L1 Source", true, 1 },	/* Stereo DAC Mixer */
	{ "DAC R1 Source", true, 1 },
	{ "HPO L Playback Switch", false, 1 },
	{ "HPO R Playback Switch", false, 1 },
	/*
	 * Mics as Lineage records them: main IN3 -> BST3, headset IN1 -> BST1,
	 * both -> RECMIX1 -> ADC1; the mic pin switches pick the active one.
	 */
	{ "RECMIX1L BST3 Switch", false, 1 },
	{ "RECMIX1R BST3 Switch", false, 1 },
	{ "RECMIX1L BST1 Switch", false, 1 },
	{ "RECMIX1R BST1 Switch", false, 1 },
	{ "Stereo1 ADC Source", true, 0 },	/* ADC1 */
	{ "Stereo1 ADC1 Source", true, 1 },	/* ADC */
	{ "Stereo1 ADC MIXL ADC1 Switch", false, 1 },
	{ "Stereo1 ADC MIXR ADC1 Switch", false, 1 },
	{ "IN3 Boost Volume", false, 53 },
	{ "IN1 Boost Volume", false, 53 },
};

static int gtaxllte_set_defaults(struct snd_soc_card *card)
{
	struct snd_ctl_elem_value *uc;
	struct snd_kcontrol *kctl;
	int i, ret = 0;

	uc = kzalloc(sizeof(*uc), GFP_KERNEL);
	if (!uc)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(gtaxllte_defaults); i++) {
		kctl = snd_soc_card_get_kcontrol(card, gtaxllte_defaults[i].name);
		if (!kctl) {
			dev_err(card->dev, "missing control %s\n",
				gtaxllte_defaults[i].name);
			ret = -ENOENT;
			break;
		}

		memset(uc, 0, sizeof(*uc));
		if (gtaxllte_defaults[i].is_enum)
			uc->value.enumerated.item[0] = gtaxllte_defaults[i].val;
		else
			uc->value.integer.value[0] = gtaxllte_defaults[i].val;

		ret = kctl->put(kctl, uc);
		if (ret < 0) {
			dev_err(card->dev, "failed to set %s: %d\n",
				gtaxllte_defaults[i].name, ret);
			break;
		}
		ret = 0;
	}

	kfree(uc);
	return ret;
}

static const struct snd_soc_ops gtaxllte_ops = {
	.hw_params = gtaxllte_hw_params,
};

static const struct snd_soc_dapm_widget gtaxllte_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static const struct snd_kcontrol_new gtaxllte_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
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

	/* Not fatal: the card works, userspace can still set the routing */
	ret = gtaxllte_set_defaults(card);
	if (ret)
		dev_warn(dev, "default routing not set: %d\n", ret);

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
