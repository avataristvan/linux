// SPDX-License-Identifier: GPL-2.0
//
// Minimal driver for the Exynos7870 S1402X audio mixer (AP interface only)
//
// Sits between I2S0 and the external codec pins. Register map and init
// sequence follow the vendor exynos-audmixer-v2.c; BT/CP/FM paths, ALC and
// the user controls are left out.

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>

/* Registers, 8 bits each, on a 4-byte stride */
#define S1402X_SOFT_RSTB	(0x00 * 4)
#define S1402X_IN1_CTL1		(0x01 * 4)
#define S1402X_IN1_CTL2		(0x02 * 4)
#define S1402X_IN2_CTL2		(0x05 * 4)
#define S1402X_IN3_CTL1		(0x07 * 4)
#define S1402X_IN3_CTL2		(0x08 * 4)
#define S1402X_IN3_CTL3		(0x09 * 4)
#define S1402X_HQ_CTL		(0x0a * 4)
#define S1402X_SLOT_L		(0x0b * 4)
#define S1402X_SLOT_R		(0x0c * 4)
#define S1402X_TSLOT		(0x0e * 4)
#define S1402X_DIG_EN		(0x0f * 4)
#define S1402X_DMIX1		(0x10 * 4)
#define S1402X_DMIX2		(0x11 * 4)
#define S1402X_DOUTMX1		(0x16 * 4)
#define S1402X_INAMP_CTL	(0x18 * 4)
#define S1402X_OUTAP1_CTL	(0x19 * 4)
#define S1402X_OUTCP1_CTL	(0x1a * 4)
#define S1402X_ALC_CTL		(0x68 * 4)
#define S1402X_ALC_SGR		(0x72 * 4)

#define SOFT_RSTB_SYS		BIT(0)
#define SOFT_RSTB_DATA		BIT(1)

/* INx_CTL2 */
#define CTL2_XFS_SHIFT		5
#define CTL2_XFS_MASK		(0x3 << CTL2_XFS_SHIFT)
#define CTL2_DL_MASK		0x3
#define XFS_32			0
#define XFS_48			1
#define XFS_64			2
#define DL_16			0
#define DL_24			3
/* LRCLK polarity "left" and I2S data format are both 0 */
#define CTL2_I2S(xfs, dl)	(((xfs) << CTL2_XFS_SHIFT) | (dl))

/* IN3_CTL1: slave, 32 BCK slot, I2S mode */
#define IN3_CTL1_DEFAULT	0x00

/* HQ_CTL */
#define HQ_CTL_HQ_EN		BIT(0)
#define HQ_CTL_MCKO_EN		BIT(2)

#define SLOT_1ST		1
#define SLOT_2ND		2
#define TSLOT_USED_2		1

#define DIG_EN_MIX_EN		BIT(3)

#define DMIX1_MIX_EN1		BIT(3)
#define DOUT_SEL1_AIF4IN	1

/* INAMP_CTL: LRCLK left, BCK normal, reserved 2, 64fs, 24 bit */
#define INAMP_CTL_DEFAULT	((2 << 4) | (XFS_64 << 2) | 2)
/* OUTAP1/OUTCP1_CTL: LRCLK left, BCK normal, 16 bit, 32fs */
#define OUTAMP_CTL_DEFAULT	0x00

/*
 * AUD_PLL runs a few Hz above 98.304 MHz, so the exact rates would round
 * down to the next divider; the vendor driver adds 100 Hz for the same reason.
 */
#define S1402X_SYSCLK_48K	(24576000 + 100)
#define S1402X_SYSCLK_192K	(49152000 + 100)

struct s1402x {
	struct device *dev;
	struct regmap *regmap;
	struct clk *sysclk;
};

static bool s1402x_readable_reg(struct device *dev, unsigned int reg)
{
	return reg <= S1402X_DMIX2 ||
	       (reg >= S1402X_DOUTMX1 && reg <= S1402X_OUTCP1_CTL) ||
	       (reg >= S1402X_ALC_CTL && reg <= S1402X_ALC_SGR);
}

static const struct regmap_config s1402x_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = S1402X_ALC_SGR,
	.readable_reg = s1402x_readable_reg,
	.writeable_reg = s1402x_readable_reg,
	.cache_type = REGCACHE_NONE,
};

/* Flush the data path after the configuration changed */
static void s1402x_reset_data(struct s1402x *mixer)
{
	regmap_update_bits(mixer->regmap, S1402X_SOFT_RSTB, SOFT_RSTB_DATA, 0);
	regmap_update_bits(mixer->regmap, S1402X_SOFT_RSTB, SOFT_RSTB_DATA,
			   SOFT_RSTB_DATA);
}

/* Vendor audmixer_init_mixer(), BCK4 used as plain BCK */
static void s1402x_init_mixer(struct s1402x *mixer)
{
	struct regmap *map = mixer->regmap;

	regmap_write(map, S1402X_SOFT_RSTB, 0);
	usleep_range(1000, 2000);
	regmap_write(map, S1402X_SOFT_RSTB, SOFT_RSTB_DATA | SOFT_RSTB_SYS);
	usleep_range(1000, 2000);

	regmap_write(map, S1402X_IN1_CTL2, CTL2_I2S(XFS_32, DL_16));
	regmap_write(map, S1402X_IN2_CTL2, CTL2_I2S(XFS_32, DL_16));
	regmap_write(map, S1402X_IN3_CTL1, IN3_CTL1_DEFAULT);
	regmap_write(map, S1402X_IN3_CTL2, CTL2_I2S(XFS_32, DL_16));
	/* 0 BCK delay, short frame sync; only matters in PCM mode */
	regmap_write(map, S1402X_IN3_CTL3, 0);
	regmap_write(map, S1402X_SLOT_L, SLOT_1ST);
	regmap_write(map, S1402X_SLOT_R, SLOT_2ND);
	regmap_write(map, S1402X_TSLOT, TSLOT_USED_2);
	regmap_write(map, S1402X_INAMP_CTL, INAMP_CTL_DEFAULT);
	regmap_write(map, S1402X_OUTAP1_CTL, OUTAMP_CTL_DEFAULT);
	regmap_write(map, S1402X_OUTCP1_CTL, OUTAMP_CTL_DEFAULT);
	regmap_write(map, S1402X_HQ_CTL, HQ_CTL_MCKO_EN);
	regmap_write(map, S1402X_DIG_EN, DIG_EN_MIX_EN);

	/*
	 * Routing that the Android HAL sets through mixer controls during
	 * headphone playback (register dump under Lineage): mix channel 1
	 * enabled at level 0, DOUT_SEL1 = AIF4IN. Without these, silence.
	 */
	regmap_write(map, S1402X_DMIX1, DMIX1_MIX_EN1);
	regmap_write(map, S1402X_DOUTMX1, DOUT_SEL1_AIF4IN);

	s1402x_reset_data(mixer);
}

static int s1402x_set_bclk_ratio(struct snd_soc_dai *dai, unsigned int ratio)
{
	struct s1402x *mixer = snd_soc_dai_get_drvdata(dai);
	unsigned int xfs;

	switch (ratio) {
	case 32:
		xfs = XFS_32;
		break;
	case 48:
		xfs = XFS_48;
		break;
	case 64:
		xfs = XFS_64;
		break;
	default:
		dev_err(mixer->dev, "unsupported bclk ratio %u\n", ratio);
		return -EINVAL;
	}

	return regmap_update_bits(mixer->regmap, S1402X_IN1_CTL2, CTL2_XFS_MASK,
				  xfs << CTL2_XFS_SHIFT);
}

static int s1402x_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct s1402x *mixer = snd_soc_dai_get_drvdata(dai);
	unsigned long sysclk;
	unsigned int dl, hq;
	int ret;

	switch (params_width(params)) {
	case 16:
		dl = DL_16;
		break;
	case 24:
		dl = DL_24;
		break;
	default:
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 48000:
		sysclk = S1402X_SYSCLK_48K;
		hq = 0;
		break;
	case 192000:
		sysclk = S1402X_SYSCLK_192K;
		hq = HQ_CTL_HQ_EN;
		break;
	default:
		return -EINVAL;
	}

	ret = clk_set_rate(mixer->sysclk, sysclk);
	if (ret) {
		dev_err(mixer->dev, "failed to set sysclk to %lu: %d\n",
			sysclk, ret);
		return ret;
	}

	regmap_update_bits(mixer->regmap, S1402X_IN1_CTL2, CTL2_DL_MASK, dl);
	regmap_update_bits(mixer->regmap, S1402X_HQ_CTL, HQ_CTL_HQ_EN, hq);
	s1402x_reset_data(mixer);

	return 0;
}

static const struct snd_soc_dai_ops s1402x_dai_ops = {
	.set_bclk_ratio = s1402x_set_bclk_ratio,
	.hw_params = s1402x_hw_params,
};

#define S1402X_RATES	(SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_192000)
#define S1402X_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_driver s1402x_dai = {
	.name = "AP0",
	.ops = &s1402x_dai_ops,
	.playback = {
		.stream_name = "AP0 Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = S1402X_RATES,
		.formats = S1402X_FORMATS,
	},
	.capture = {
		.stream_name = "AP0 Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = S1402X_RATES,
		.formats = S1402X_FORMATS,
	},
	.symmetric_rate = 1,
	.symmetric_channels = 1,
	.symmetric_sample_bits = 1,
};

static const struct snd_soc_component_driver s1402x_component = {
	.name = "s1402x",
};

static int s1402x_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rstc;
	struct s1402x *mixer;
	void __iomem *regs;
	struct clk *apb;
	unsigned int val;
	int ret;

	mixer = devm_kzalloc(dev, sizeof(*mixer), GFP_KERNEL);
	if (!mixer)
		return -ENOMEM;
	mixer->dev = dev;
	platform_set_drvdata(pdev, mixer);

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	apb = devm_clk_get_enabled(dev, "apb");
	if (IS_ERR(apb))
		return dev_err_probe(dev, PTR_ERR(apb), "failed to get apb clock\n");

	mixer->sysclk = devm_clk_get_enabled(dev, "sysclk");
	if (IS_ERR(mixer->sysclk))
		return dev_err_probe(dev, PTR_ERR(mixer->sysclk),
				     "failed to get sysclk\n");
	ret = clk_set_rate(mixer->sysclk, S1402X_SYSCLK_48K);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set sysclk rate\n");

	/* Like I2S0 and ADMA, the mixer is dead until its sysreg reset is pulsed */
	rstc = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rstc))
		return dev_err_probe(dev, PTR_ERR(rstc), "failed to get reset\n");
	reset_control_assert(rstc);
	udelay(100);
	reset_control_deassert(rstc);

	mixer->regmap = devm_regmap_init_mmio(dev, regs, &s1402x_regmap_config);
	if (IS_ERR(mixer->regmap))
		return dev_err_probe(dev, PTR_ERR(mixer->regmap),
				     "failed to init regmap\n");

	s1402x_init_mixer(mixer);

	regmap_read(mixer->regmap, S1402X_DIG_EN, &val);
	dev_info(dev, "sysclk %lu Hz, DIG_EN 0x%02x\n",
		 clk_get_rate(mixer->sysclk), val);

	return devm_snd_soc_register_component(dev, &s1402x_component,
					       &s1402x_dai, 1);
}

static const struct of_device_id s1402x_of_match[] = {
	{ .compatible = "samsung,exynos7870-s1402x" },
	{ }
};
MODULE_DEVICE_TABLE(of, s1402x_of_match);

static struct platform_driver s1402x_driver = {
	.probe = s1402x_probe,
	.driver = {
		.name = "s1402x-mixer",
		.of_match_table = s1402x_of_match,
	},
};
module_platform_driver(s1402x_driver);

MODULE_DESCRIPTION("Exynos7870 S1402X audio mixer");
MODULE_LICENSE("GPL");
