// SPDX-License-Identifier: GPL-2.0-only
/*
 * Fuel gauge driver for the Silicon Mitus SM5703
 *
 * Read-only: the gauge keeps its state on the battery, and the battery
 * model tables are programmed by the vendor kernel. This driver only
 * reports what the gauge already computes.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#define SM5703_FG_REG_DEVICE_ID		0x00
#define SM5703_FG_REG_SOC		0x05
#define SM5703_FG_REG_OCV		0x06
#define SM5703_FG_REG_VOLTAGE		0x07
#define SM5703_FG_REG_CURRENT		0x08
#define SM5703_FG_REG_OP_STATUS		0x10
#define SM5703_FG_REG_END		0xff

/* Low byte of OP_STATUS once the battery model has been programmed */
#define SM5703_FG_INIT_DONE		0x07

/*
 * Voltage and current words: bits 10:8 integer part (V or A),
 * bits 7:0 fraction in 1/256, bit 15 sign (current only).
 */
#define SM5703_FG_SIGN			BIT(15)
#define SM5703_FG_MAGNITUDE		GENMASK(10, 0)

struct sm5703_fg {
	struct regmap		*regmap;
	struct power_supply	*psy;
};

static int sm5703_fg_read_fixed(struct sm5703_fg *fg, unsigned int reg,
				int *val)
{
	unsigned int word;
	int ret;

	ret = regmap_read(fg->regmap, reg, &word);
	if (ret)
		return ret;

	/* 1/256 units -> micro units */
	*val = DIV_ROUND_CLOSEST((word & SM5703_FG_MAGNITUDE) * 1000000, 256);
	if (word & SM5703_FG_SIGN)
		*val = -*val;

	return 0;
}

static int sm5703_fg_get_capacity(struct sm5703_fg *fg, int *val)
{
	unsigned int word;
	int ret;

	ret = regmap_read(fg->regmap, SM5703_FG_REG_SOC, &word);
	if (ret)
		return ret;

	/* Bits 15:8 integer percent, 7:0 fraction */
	*val = clamp(word >> 8, 0U, 100U);

	return 0;
}

static int sm5703_fg_get_status(struct sm5703_fg *fg, int *val)
{
	int current_ua, ret;

	ret = sm5703_fg_read_fixed(fg, SM5703_FG_REG_CURRENT, &current_ua);
	if (ret)
		return ret;

	/* No charger driver yet: positive current means charging */
	*val = current_ua > 0 ? POWER_SUPPLY_STATUS_CHARGING :
				POWER_SUPPLY_STATUS_DISCHARGING;

	return 0;
}

static int sm5703_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5703_fg *fg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return sm5703_fg_get_status(fg, &val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return sm5703_fg_read_fixed(fg, SM5703_FG_REG_VOLTAGE,
					    &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		return sm5703_fg_read_fixed(fg, SM5703_FG_REG_OCV,
					    &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return sm5703_fg_read_fixed(fg, SM5703_FG_REG_CURRENT,
					    &val->intval);
	case POWER_SUPPLY_PROP_CAPACITY:
		return sm5703_fg_get_capacity(fg, &val->intval);
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property sm5703_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

/* 16-bit registers, transferred low byte first */
static const struct regmap_config sm5703_fg_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 16,
	.val_format_endian	= REGMAP_ENDIAN_LITTLE,
	.max_register		= SM5703_FG_REG_END,
};

static const struct power_supply_desc sm5703_fg_desc = {
	.name		= "sm5703-fuel-gauge",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= sm5703_fg_get_property,
	.properties	= sm5703_fg_props,
	.num_properties	= ARRAY_SIZE(sm5703_fg_props),
};

static int sm5703_fg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	unsigned int id, status;
	struct sm5703_fg *fg;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->regmap = devm_regmap_init_i2c(client, &sm5703_fg_regmap_config);
	if (IS_ERR(fg->regmap))
		return dev_err_probe(dev, PTR_ERR(fg->regmap),
				     "Failed to initialize regmap\n");

	ret = regmap_read(fg->regmap, SM5703_FG_REG_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read device ID\n");

	ret = regmap_read(fg->regmap, SM5703_FG_REG_OP_STATUS, &status);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read status\n");

	if ((status & 0xff) != SM5703_FG_INIT_DONE)
		dev_warn(dev, "gauge not initialised (status 0x%04x), capacity unreliable\n",
			 status);

	dev_info(dev, "SM5703 fuel gauge, device ID 0x%04x\n", id);

	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.drv_data = fg;

	fg->psy = devm_power_supply_register(dev, &sm5703_fg_desc, &psy_cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy),
				     "Failed to register power supply\n");

	return 0;
}

static const struct i2c_device_id sm5703_fg_id[] = {
	{ .name = "sm5703-fuel-gauge" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5703_fg_id);

static const struct of_device_id sm5703_fg_of_match[] = {
	{ .compatible = "siliconmitus,sm5703-fuel-gauge", },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5703_fg_of_match);

static struct i2c_driver sm5703_fg_driver = {
	.driver = {
		.name = "sm5703-fuel-gauge",
		.of_match_table = sm5703_fg_of_match,
	},
	.probe = sm5703_fg_probe,
	.id_table = sm5703_fg_id,
};
module_i2c_driver(sm5703_fg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5703 fuel gauge driver");
MODULE_LICENSE("GPL");
