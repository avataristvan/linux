// SPDX-License-Identifier: GPL-2.0-only
/*
 * Fuel gauge driver for the Silicon Mitus SM5703
 *
 * Read-only: the gauge keeps its state on the battery, and the battery
 * model tables are programmed by the vendor kernel. This driver only
 * reports what the gauge already computes.
 *
 * The charger owns the charge limit and the battery thermistor; their
 * properties are forwarded so that userspace finds them on the battery.
 */

#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

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

/* Net current below this is noise, not charging */
#define SM5703_FG_DEADBAND_UA		20000

/* Capacity and status are announced at this interval when they change */
#define SM5703_FG_POLL_MS		30000

struct sm5703_fg {
	struct regmap		*regmap;
	struct power_supply	*psy;
	struct delayed_work	poll_work;
	int			capacity;
	int			status;
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

/* State of charge in 1/256 percent: bits 15:8 integer, 7:0 fraction */
static int sm5703_fg_read_soc(struct sm5703_fg *fg, unsigned int *soc)
{
	int ret;

	ret = regmap_read(fg->regmap, SM5703_FG_REG_SOC, soc);
	if (ret)
		return ret;

	*soc = min(*soc, 100U * 256);

	return 0;
}

static int sm5703_fg_get_capacity(struct sm5703_fg *fg, int *val)
{
	unsigned int soc;
	int ret;

	ret = sm5703_fg_read_soc(fg, &soc);
	if (ret)
		return ret;

	*val = soc >> 8;

	return 0;
}

/*
 * The gauge's learned full capacity is not read, so the full charge is the
 * design capacity from the monitored-battery node.
 */
static int sm5703_fg_get_charge_full(struct power_supply *psy, int *val)
{
	struct power_supply_battery_info *info = psy->battery_info;

	if (!info || info->charge_full_design_uah <= 0)
		return -ENODATA;

	*val = info->charge_full_design_uah;

	return 0;
}

static int sm5703_fg_get_charge_now(struct sm5703_fg *fg,
				    struct power_supply *psy, int *val)
{
	unsigned int soc;
	int full, ret;

	ret = sm5703_fg_get_charge_full(psy, &full);
	if (ret)
		return ret;

	ret = sm5703_fg_read_soc(fg, &soc);
	if (ret)
		return ret;

	*val = div_u64((u64)full * soc, 100 * 256);

	return 0;
}

/*
 * The charge state comes from the charger (power-supplies link): the net
 * current can be negative while charging, when the system draws more than
 * the input limit or the charger pauses at its end threshold.
 */
static int sm5703_fg_get_status(struct sm5703_fg *fg, struct power_supply *psy,
				int *val)
{
	union power_supply_propval prop;
	int supplied, current_ua, ret;

	supplied = power_supply_am_i_supplied(psy);
	if (supplied == 0) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	if (supplied > 0 &&
	    !power_supply_get_property_from_supplier(psy,
						     POWER_SUPPLY_PROP_STATUS,
						     &prop)) {
		*val = prop.intval;
		return 0;
	}

	/* No charger to ask: fall back to the current sign */
	ret = sm5703_fg_read_fixed(fg, SM5703_FG_REG_CURRENT, &current_ua);
	if (ret)
		return ret;

	if (current_ua > SM5703_FG_DEADBAND_UA)
		*val = POWER_SUPPLY_STATUS_CHARGING;
	else if (supplied > 0)
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		*val = POWER_SUPPLY_STATUS_DISCHARGING;

	return 0;
}

static void sm5703_fg_poll(struct work_struct *work)
{
	struct sm5703_fg *fg = container_of(work, struct sm5703_fg,
					    poll_work.work);
	int capacity, status;

	if (!sm5703_fg_get_capacity(fg, &capacity) &&
	    !sm5703_fg_get_status(fg, fg->psy, &status) &&
	    (capacity != fg->capacity || status != fg->status)) {
		fg->capacity = capacity;
		fg->status = status;
		power_supply_changed(fg->psy);
	}

	schedule_delayed_work(&fg->poll_work, msecs_to_jiffies(SM5703_FG_POLL_MS));
}

static int sm5703_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5703_fg *fg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return sm5703_fg_get_status(fg, psy, &val->intval);
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
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		return sm5703_fg_get_charge_full(psy, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		return sm5703_fg_get_charge_now(fg, psy, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return power_supply_get_property_from_supplier(psy, psp, val);
	default:
		return -EINVAL;
	}
}

static int sm5703_fg_property_is_writeable(struct power_supply *psy,
					   enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
}

static int sm5703_fg_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct power_supply *charger;
	int ret;

	if (!sm5703_fg_property_is_writeable(psy, psp))
		return -EINVAL;

	/* Not held between calls: a held reference blocks its unregister */
	charger = power_supply_get_by_reference(dev_fwnode(psy->dev.parent),
						"power-supplies");
	if (IS_ERR_OR_NULL(charger))
		return -ENODEV;

	ret = power_supply_set_property(charger, psp, val);
	power_supply_put(charger);

	return ret;
}

static const enum power_supply_property sm5703_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
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
	.set_property	= sm5703_fg_set_property,
	.property_is_writeable = sm5703_fg_property_is_writeable,
	.external_power_changed = power_supply_changed,
	.properties	= sm5703_fg_props,
	.num_properties	= ARRAY_SIZE(sm5703_fg_props),
};

/*
 * The core keeps the charger's name in supplied_from, which lives in the
 * charger module. A managed device link unbinds this driver before the
 * charger goes away, so that name is never read after it is freed.
 */
static int sm5703_fg_link_charger(struct device *dev)
{
	struct power_supply *charger;
	struct device_link *link;

	charger = power_supply_get_by_reference(dev_fwnode(dev), "power-supplies");
	if (IS_ERR(charger))
		return 0;
	if (!charger)
		return -EPROBE_DEFER;

	link = device_link_add(dev, charger->dev.parent,
			       DL_FLAG_AUTOREMOVE_CONSUMER);
	power_supply_put(charger);

	return link ? 0 : -EINVAL;
}

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

	ret = sm5703_fg_link_charger(dev);
	if (ret)
		return ret;

	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.drv_data = fg;

	fg->psy = devm_power_supply_register(dev, &sm5703_fg_desc, &psy_cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy),
				     "Failed to register power supply\n");

	fg->capacity = -1;
	fg->status = POWER_SUPPLY_STATUS_UNKNOWN;

	ret = devm_delayed_work_autocancel(dev, &fg->poll_work, sm5703_fg_poll);
	if (ret)
		return ret;

	schedule_delayed_work(&fg->poll_work, 0);

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
