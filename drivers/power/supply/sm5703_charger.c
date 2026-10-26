// SPDX-License-Identifier: GPL-2.0-only
/*
 * Charger driver for the Silicon Mitus SM5703
 *
 * The cable type comes from the device-type registers of the SM5703 MUIC,
 * which are only read: the MUIC is never written, so the USB switch stays
 * as the bootloader left it. The battery NTC is read through an IIO channel
 * and drives the vendor's temperature zones: charging stops when cold or
 * hot, and current and float voltage are reduced when cool or warm.
 * Everything is polled.
 */

#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define SM5703_REG_STATUS3		0x0a
#define SM5703_REG_CNTL			0x0c
#define SM5703_REG_VBUSCNTL		0x0d
#define SM5703_REG_CHGCNTL2		0x0f
#define SM5703_REG_CHGCNTL3		0x10
#define SM5703_REG_CHGCNTL4		0x11
#define SM5703_REG_CHGCNTL5		0x12
#define SM5703_REG_CHGCNTL6		0x13
#define SM5703_REG_DEVICE_ID		0x1e
#define SM5703_REG_STATUS5		0x6b
#define SM5703_REG_END			0x7f

#define SM5703_STATUS3_DONE		BIT(3)
#define SM5703_STATUS3_TOPOFF		BIT(2)
#define SM5703_STATUS3_CHGON		BIT(0)

#define SM5703_STATUS5_VBUSOVP		BIT(7)
#define SM5703_STATUS5_VBUSOK		BIT(5)

#define SM5703_CNTL_AUTOSET		BIT(4)
#define SM5703_CNTL_OPMODE		GENMASK(2, 0)
#define SM5703_OPMODE_CHARGING_ON	0x5

/* Input limit and fast-charge current: 100 mA + 50 mA steps */
#define SM5703_CURRENT_MASK		GENMASK(5, 0)
#define SM5703_CURRENT_UA(n)		(100000 + (n) * 50000)
#define SM5703_CURRENT_SEL(ua)		(((ua) - 100000) / 50000)

/* Float voltage: 4.12 V + 10 mV steps */
#define SM5703_BATREG_MASK		GENMASK(4, 0)
#define SM5703_BATREG_UV(n)		(4120000 + (n) * 10000)
#define SM5703_BATREG_SEL(uv)		(((uv) - 4120000) / 10000)

#define SM5703_CHGCNTL4_AUTOSTOP	BIT(7)
#define SM5703_CHGCNTL4_TOPOFF		GENMASK(6, 3)
#define SM5703_CHGCNTL5_AICLEN		BIT(7)
#define SM5703_CHGCNTL5_TOPOFF_TIMER	GENMASK(6, 5)
#define SM5703_CHGCNTL5_AICLTH		GENMASK(2, 0)
#define SM5703_CHGCNTL6_FREQSEL		GENMASK(3, 2)

/* Values the vendor kernel programs on this board */
#define SM5703_FLOAT_UV			4300000
#define SM5703_TOPOFF_SEL		0xb	/* 100 mA + 25 mA steps: 375 mA */
#define SM5703_TOPOFF_TIMER_45MIN	0x3
#define SM5703_AICLTH_4500MV		0x2
#define SM5703_FREQSEL_1P5MHZ		0x2

/* MUIC device-type registers */
#define SM5703_MUIC_REG_DEV_T1		0x0a
#define SM5703_MUIC_REG_DEV_T3		0x15
#define SM5703_MUIC_DEV_T1_USB		BIT(2)
#define SM5703_MUIC_DEV_T3_U200_CHG	BIT(6)	/* wall charger */

/* Temperature zones of the vendor kernel, in 0.1 C, with hysteresis */
#define SM5703_TEMP_COLD_ENTER		0	/* below */
#define SM5703_TEMP_COLD_LEAVE		50	/* at or above */
#define SM5703_TEMP_COOL_ENTER		150	/* below */
#define SM5703_TEMP_COOL_LEAVE		200	/* at or above */
#define SM5703_TEMP_WARM_ENTER		410	/* above */
#define SM5703_TEMP_WARM_LEAVE		390	/* at or below */
#define SM5703_TEMP_HOT_ENTER		500	/* above */
#define SM5703_TEMP_HOT_LEAVE		450	/* at or below */

/* Swelling: reduced float voltage, and fast-charge current caps */
#define SM5703_FLOAT_SWELL_UV		4120000
#define SM5703_COOL_CHARGE_UA		850000
#define SM5703_WARM_CHARGE_UA		1300000

#define SM5703_POLL_MS			5000

#define SM5703_FG_NAME			"sm5703-fuel-gauge"
#define SM5703_CHARGE_END_DEFAULT	100
#define SM5703_CHARGE_START_DEFAULT	0

static int charge_end = SM5703_CHARGE_END_DEFAULT;
module_param(charge_end, int, 0444);
MODULE_PARM_DESC(charge_end, "Stop charging at this battery percentage, 100 disables the limit (default 100)");

static int charge_start = SM5703_CHARGE_START_DEFAULT;
module_param(charge_start, int, 0444);
MODULE_PARM_DESC(charge_start, "Resume charging at this battery percentage (default 0)");

/* Test aid: shifts the temperature the zones see, not the reported one */
static int temp_offset;
module_param(temp_offset, int, 0644);
MODULE_PARM_DESC(temp_offset, "Debug: add this many 0.1 C to the battery temperature for the zones (default 0)");

struct sm5703_chg_limits {
	int input_ua;
	int charge_ua;
};

static const struct sm5703_chg_limits sm5703_limits_sdp = { 450000, 450000 };
static const struct sm5703_chg_limits sm5703_limits_dcp = { 1650000, 2400000 };

enum sm5703_zone {
	SM5703_ZONE_NORMAL,
	SM5703_ZONE_COOL,
	SM5703_ZONE_WARM,
	SM5703_ZONE_COLD,
	SM5703_ZONE_HOT,
};

static const char * const sm5703_zone_names[] = {
	[SM5703_ZONE_NORMAL]	= "normal",
	[SM5703_ZONE_COOL]	= "cool",
	[SM5703_ZONE_WARM]	= "warm",
	[SM5703_ZONE_COLD]	= "cold",
	[SM5703_ZONE_HOT]	= "hot",
};

struct sm5703_charger {
	struct device		*dev;
	struct regmap		*regmap;
	struct i2c_client	*muic;
	struct iio_channel	*temp_chan;
	struct gpio_desc	*enable_gpio;
	struct power_supply	*psy;
	struct delayed_work	poll_work;
	struct mutex		lock;

	/* Written by the poll work under lock (applied values also by hw_init) */
	bool			online;
	enum power_supply_usb_type usb_type;
	enum sm5703_zone	zone;
	int			temp;	/* 0.1 C */
	bool			temp_valid;
	bool			limited;
	int			input_ua;
	int			charge_ua;
	int			float_uv;
	int			end_threshold;
	int			start_threshold;
};

/* Each zone is left only past its hysteresis, but zones may be skipped */
static enum sm5703_zone sm5703_chg_zone(enum sm5703_zone prev, int temp)
{
	if (temp > SM5703_TEMP_HOT_ENTER)
		return SM5703_ZONE_HOT;
	if (temp < SM5703_TEMP_COLD_ENTER)
		return SM5703_ZONE_COLD;
	if (prev == SM5703_ZONE_HOT && temp > SM5703_TEMP_HOT_LEAVE)
		return SM5703_ZONE_HOT;
	if (prev == SM5703_ZONE_COLD && temp < SM5703_TEMP_COLD_LEAVE)
		return SM5703_ZONE_COLD;

	if (temp > SM5703_TEMP_WARM_ENTER)
		return SM5703_ZONE_WARM;
	if (temp < SM5703_TEMP_COOL_ENTER)
		return SM5703_ZONE_COOL;
	if (prev == SM5703_ZONE_WARM && temp > SM5703_TEMP_WARM_LEAVE)
		return SM5703_ZONE_WARM;
	if (prev == SM5703_ZONE_COOL && temp < SM5703_TEMP_COOL_LEAVE)
		return SM5703_ZONE_COOL;

	return SM5703_ZONE_NORMAL;
}

static enum power_supply_usb_type sm5703_chg_detect(struct sm5703_charger *chg)
{
	int t1, t3;

	t1 = i2c_smbus_read_byte_data(chg->muic, SM5703_MUIC_REG_DEV_T1);
	t3 = i2c_smbus_read_byte_data(chg->muic, SM5703_MUIC_REG_DEV_T3);
	if (t1 < 0 || t3 < 0) {
		dev_warn_ratelimited(chg->dev, "MUIC read failed (%d, %d)\n",
				     t1, t3);
		return POWER_SUPPLY_USB_TYPE_UNKNOWN;
	}

	if (t3 & SM5703_MUIC_DEV_T3_U200_CHG)
		return POWER_SUPPLY_USB_TYPE_DCP;
	if (t1 & SM5703_MUIC_DEV_T1_USB)
		return POWER_SUPPLY_USB_TYPE_SDP;

	return POWER_SUPPLY_USB_TYPE_UNKNOWN;
}

static int sm5703_chg_set_limits(struct sm5703_charger *chg, int input_ua,
				 int charge_ua)
{
	int ret;

	ret = regmap_update_bits(chg->regmap, SM5703_REG_VBUSCNTL,
				 SM5703_CURRENT_MASK, SM5703_CURRENT_SEL(input_ua));
	if (ret)
		return ret;
	chg->input_ua = input_ua;

	ret = regmap_update_bits(chg->regmap, SM5703_REG_CHGCNTL2,
				 SM5703_CURRENT_MASK, SM5703_CURRENT_SEL(charge_ua));
	if (ret)
		return ret;
	chg->charge_ua = charge_ua;

	return 0;
}

static int sm5703_chg_set_float(struct sm5703_charger *chg, int float_uv)
{
	int ret;

	ret = regmap_update_bits(chg->regmap, SM5703_REG_CHGCNTL3,
				 SM5703_BATREG_MASK, SM5703_BATREG_SEL(float_uv));
	if (!ret)
		chg->float_uv = float_uv;

	return ret;
}

/* Writes the registers only when the cable or the zone changed a value */
static int sm5703_chg_apply(struct sm5703_charger *chg,
			    enum power_supply_usb_type type, enum sm5703_zone zone)
{
	const struct sm5703_chg_limits *lim = type == POWER_SUPPLY_USB_TYPE_DCP ?
					      &sm5703_limits_dcp : &sm5703_limits_sdp;
	int charge_ua = lim->charge_ua, float_uv = SM5703_FLOAT_UV;
	int ret;

	if (zone == SM5703_ZONE_COOL) {
		charge_ua = min(charge_ua, SM5703_COOL_CHARGE_UA);
		float_uv = SM5703_FLOAT_SWELL_UV;
	} else if (zone == SM5703_ZONE_WARM) {
		charge_ua = min(charge_ua, SM5703_WARM_CHARGE_UA);
		float_uv = SM5703_FLOAT_SWELL_UV;
	}

	if (lim->input_ua != chg->input_ua || charge_ua != chg->charge_ua) {
		ret = sm5703_chg_set_limits(chg, lim->input_ua, charge_ua);
		if (ret)
			return ret;
	}

	if (float_uv != chg->float_uv)
		return sm5703_chg_set_float(chg, float_uv);

	return 0;
}

static void sm5703_chg_poll(struct work_struct *work)
{
	struct sm5703_charger *chg = container_of(work, struct sm5703_charger,
						  poll_work.work);
	enum power_supply_usb_type type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	bool changed, online = false, limited, temp_ok;
	enum sm5703_zone zone;
	union power_supply_propval cap;
	struct power_supply *fg_psy;
	unsigned int st5;
	int temp = chg->temp, capacity = -1, offset = READ_ONCE(temp_offset);
	int mdeg, ret;

	ret = regmap_read(chg->regmap, SM5703_REG_STATUS5, &st5);
	if (!ret)
		online = st5 & SM5703_STATUS5_VBUSOK;

	if (online)
		type = sm5703_chg_detect(chg);

	/* Fail safe: without a temperature, do not charge */
	ret = iio_read_channel_processed(chg->temp_chan, &mdeg);
	temp_ok = !ret;
	if (!temp_ok) {
		dev_warn_ratelimited(chg->dev, "battery temperature read failed: %d\n",
				     ret);
		zone = SM5703_ZONE_HOT;
	} else {
		temp = DIV_ROUND_CLOSEST(mdeg, 100);
		zone = sm5703_chg_zone(chg->zone, temp + offset);
	}

	/*
	 * Do not depend on the fuel gauge, and do not hold it between polls:
	 * a held reference trips power_supply_unregister() when it goes away.
	 */
	fg_psy = power_supply_get_by_name(SM5703_FG_NAME);
	if (fg_psy) {
		if (!power_supply_get_property(fg_psy, POWER_SUPPLY_PROP_CAPACITY, &cap))
			capacity = cap.intval;
		power_supply_put(fg_psy);
	}

	mutex_lock(&chg->lock);

	limited = chg->limited;
	if (chg->end_threshold == 100)
		limited = false;
	else if (capacity >= chg->end_threshold)
		limited = true;
	else if (capacity >= 0 && capacity <= chg->start_threshold)
		limited = false;

	if (limited != chg->limited)
		dev_info(chg->dev, "charging %s at %d%%\n",
			 limited ? "paused" : "resumed", capacity);

	if (zone != chg->zone) {
		if (!temp_ok)
			dev_info(chg->dev, "battery temperature unknown: %s\n",
				 sm5703_zone_names[zone]);
		else
			dev_info(chg->dev, "battery temperature %s%d.%d C%s: %s\n",
				 temp < 0 ? "-" : "", abs(temp) / 10, abs(temp) % 10,
				 offset ? " (debug offset)" : "", sm5703_zone_names[zone]);
	}

	ret = sm5703_chg_apply(chg, type, zone);
	if (ret)
		dev_err_ratelimited(chg->dev, "failed to set charge limits: %d\n", ret);
	else if (type != chg->usb_type)
		dev_info(chg->dev, "cable type %d\n", type);

	gpiod_set_value_cansleep(chg->enable_gpio,
				 zone != SM5703_ZONE_COLD && zone != SM5703_ZONE_HOT &&
				 !limited);

	changed = online != chg->online || type != chg->usb_type ||
		  zone != chg->zone || limited != chg->limited;
	chg->online = online;
	chg->usb_type = type;
	chg->zone = zone;
	chg->temp = temp;
	chg->temp_valid = temp_ok;
	chg->limited = limited;

	mutex_unlock(&chg->lock);

	if (changed)
		power_supply_changed(chg->psy);

	schedule_delayed_work(&chg->poll_work, msecs_to_jiffies(SM5703_POLL_MS));
}

static int sm5703_chg_get_status(struct sm5703_charger *chg, int *val)
{
	unsigned int st3;
	int ret;

	ret = regmap_read(chg->regmap, SM5703_REG_STATUS3, &st3);
	if (ret)
		return ret;

	if (!chg->online)
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (chg->limited)
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if (st3 & (SM5703_STATUS3_DONE | SM5703_STATUS3_TOPOFF))
		*val = POWER_SUPPLY_STATUS_FULL;
	else if (st3 & SM5703_STATUS3_CHGON)
		*val = POWER_SUPPLY_STATUS_CHARGING;
	else
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

static int sm5703_chg_get_health(struct sm5703_charger *chg, int *val)
{
	unsigned int st5;
	int ret;

	switch (chg->zone) {
	case SM5703_ZONE_COLD:
		*val = POWER_SUPPLY_HEALTH_COLD;
		return 0;
	case SM5703_ZONE_HOT:
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
		return 0;
	case SM5703_ZONE_COOL:
		*val = POWER_SUPPLY_HEALTH_COOL;
		return 0;
	case SM5703_ZONE_WARM:
		*val = POWER_SUPPLY_HEALTH_WARM;
		return 0;
	case SM5703_ZONE_NORMAL:
		break;
	}

	ret = regmap_read(chg->regmap, SM5703_REG_STATUS5, &st5);
	if (ret)
		return ret;

	if (st5 & SM5703_STATUS5_VBUSOVP)
		*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else
		*val = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int sm5703_chg_read_field(struct sm5703_charger *chg, unsigned int reg,
				 unsigned int mask, unsigned int *sel)
{
	int ret;

	ret = regmap_read(chg->regmap, reg, sel);
	if (ret)
		return ret;

	*sel &= mask;

	return 0;
}

static int sm5703_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sm5703_charger *chg = power_supply_get_drvdata(psy);
	unsigned int sel;
	int ret = 0;

	mutex_lock(&chg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chg->online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = sm5703_chg_get_status(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = sm5703_chg_get_health(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = chg->usb_type;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (chg->temp_valid)
			val->intval = chg->temp;
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sm5703_chg_read_field(chg, SM5703_REG_VBUSCNTL,
					    SM5703_CURRENT_MASK, &sel);
		val->intval = SM5703_CURRENT_UA(sel);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sm5703_chg_read_field(chg, SM5703_REG_CHGCNTL2,
					    SM5703_CURRENT_MASK, &sel);
		val->intval = SM5703_CURRENT_UA(sel);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = sm5703_chg_read_field(chg, SM5703_REG_CHGCNTL3,
					    SM5703_BATREG_MASK, &sel);
		val->intval = SM5703_BATREG_UV(sel);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = chg->end_threshold;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = chg->start_threshold;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&chg->lock);

	return ret;
}

static int sm5703_chg_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct sm5703_charger *chg = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&chg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		if (val->intval < 1 || val->intval > 100 ||
		    val->intval <= chg->start_threshold)
			ret = -EINVAL;
		else
			chg->end_threshold = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		if (val->intval < 0 || val->intval >= chg->end_threshold)
			ret = -EINVAL;
		else
			chg->start_threshold = val->intval;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&chg->lock);

	/* A no-op once the stop action has disabled the work */
	if (!ret)
		mod_delayed_work(system_percpu_wq, &chg->poll_work, 0);

	return ret;
}

static int sm5703_chg_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD;
}

static const enum power_supply_property sm5703_chg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
};

static const struct power_supply_desc sm5703_chg_desc = {
	.name		= "sm5703-charger",
	.type		= POWER_SUPPLY_TYPE_USB,
	.usb_types	= BIT(POWER_SUPPLY_USB_TYPE_SDP) |
			  BIT(POWER_SUPPLY_USB_TYPE_DCP) |
			  BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
	.get_property	= sm5703_chg_get_property,
	.set_property	= sm5703_chg_set_property,
	.property_is_writeable = sm5703_chg_property_is_writeable,
	.properties	= sm5703_chg_props,
	.num_properties	= ARRAY_SIZE(sm5703_chg_props),
};

static int sm5703_chg_hw_init(struct sm5703_charger *chg)
{
	int ret;

	/*
	 * Float voltage first: the charger already runs at its POR
	 * 4.34 V, which is above what the vendor kernel allows.
	 */
	ret = sm5703_chg_set_float(chg, SM5703_FLOAT_UV);
	if (ret)
		return ret;

	ret = regmap_update_bits(chg->regmap, SM5703_REG_CHGCNTL4,
				 SM5703_CHGCNTL4_AUTOSTOP | SM5703_CHGCNTL4_TOPOFF,
				 SM5703_CHGCNTL4_AUTOSTOP |
				 FIELD_PREP(SM5703_CHGCNTL4_TOPOFF, SM5703_TOPOFF_SEL));
	if (ret)
		return ret;

	ret = regmap_update_bits(chg->regmap, SM5703_REG_CHGCNTL5,
				 SM5703_CHGCNTL5_AICLEN |
				 SM5703_CHGCNTL5_TOPOFF_TIMER |
				 SM5703_CHGCNTL5_AICLTH,
				 FIELD_PREP(SM5703_CHGCNTL5_TOPOFF_TIMER,
					    SM5703_TOPOFF_TIMER_45MIN) |
				 FIELD_PREP(SM5703_CHGCNTL5_AICLTH,
					    SM5703_AICLTH_4500MV));
	if (ret)
		return ret;

	ret = regmap_update_bits(chg->regmap, SM5703_REG_CHGCNTL6,
				 SM5703_CHGCNTL6_FREQSEL,
				 FIELD_PREP(SM5703_CHGCNTL6_FREQSEL,
					    SM5703_FREQSEL_1P5MHZ));
	if (ret)
		return ret;

	ret = regmap_update_bits(chg->regmap, SM5703_REG_CNTL,
				 SM5703_CNTL_AUTOSET | SM5703_CNTL_OPMODE,
				 SM5703_CNTL_AUTOSET | SM5703_OPMODE_CHARGING_ON);
	if (ret)
		return ret;

	/* PC-port limits until the first poll has seen the cable */
	return sm5703_chg_set_limits(chg, sm5703_limits_sdp.input_ua,
				     sm5703_limits_sdp.charge_ua);
}

static int sm5703_chg_get_client(struct device *dev, const char *prop,
				 struct i2c_client **client)
{
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, prop, 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "missing %s\n", prop);

	*client = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!*client)
		return dev_err_probe(dev, -EPROBE_DEFER, "%s not found\n", prop);

	return 0;
}

static void sm5703_chg_put_client(void *client)
{
	put_device(&((struct i2c_client *)client)->dev);
}

/* Runs before the psy unregister: a threshold write must not requeue the poll */
static void sm5703_chg_stop(void *data)
{
	struct sm5703_charger *chg = data;

	disable_delayed_work_sync(&chg->poll_work);
}

static const struct regmap_config sm5703_chg_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= SM5703_REG_END,
};

static int sm5703_chg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct sm5703_charger *chg;
	unsigned int id;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	chg->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	mutex_init(&chg->lock);
	INIT_DELAYED_WORK(&chg->poll_work, sm5703_chg_poll);

	if (charge_end < 1 || charge_end > 100 ||
	    charge_start < 0 || charge_start >= charge_end) {
		dev_warn(dev, "invalid charge_end/charge_start %d/%d, using %d/%d\n",
			 charge_end, charge_start, SM5703_CHARGE_END_DEFAULT,
			 SM5703_CHARGE_START_DEFAULT);
		chg->end_threshold = SM5703_CHARGE_END_DEFAULT;
		chg->start_threshold = SM5703_CHARGE_START_DEFAULT;
	} else {
		chg->end_threshold = charge_end;
		chg->start_threshold = charge_start;
	}

	chg->regmap = devm_regmap_init_i2c(client, &sm5703_chg_regmap_config);
	if (IS_ERR(chg->regmap))
		return dev_err_probe(dev, PTR_ERR(chg->regmap),
				     "Failed to initialize regmap\n");

	ret = sm5703_chg_get_client(dev, "siliconmitus,muic", &chg->muic);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, sm5703_chg_put_client, chg->muic);
	if (ret)
		return ret;

	chg->temp_chan = devm_iio_channel_get(dev, "batt-temp");
	if (IS_ERR(chg->temp_chan))
		return dev_err_probe(dev, PTR_ERR(chg->temp_chan),
				     "Failed to get battery temperature channel\n");

	/* nCHG_EN, active low: logical 1 enables charging */
	chg->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(chg->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(chg->enable_gpio),
				     "Failed to get enable GPIO\n");

	ret = regmap_read(chg->regmap, SM5703_REG_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read device ID\n");

	ret = sm5703_chg_hw_init(chg);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize charger\n");

	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.drv_data = chg;

	chg->psy = devm_power_supply_register(dev, &sm5703_chg_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "Failed to register power supply\n");

	ret = devm_add_action_or_reset(dev, sm5703_chg_stop, chg);
	if (ret)
		return ret;

	dev_info(dev, "SM5703 charger, device ID 0x%02x\n", id);

	schedule_delayed_work(&chg->poll_work, 0);

	return 0;
}

static const struct i2c_device_id sm5703_chg_id[] = {
	{ .name = "sm5703-charger" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5703_chg_id);

static const struct of_device_id sm5703_chg_of_match[] = {
	{ .compatible = "siliconmitus,sm5703-charger", },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5703_chg_of_match);

static struct i2c_driver sm5703_chg_driver = {
	.driver = {
		.name = "sm5703-charger",
		.of_match_table = sm5703_chg_of_match,
	},
	.probe = sm5703_chg_probe,
	.id_table = sm5703_chg_id,
};
module_i2c_driver(sm5703_chg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5703 charger driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
