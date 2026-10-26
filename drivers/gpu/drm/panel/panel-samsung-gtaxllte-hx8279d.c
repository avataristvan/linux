// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Galaxy Tab A 10.1 (2016) LTE (SM-T585, gtaxllte) panel driver:
 * 1200x1920 MIPI-DSI video mode panel on a Himax HX8279-D driver IC.
 *
 * The panel ID read by the bootloader is 0x827903, which the downstream
 * kernel treats as the "BOE2" variant (BOE 8.5" panel for the T58x). The
 * vendor and exact panel model are not otherwise known, hence the board
 * based compatible. Init sequence and brightness command (DSI register 0xB8
 * on page 4, no separate backlight IC) are taken from the downstream
 * hx8279d_lcd_ctrl.c / hx8279d_param.h.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#define GTAXL_REG_PAGE			0xb0
#define GTAXL_REG_SLEEP			0xb2
#define GTAXL_REG_DISPLAY		0xb3
#define GTAXL_REG_BRIGHTNESS		0xb8

#define GTAXL_DISPLAY_ON		0x08
#define GTAXL_DISPLAY_OFF_BL_OFF	0x8c
#define GTAXL_DISPLAY_OFF		0x88

#define GTAXL_PAGE_CABC			0x04

/* Largest value of the hardware brightness register that the UI range maps to */
#define GTAXL_HW_BRIGHTNESS_MAX		199
#define GTAXL_UI_BRIGHTNESS_MAX		255
#define GTAXL_UI_BRIGHTNESS_DEFAULT	117

struct gtaxl_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *iovcc;
	struct regulator *vdd;
	struct gpio_desc *reset_gpio;
	struct backlight_device *backlight;

	/* Serializes brightness writes against enable/disable */
	struct mutex lock;
	bool enabled;
};

static inline struct gtaxl_panel *to_gtaxl_panel(struct drm_panel *panel)
{
	return container_of(panel, struct gtaxl_panel, panel);
}

struct gtaxl_cmd {
	u8 reg;
	u8 val;
};

/* SEQ_CMD_TABLE_BOE2 of the downstream driver, in order */
static const struct gtaxl_cmd gtaxl_boe2_init[] = {
	{ 0xb0, 0x00 }, { 0xb6, 0x03 }, { 0xba, 0x87 }, { 0xbf, 0x15 },
	{ 0xc0, 0x0f }, { 0xc2, 0x0c }, { 0xc3, 0x02 }, { 0xc4, 0x0c },
	{ 0xc5, 0x02 }, { 0xb0, 0x01 }, { 0xe0, 0x26 }, { 0xe1, 0x26 },
	{ 0xdc, 0x00 }, { 0xdd, 0x00 }, { 0xcc, 0x26 }, { 0xcd, 0x26 },
	{ 0xc8, 0x00 }, { 0xc9, 0x00 }, { 0xd2, 0x03 }, { 0xd3, 0x03 },
	{ 0xe6, 0x04 }, { 0xe7, 0x04 }, { 0xc4, 0x09 }, { 0xc5, 0x09 },
	{ 0xd8, 0x0a }, { 0xd9, 0x0a }, { 0xc2, 0x0b }, { 0xc3, 0x0b },
	{ 0xd6, 0x0c }, { 0xd7, 0x0c }, { 0xc0, 0x05 }, { 0xc1, 0x05 },
	{ 0xd4, 0x06 }, { 0xd5, 0x06 }, { 0xca, 0x07 }, { 0xcb, 0x07 },
	{ 0xde, 0x08 }, { 0xdf, 0x08 }, { 0xb0, 0x03 }, { 0xc8, 0x0b },
	{ 0xc9, 0x07 }, { 0xc3, 0x00 }, { 0xe7, 0x00 }, { 0xc5, 0x2a },
	{ 0xde, 0x2a }, { 0xca, 0x43 }, { 0xc9, 0x07 }, { 0xe4, 0xc0 },
	{ 0xe5, 0x0d }, { 0xcb, 0x28 }, { 0xb0, 0x05 }, { 0xb3, 0x52 },
	{ 0xb0, 0x06 }, { 0xb8, 0xa5 }, { 0xc0, 0xa5 }, { 0xc7, 0x0f },
	{ 0xd5, 0x32 }, { 0xb0, 0x02 }, { 0xc0, 0x00 }, { 0xc1, 0x07 },
	{ 0xc2, 0x10 }, { 0xc3, 0x1f }, { 0xc4, 0x32 }, { 0xc5, 0x35 },
	{ 0xc6, 0x38 }, { 0xc7, 0x3a }, { 0xc8, 0x3e }, { 0xc9, 0x3f },
	{ 0xca, 0x3f }, { 0xcb, 0x3f }, { 0xcc, 0x3f }, { 0xcd, 0x37 },
	{ 0xce, 0x36 }, { 0xcf, 0x34 }, { 0xd0, 0x07 }, { 0xd2, 0x00 },
	{ 0xd3, 0x07 }, { 0xd4, 0x10 }, { 0xd5, 0x1f }, { 0xd6, 0x32 },
	{ 0xd7, 0x35 }, { 0xd8, 0x38 }, { 0xd9, 0x3a }, { 0xda, 0x3e },
	{ 0xdb, 0x3f }, { 0xdc, 0x3f }, { 0xdd, 0x3f }, { 0xde, 0x3f },
	{ 0xdf, 0x37 }, { 0xe0, 0x36 }, { 0xe1, 0x34 }, { 0xe2, 0x07 },
	{ 0xb0, 0x06 }, { 0xb8, 0xa5 }, { 0xbc, 0x33 }, { 0xb0, 0x07 },
	{ 0xb1, 0x08 }, { 0xb2, 0x08 }, { 0xb3, 0x11 }, { 0xb4, 0x24 },
	{ 0xb5, 0x37 }, { 0xb6, 0x48 }, { 0xb7, 0x60 }, { 0xb8, 0x79 },
	{ 0xb9, 0xb9 }, { 0xba, 0xf8 }, { 0xbb, 0x71 }, { 0xbc, 0xe9 },
	{ 0xbd, 0xed }, { 0xbe, 0x67 }, { 0xbf, 0xe4 }, { 0xcb, 0x05 },
	{ 0xcc, 0x6b }, { 0xc0, 0x25 }, { 0xc1, 0x63 }, { 0xc2, 0x84 },
	{ 0xc3, 0x9c }, { 0xc4, 0xa6 }, { 0xc5, 0xb1 }, { 0xc6, 0xbd },
	{ 0xc7, 0xc4 }, { 0xc8, 0xc8 }, { 0xb0, 0x08 }, { 0xb1, 0x04 },
	{ 0xb2, 0x06 }, { 0xb3, 0x11 }, { 0xb4, 0x24 }, { 0xb5, 0x38 },
	{ 0xb6, 0x49 }, { 0xb7, 0x61 }, { 0xb8, 0x79 }, { 0xb9, 0xb9 },
	{ 0xba, 0xf8 }, { 0xbb, 0x71 }, { 0xbc, 0xea }, { 0xbd, 0xed },
	{ 0xbe, 0x68 }, { 0xbf, 0xe5 }, { 0xcb, 0x05 }, { 0xcc, 0x6b },
	{ 0xc0, 0x26 }, { 0xc1, 0x64 }, { 0xc2, 0x85 }, { 0xc3, 0x9c },
	{ 0xc4, 0xa7 }, { 0xc5, 0xb1 }, { 0xc6, 0xbd }, { 0xc7, 0xc4 },
	{ 0xc8, 0xc8 }, { 0xb0, 0x09 }, { 0xb1, 0x0c }, { 0xb2, 0x0c },
	{ 0xb3, 0x0f }, { 0xb4, 0x23 }, { 0xb5, 0x37 }, { 0xb6, 0x48 },
	{ 0xb7, 0x60 }, { 0xb8, 0x77 }, { 0xb9, 0xb8 }, { 0xba, 0xf7 },
	{ 0xbb, 0x6f }, { 0xbc, 0xe6 }, { 0xbd, 0xea }, { 0xbe, 0x62 },
	{ 0xbf, 0xda }, { 0xcb, 0x05 }, { 0xcc, 0x6b }, { 0xc0, 0x1a },
	{ 0xc1, 0x56 }, { 0xc2, 0x75 }, { 0xc3, 0x91 }, { 0xc4, 0x9c },
	{ 0xc5, 0xa5 }, { 0xc6, 0xaf }, { 0xc7, 0xb5 }, { 0xc8, 0xb8 },
	{ 0xb0, 0x0a }, { 0xb1, 0x08 }, { 0xb2, 0x08 }, { 0xb3, 0x11 },
	{ 0xb4, 0x24 }, { 0xb5, 0x37 }, { 0xb6, 0x48 }, { 0xb7, 0x60 },
	{ 0xb8, 0x79 }, { 0xb9, 0xb9 }, { 0xba, 0xf8 }, { 0xbb, 0x71 },
	{ 0xbc, 0xe9 }, { 0xbd, 0xed }, { 0xbe, 0x67 }, { 0xbf, 0xe4 },
	{ 0xcb, 0x05 }, { 0xcc, 0x6b }, { 0xc0, 0x25 }, { 0xc1, 0x63 },
	{ 0xc2, 0x84 }, { 0xc3, 0x9c }, { 0xc4, 0xa6 }, { 0xc5, 0xb1 },
	{ 0xc6, 0xbd }, { 0xc7, 0xc4 }, { 0xc8, 0xc8 }, { 0xb0, 0x0b },
	{ 0xb1, 0x04 }, { 0xb2, 0x06 }, { 0xb3, 0x11 }, { 0xb4, 0x24 },
	{ 0xb5, 0x38 }, { 0xb6, 0x49 }, { 0xb7, 0x61 }, { 0xb8, 0x79 },
	{ 0xb9, 0xb9 }, { 0xba, 0xf8 }, { 0xbb, 0x71 }, { 0xbc, 0xea },
	{ 0xbd, 0xed }, { 0xbe, 0x68 }, { 0xbf, 0xe5 }, { 0xcb, 0x05 },
	{ 0xcc, 0x6b }, { 0xc0, 0x26 }, { 0xc1, 0x64 }, { 0xc2, 0x85 },
	{ 0xc3, 0x9c }, { 0xc4, 0xa7 }, { 0xc5, 0xb1 }, { 0xc6, 0xbd },
	{ 0xc7, 0xc4 }, { 0xc8, 0xc8 }, { 0xb0, 0x0c }, { 0xb1, 0x0c },
	{ 0xb2, 0x0c }, { 0xb3, 0x0f }, { 0xb4, 0x23 }, { 0xb5, 0x37 },
	{ 0xb6, 0x48 }, { 0xb7, 0x60 }, { 0xb8, 0x77 }, { 0xb9, 0xb8 },
	{ 0xba, 0xf7 }, { 0xbb, 0x6f }, { 0xbc, 0xe6 }, { 0xbd, 0xea },
	{ 0xbe, 0x62 }, { 0xbf, 0xda }, { 0xcb, 0x05 }, { 0xcc, 0x6b },
	{ 0xc0, 0x1a }, { 0xc1, 0x56 }, { 0xc2, 0x75 }, { 0xc3, 0x91 },
	{ 0xc4, 0x9c }, { 0xc5, 0xa5 }, { 0xc6, 0xaf }, { 0xc7, 0xb5 },
	{ 0xc8, 0xb8 },
};

static void gtaxl_write(struct mipi_dsi_multi_context *ctx, u8 reg, u8 val)
{
	const u8 cmd[] = { reg, val };

	mipi_dsi_dcs_write_buffer_multi(ctx, cmd, sizeof(cmd));
}

static void gtaxl_set_page(struct mipi_dsi_multi_context *ctx, u8 page)
{
	gtaxl_write(ctx, GTAXL_REG_PAGE, page);
}

static void gtaxl_write_brightness(struct gtaxl_panel *p,
				   struct mipi_dsi_multi_context *ctx)
{
	int ui = backlight_get_brightness(p->backlight);
	u8 hw = ui * GTAXL_HW_BRIGHTNESS_MAX / GTAXL_UI_BRIGHTNESS_MAX;

	if (ui > 0 && !hw)
		hw = 1;

	gtaxl_set_page(ctx, GTAXL_PAGE_CABC);
	gtaxl_write(ctx, GTAXL_REG_BRIGHTNESS, hw);
}

static int gtaxl_bl_update_status(struct backlight_device *bl)
{
	struct gtaxl_panel *p = bl_get_data(bl);
	struct mipi_dsi_multi_context ctx = { .dsi = p->dsi };

	mutex_lock(&p->lock);
	if (p->enabled)
		gtaxl_write_brightness(p, &ctx);
	mutex_unlock(&p->lock);

	return ctx.accum_err;
}

static const struct backlight_ops gtaxl_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = gtaxl_bl_update_status,
};

static void gtaxl_reset(struct gtaxl_panel *p)
{
	/* Logical 1 means reset asserted (GPIO_ACTIVE_LOW in the DT) */
	gpiod_set_value_cansleep(p->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(p->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(p->reset_gpio, 0);
	usleep_range(10000, 11000);
}

/* Downstream decon_board order: iovcc, 10 ms, vdd, 15 ms */
static int gtaxl_power_on(struct gtaxl_panel *p)
{
	int ret;

	ret = regulator_enable(p->iovcc);
	if (ret)
		return ret;
	msleep(10);

	ret = regulator_enable(p->vdd);
	if (ret) {
		regulator_disable(p->iovcc);
		return ret;
	}
	msleep(15);

	return 0;
}

/* Downstream decon_board order: reset asserted, vdd off, 10 ms, iovcc off */
static void gtaxl_power_off(struct gtaxl_panel *p)
{
	gpiod_set_value_cansleep(p->reset_gpio, 1);
	regulator_disable(p->vdd);
	msleep(10);
	regulator_disable(p->iovcc);
}

static int gtaxl_prepare(struct drm_panel *panel)
{
	struct gtaxl_panel *p = to_gtaxl_panel(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = p->dsi };
	int ret, i;

	ret = gtaxl_power_on(p);
	if (ret)
		return ret;

	gtaxl_reset(p);

	gtaxl_set_page(&ctx, 0x05);
	gtaxl_write(&ctx, 0xb1, 0xe5);	/* EOTP disable */
	gtaxl_write(&ctx, GTAXL_REG_DISPLAY, 0x32);	/* TLPX 80 ns */

	gtaxl_set_page(&ctx, 0x00);
	gtaxl_write(&ctx, GTAXL_REG_DISPLAY, GTAXL_DISPLAY_OFF);

	gtaxl_set_page(&ctx, GTAXL_PAGE_CABC);
	gtaxl_write(&ctx, GTAXL_REG_BRIGHTNESS, 0x00);

	gtaxl_set_page(&ctx, 0x00);
	for (i = 0; i < ARRAY_SIZE(gtaxl_boe2_init); i++)
		gtaxl_write(&ctx, gtaxl_boe2_init[i].reg, gtaxl_boe2_init[i].val);

	/* CABC page: PWM frequency 8 kHz */
	gtaxl_set_page(&ctx, GTAXL_PAGE_CABC);
	gtaxl_write(&ctx, 0xb5, 0x02);
	gtaxl_write(&ctx, 0xb6, 0x02);
	mipi_dsi_msleep(&ctx, 110);

	if (ctx.accum_err)
		gtaxl_power_off(p);

	return ctx.accum_err;
}

static int gtaxl_enable(struct drm_panel *panel)
{
	struct gtaxl_panel *p = to_gtaxl_panel(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = p->dsi };

	gtaxl_set_page(&ctx, 0x00);
	gtaxl_write(&ctx, GTAXL_REG_DISPLAY, GTAXL_DISPLAY_ON);
	mipi_dsi_msleep(&ctx, 40);

	mutex_lock(&p->lock);
	gtaxl_write_brightness(p, &ctx);
	p->enabled = !ctx.accum_err;
	mutex_unlock(&p->lock);

	return ctx.accum_err;
}

static int gtaxl_disable(struct drm_panel *panel)
{
	struct gtaxl_panel *p = to_gtaxl_panel(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = p->dsi };

	mutex_lock(&p->lock);
	p->enabled = false;
	mutex_unlock(&p->lock);

	gtaxl_set_page(&ctx, 0x00);
	gtaxl_write(&ctx, GTAXL_REG_DISPLAY, GTAXL_DISPLAY_OFF_BL_OFF);
	mipi_dsi_msleep(&ctx, 50);

	return ctx.accum_err;
}

static int gtaxl_unprepare(struct drm_panel *panel)
{
	struct gtaxl_panel *p = to_gtaxl_panel(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = p->dsi };

	gtaxl_set_page(&ctx, 0x00);
	gtaxl_write(&ctx, GTAXL_REG_SLEEP, 0x00);
	mipi_dsi_msleep(&ctx, 120);

	gtaxl_power_off(p);

	return 0;
}

/*
 * Downstream porches: h = bp 32 / fp 120 / sa 8, v = bp 16 / fp 16 / sa 4,
 * pixel clock 159 MHz from a 60 Hz refresh.
 */
static const struct drm_display_mode gtaxl_mode = {
	.clock = 159600,
	.hdisplay = 1200,
	.hsync_start = 1200 + 120,
	.hsync_end = 1200 + 120 + 8,
	.htotal = 1200 + 120 + 8 + 32,
	.vdisplay = 1920,
	.vsync_start = 1920 + 16,
	.vsync_end = 1920 + 16 + 4,
	.vtotal = 1920 + 16 + 4 + 16,
	.width_mm = 135,
	.height_mm = 216,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int gtaxl_get_modes(struct drm_panel *panel,
			   struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &gtaxl_mode);
}

static const struct drm_panel_funcs gtaxl_panel_funcs = {
	.prepare = gtaxl_prepare,
	.enable = gtaxl_enable,
	.disable = gtaxl_disable,
	.unprepare = gtaxl_unprepare,
	.get_modes = gtaxl_get_modes,
};

static int gtaxl_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = GTAXL_UI_BRIGHTNESS_MAX,
		.brightness = GTAXL_UI_BRIGHTNESS_DEFAULT,
	};
	struct gtaxl_panel *p;
	int ret;

	p = devm_drm_panel_alloc(dev, struct gtaxl_panel, panel,
				 &gtaxl_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(p))
		return PTR_ERR(p);

	p->dsi = dsi;
	mutex_init(&p->lock);
	mipi_dsi_set_drvdata(dsi, p);

	p->iovcc = devm_regulator_get(dev, "iovcc");
	if (IS_ERR(p->iovcc))
		return dev_err_probe(dev, PTR_ERR(p->iovcc),
				     "Failed to get iovcc supply\n");

	p->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(p->vdd))
		return dev_err_probe(dev, PTR_ERR(p->vdd),
				     "Failed to get vdd supply\n");

	/* Logical 0 = reset released: sets the direction without glitching a lit panel */
	p->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(p->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(p->reset_gpio),
				     "Failed to get reset GPIO\n");

	p->backlight = devm_backlight_device_register(dev, dev_name(dev), dev, p,
						      &gtaxl_bl_ops, &props);
	if (IS_ERR(p->backlight))
		return dev_err_probe(dev, PTR_ERR(p->backlight),
				     "Failed to register backlight\n");

	/* The panel must be prepared after the DSI host is up */
	p->panel.prepare_prev_first = true;
	drm_panel_add(&p->panel);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO_NO_HFP |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_MODE_LPM;

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret) {
		drm_panel_remove(&p->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void gtaxl_remove(struct mipi_dsi_device *dsi)
{
	struct gtaxl_panel *p = mipi_dsi_get_drvdata(dsi);

	/* Drop the regulator enable counts before devm releases the supplies */
	drm_panel_disable(&p->panel);
	drm_panel_unprepare(&p->panel);
	drm_panel_remove(&p->panel);
}

static const struct of_device_id gtaxl_of_match[] = {
	{ .compatible = "samsung,gtaxllte-boe-hx8279d" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gtaxl_of_match);

static struct mipi_dsi_driver gtaxl_driver = {
	.probe = gtaxl_probe,
	.remove = gtaxl_remove,
	.driver = {
		.name = "panel-samsung-gtaxllte-hx8279d",
		.of_match_table = gtaxl_of_match,
	},
};
module_mipi_dsi_driver(gtaxl_driver);

MODULE_DESCRIPTION("Samsung Galaxy Tab A 10.1 (2016) LTE HX8279-D DSI panel driver");
MODULE_LICENSE("GPL");
