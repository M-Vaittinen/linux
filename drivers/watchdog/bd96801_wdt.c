// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 ROHM Semiconductors
// ROHM BD96801 watchdog driver

#include <linux/bcd.h>
#include <linux/kernel.h>
#include <linux/mfd/rohm-bd96801.h>
#include <linux/mfd/rohm-generic.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>

#define BD96801_WD_TYPE_MASK		0x4
#define BD96801_WD_TYPE_SLOW		0x4
#define BD96801_WD_TYPE_WIN		0x0

#define BD96801_WD_EN_MASK		0x3
#define BD96801_WD_IF_EN		0x1
#define BD96801_WD_QA_EN		0x2
#define BD96801_WD_DISABLE		0x0

#define BD96801_WD_ASSERT_MASK		0x8
#define BD96801_WD_ASSERT_RST		0x8
#define BD96801_WD_ASSERT_IRQ		0x0

#define BD96801_WD_FEED_MASK		0x1
#define BD96801_WD_FEED			0x1

/*
 * BD96801 WDG supports window mode (not used by this driver)
 * Thus the TMO consists of SHORT and LONG timeout values. SHORT time
 * is meaningfull only in window mode where feeding period shorter
 * than SHORT would be an error. LONG time is used (also by this driver)
 * to detect if feeding is not occurring withing give time limit (SoC
 * SW hangs). And why all this babblinge here? Because the LONG time is
 * created as (SHORT time << (BD96801_WD_TMO_SHIFT + 1))
 */

/* SHORT TMO in uS */
#define BD96801_WD_TMO_SHORT_DEFAULT_US	431570
#define BD96801_WD_TMO_SHORT_MASK	0x70
#define BD96801_WD_TMO_SHORT_DEFAULT	0x70

#define BD96801_WD_TMO_SHIFT_MASK	0x3

#define BD96801_WD_TMO_DEFAULT_SHIFT	3

/* 431570 << 4 is roughly 6.9 Sec */
#define BD96801_WD_TMO_DEFAULT	(BD96801_WD_TMO_SHORT_DEFAULT_US <<	\
				 (BD96801_WD_TMO_DEFAULT_SHIFT + 1))

/* 
 * In order to keep this simple we won't change the BD96801_WD_TMO_SHORT
 * (FAST_NG in data-sheet). We only change the BD96801_WD_TMO_SHIFT
 * (RATIO_TO in data-sheet).
 */

/* Maximum TMO is roughly 7 Sec */
#define WDT_MAX_MS ((BD96801_WD_TMO_SHORT_DEFAULT_US <<			\
		    (BD96801_WD_TMO_SHIFT_MASK + 1)) / 1000)

/* Minimum TMO is roughly 863 mS */
#define WDT_MIN_MS ((BD96801_WD_TMO_SHORT_DEFAULT_US << 1) / 1000)

#define DEFAULT_TIMEOUT_MS (BD96801_WD_TMO_DEFAULT / 1000)

#define SEC_TO_TMO(msec) ({						\
	int _tmp, _i = 0;						\
									\
	for (_tmp = BD96801_WD_TMO_SHORT_DEFAULT_US / 1000 << 1;	\
	     _tmp << _i < (msec) && _tmp << _i < WDT_MAX_MS; _i++)		\
		;							\
	_i;								\
})

struct wdtbd96801 {
	struct device *dev;
	struct regmap *regmap;
	struct rohm_regmap_dev *mfd;
	struct watchdog_device wdt;
};

static int bd96801_wdt_ping(struct watchdog_device *wdt)
{
	struct wdtbd96801 *w = watchdog_get_drvdata(wdt);

	dev_dbg(w->dev, "WDT ping...\n");

	return regmap_update_bits(w->regmap, BD96801_REG_WD_FEED,
				 BD96801_WD_FEED_MASK, BD96801_WD_FEED);
}

static int bd96801_wdt_start(struct watchdog_device *wdt)
{
	struct wdtbd96801 *w = watchdog_get_drvdata(wdt);
	int ret;

	ret = regmap_update_bits(w->regmap, BD96801_REG_WD_CONF,
				 BD96801_WD_EN_MASK, BD96801_WD_IF_EN);
	dev_dbg(w->dev, "WDT started\n");
	return ret;
}

static int bd96801_wdt_stop(struct watchdog_device *wdt)
{
	struct wdtbd96801 *w = watchdog_get_drvdata(wdt);

	dev_dbg(w->dev, "WDT stopping\n");
	return regmap_update_bits(w->regmap, BD96801_REG_WD_CONF,
				 BD96801_WD_EN_MASK, BD96801_WD_DISABLE);
}

static int bd96801_wdt_set_timeout(struct watchdog_device *wdt,
				   unsigned int timeout)
{
	int ret;
	struct wdtbd96801 *w = watchdog_get_drvdata(wdt);

	u8 reg = SEC_TO_TMO(timeout);

	ret = regmap_update_bits(w->regmap, BD96801_REG_WD_TMO,
				 BD96801_WD_TMO_SHIFT_MASK, reg);
	if (ret)
		dev_err(w->dev, "Failed to set WDT timeout\n");

	return ret;
}

static const struct watchdog_info bd96801_wdt_info = {
	.identity = "bd96801-wdt",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops bd96801_wdt_ops = {
	.start		= bd96801_wdt_start,
	.stop		= bd96801_wdt_stop,
	.set_timeout	= bd96801_wdt_set_timeout,
};

static int init_wdg_hw(struct wdtbd96801 *w)
{
	int ret;
	struct device_node *np = w->dev->parent->of_node;

	/*
	 * This driver does not support windowed mode. Configure to do
	 * only delayed feeding detetion.
	 */
	ret = regmap_update_bits(w->regmap, BD96801_REG_WD_CONF,
				 BD96801_WD_TYPE_MASK, BD96801_WD_TYPE_SLOW);
	if (ret) {
		dev_err(w->dev, "Failed to set WDT type\n");
		return ret;
	}

	/*
	 * We set the SHORT TMO here as this driver excepts SHORT TMO to
	 * be fixed. Othervice setting timeout would yield incorrect
	 * timeouts - potentially causing unwanted reset at startup.
	 *
	 * Shortest TMO using the BD96801_WD_TMO_SHORT_DEFAULT is
	 * aroung 800mS so it should be Ok to set only it.
	 */
	ret = regmap_update_bits(w->regmap, BD96801_REG_WD_TMO,
				 BD96801_WD_TMO_SHORT_MASK,
				 BD96801_WD_TMO_SHORT_DEFAULT);
	if (ret) {
		dev_err(w->dev, "Failed to set WDT timeout\n");
		return ret;
	}

	ret = of_property_match_string(np, "rohm,wdg-action", "prstb");
	if (ret > 0) {
		ret = regmap_update_bits(w->regmap, BD96801_REG_WD_CONF,
				 BD96801_WD_ASSERT_MASK,
				 BD96801_WD_ASSERT_RST);
		return ret;
	}

	ret = of_property_match_string(np, "rohm,wdg-action", "intb-only");
	if (ret > 0) {
		ret = regmap_update_bits(w->regmap, BD96801_REG_WD_CONF,
				 BD96801_WD_ASSERT_MASK,
				 BD96801_WD_ASSERT_IRQ);
		return ret;
	}

	return 0;
}

static int bd96801_wdt_probe(struct platform_device *pdev)
{
	struct rohm_regmap_dev *bd96801;
	struct wdtbd96801 *w;
	int ret;
	unsigned int reg;

	bd96801 = dev_get_drvdata(pdev->dev.parent);
	if (!bd96801) {
		dev_err(&pdev->dev, "No MFD driver data\n");
		return -EINVAL;
	}
	w = devm_kzalloc(&pdev->dev, sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;

	w->regmap = bd96801->regmap;
	w->mfd = bd96801;
	w->dev = &pdev->dev;

	w->wdt.info = &bd96801_wdt_info;
	w->wdt.ops =  &bd96801_wdt_ops;
	w->wdt.min_hw_heartbeat_ms = WDT_MIN_MS;
	w->wdt.max_hw_heartbeat_ms = WDT_MAX_MS;
	w->wdt.parent = pdev->dev.parent;
	w->wdt.timeout = DEFAULT_TIMEOUT_MS;
	watchdog_set_drvdata(&w->wdt, w);

	ret = regmap_read(w->regmap, BD96801_REG_WD_CONF, &reg);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get the watchdog state\n");
		return ret;
	}
	if ((reg & BD96801_WD_EN_MASK) != BD96801_WD_DISABLE) {
		if ((reg & BD96801_WD_EN_MASK) != BD96801_WD_IF_EN) {
			dev_err(&pdev->dev,
				"watchdog set to Q&A mode - exiting\n");
			return -EINVAL;
		}
		dev_dbg(&pdev->dev, "watchdog was running during probe\n");
		set_bit(WDOG_HW_RUNNING, &w->wdt.status);
	}

	ret = init_wdg_hw(w);
	if (ret)
		return ret;

	watchdog_init_timeout(&w->wdt, 0, pdev->dev.parent);

	ret = bd96801_wdt_set_timeout(&w->wdt, w->wdt.timeout);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set the watchdog timeout\n");
		return ret;
	}

	ret = devm_watchdog_register_device(&pdev->dev, &w->wdt);
	if (ret < 0)
		dev_err(&pdev->dev, "watchdog registration failed: %d\n", ret);

	return ret;
}

static struct platform_driver bd96801_wdt = {
	.driver = {
		.name = "bd96801-wdt"
	},
	.probe = bd96801_wdt_probe,
};

module_platform_driver(bd96801_wdt);

MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("BD96801 watchdog driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bd96801-wdt");
