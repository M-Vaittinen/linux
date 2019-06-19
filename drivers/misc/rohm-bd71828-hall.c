// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2019 ROHM Semiconductors

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/rohm-bd71828.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct bd71828_hall {
	struct regmap *regmap;
	struct device *dev;
	unsigned int open_state;
};

static void hall_send_open_event(struct device *dev)
{
	char *envp[] = {"HALLSENSOR=opened", NULL};

	dev_dbg(dev, "KERNEL: send event: close\n");
	kobject_uevent_env(&dev->kobj, KOBJ_ONLINE, envp);
}

static void hall_send_close_event(struct device *dev)
{
	char *envp[] = {"HALLSENSOR=opened", NULL};

	dev_dbg(dev, "KERNEL: send event: open\n");
	kobject_uevent_env(&dev->kobj, KOBJ_OFFLINE, envp);
}

static irqreturn_t hall_hndlr(int irq, void *data)
{
	struct bd71828_hall *hall = data;
	unsigned int reg;
	int ret;

	ret = regmap_read(hall->regmap, BD71828_REG_IO_STAT, &reg);
	if (ret) {
		dev_err(hall->dev, "getting HALL status failed\n");
		return IRQ_NONE;
	}
	if (hall->open_state == (reg & BD71828_HALL_STATE_MASK))
		hall_send_open_event(hall->dev);
	else 
		hall_send_close_event(hall->dev);

	return IRQ_HANDLED;
}

static int bd71828_probe(struct platform_device *pdev)
{
	int irq, ret;
	struct bd71828_hall *hall;
	struct rohm_regmap_dev *mfd;

	mfd = dev_get_drvdata(pdev->dev.parent);
	if (!mfd) {
		dev_err(&pdev->dev, "No MFD driver data\n");
		return -EINVAL;
	}
	hall = devm_kzalloc(&pdev->dev, sizeof(*hall), GFP_KERNEL);
	if (!hall)
		return -ENOMEM;

	hall->regmap = mfd->regmap;
	hall->dev = &pdev->dev;

	if (of_property_read_bool(pdev->dev.parent->of_node,
				  "rohm,lid-open-high"))
		hall->open_state = BD71828_HALL_STATE_MASK;

	irq = platform_get_irq_byname(pdev, "bd71828-hall");
	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, &hall_hndlr,
					IRQF_ONESHOT, "bd70528-hall", hall);
//	if (ret)
	return ret;
}

static struct platform_driver bd71828_hall = {
	.driver = {
		.name = "bd71828-lid"
	},
	.probe = bd71828_probe,
};

module_platform_driver(bd71828_hall);

MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("BD71828 LID event driver");
MODULE_LICENSE("GPL");
