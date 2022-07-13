// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2022 ROHM Semiconductors
//
// ROHM/KIONIX KX022 accelerometer driver

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
//#ifdef CONFIG_OF
//No need to have this ifdef - of.h has the stubs, right?
#include <linux/of.h>
#include <linux/of_gpio.h>
//#endif
#include <linux/regulator/consumer.h>

#include "kionix-kx022a.h"

/* TODO: Check what is correct value? */
#define KX022_INPUT_DEV_EVENTS_NUM 120

/* Driver state bits */
#define KX022_STATE_STANDBY 0
#define KX022_STATE_STRM BIT(0)
#define KX022_STATE_FIFO BIT(1)
#define KX022_STATE_TILT BIT(2)
#define KX022_STATE_RUNNING BIT(3)

/* Regmap configs */
/* TODO: Add rest of the special regs */
static const struct regmap_range kx022a_volatile_ranges[] = {
	{
		.range_min = KX022_REG_XHP_L,
		.range_max = KX022_REG_COTR,
	}, {
		.range_min = KX022_REG_TSCP,
		.range_max = KX022_REG_INT_REL,
	}, {
		.range_min = KX022_REG_BUF_STATUS_1,
		.range_max = KX022_REG_BUF_READ,
/*	}, {
		.range_min = KX022_REG_SELF_TEST,
		.range_max = KX022_REG_SELF_TEST,
	}, {
		.range_min = KX022_REG_BUF_CLEAR,
		.range_max = KX022_REG_BUF_CLEAR, */
	},
};

static const struct regmap_access_table kx022a_volatile_regs = {
	.yes_ranges = &kx022a_volatile_ranges[0],
	.n_yes_ranges = ARRAY_SIZE(kx022a_volatile_ranges),
};

static const struct regmap_range kx022a_precious_ranges[] = {
	{
		.range_min = KX022_REG_INT_REL,
		.range_max = KX022_REG_INT_REL,
	},
};

static const struct regmap_access_table kx022a_precious_regs = {
	.yes_ranges = &kx022a_precious_ranges[0],
	.n_yes_ranges = ARRAY_SIZE(kx022a_precious_ranges),
};

/*
 * The HW does not set WHO_AM_I reg as read-only but we don't want to write it
 * so we still include it in the read-only ranges.
 */
static const struct regmap_range kx022a_read_only_ranges[] = {
	{
		.range_min = KX022_REG_XHP_L,
		.range_max = KX022_REG_INT_REL,
	}, {
		.range_min = KX022_REG_BUF_STATUS_1,
		.range_max = KX022_REG_BUF_STATUS_2,
	}, {
		.range_min = KX022_REG_BUF_READ,
		.range_max = KX022_REG_BUF_READ,
	},
};

static const struct regmap_access_table kx022a_ro_regs = {
	.yes_ranges = &kx022a_read_only_ranges[0],
	.n_yes_ranges = ARRAY_SIZE(kx022a_read_only_ranges),
};

static const struct regmap_range kx022a_write_only_ranges[] = {
	{
		.range_min = KX022_REG_BTS_WUF_TH,
		.range_max = KX022_REG_BTS_WUF_TH,
	}, {
		.range_min = KX022_REG_MAN_WAKE,
		.range_max = KX022_REG_MAN_WAKE,
	}, {
		.range_min = KX022_REG_SELF_TEST,
		.range_max = KX022_REG_SELF_TEST,
	}, {
		.range_min = KX022_REG_BUF_CLEAR,
		.range_max = KX022_REG_BUF_CLEAR,
	},
};

static const struct regmap_access_table kx022a_wo_regs = {
	.yes_ranges = &kx022a_write_only_ranges[0],
	.n_yes_ranges = ARRAY_SIZE(kx022a_write_only_ranges),
};

static const struct regmap_range kx022a_noinc_read_ranges[] = {
	{
		.range_min = KX022_REG_BUF_READ,
		.range_max = KX022_REG_BUF_READ,
	},
};

static const struct regmap_access_table kx022a_nir_regs = {
	.yes_ranges = &kx022a_noinc_read_ranges[0],
	.n_yes_ranges = ARRAY_SIZE(kx022a_noinc_read_ranges),
};

const struct regmap_config kx022a_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &kx022a_volatile_regs,
	.wr_table = &kx022a_wo_regs,
	.rd_table = &kx022a_ro_regs,
	.rd_noinc_table = &kx022a_nir_regs,
	.precious_table = &kx022a_precious_regs,
	.max_register = KX022_MAX_REGISTER,
	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(kx022a_regmap);

struct kx022a_data {
	struct regmap *regmap;
	struct device *dev;
	struct input_dev *accel_input_dev;
	unsigned int g_range;
	struct mutex state_lock;
	unsigned int state;
	unsigned long req_sample_interval_ms;
	unsigned long odr_interval_ms;


	unsigned int max_latency; /* FIFO flush period in ms */
	ktime_t fifo_last_ts;
//	ktime_t fifo_last_read;

	u8 x_idx;
	u8 y_idx;
	u8 z_idx;
	bool invert_x;
	bool invert_y;
	bool invert_z;
};

/*
 * Custom sysfs interfaces as requested by customers and as seen in many
 * Kionix Accel drivers
 */

/**
 * kx022a_sysfs_get_max_latency - return fifo read latency
 */
static ssize_t kx022a_sysfs_get_max_latency(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kx022a_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->max_latency);
}

/**
 * kx022a_sysfs_set_max_latency - set fifo read latency
 */
static ssize_t kx022a_sysfs_set_max_latency(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned long value;
	struct kx022a_data *data = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&data->state_lock);
	if (data->state == KX022_STATE_RUNNING) {
		dev_err(data->dev, "measurement running\n");
		len = -EBUSY;
	} else {
		data->max_latency = value;
	}
	mutex_unlock(&data->state_lock);

	return len;
}

static struct device_attribute dev_attr_accel_max_latency = __ATTR(max_latency,
		0644,
		kx022a_sysfs_get_max_latency,
		kx022a_sysfs_set_max_latency);


static struct attribute *kx022a_sysfs_attrs[] = {
//	&dev_attr_accel_tilt_enable.attr,
//	&dev_attr_accel_enable.attr,
//	&dev_attr_accel_delay.attr,
	&dev_attr_accel_max_latency.attr,
	NULL
};

static struct attribute_group kx022a_accel_attribute_group = {
	.attrs = kx022a_sysfs_attrs
};

static int kx022a_set_g_range(struct kx022a_data *data, unsigned int g_range)
{
	int gsel;

	switch (g_range) {
	case 2000:
		gsel = KX022_GSEL_2;
		break;
	case 4000:
		gsel = KX022_GSEL_4;
		break;
	case 8000:
		gsel = KX022_GSEL_8;
		break;
	case 16000:
		gsel = KX022_GSEL_16;
		break;
	default:
		dev_err(data->dev, "Unsupported G-range %u\n", g_range);
		return -EINVAL;
	}

	return regmap_update_bits(data->regmap, KX022_REG_CNTL, KX022_MASK_GSEL,
				  gsel);
}

static int sanity_check_axis(u32 *axis)
{
	unsigned int i, chk = 0;

	for (i = 0; i < 3; i++)
		chk |= (1 << axis[i]);

	return (chk != GENMASK(2, 0));
}

static int kx022a_parse_dt(struct kx022a_data *data)
{
	int ret, i;
	u32 g_range, axis_order[3];
	struct fwnode_handle *fw = dev_fwnode(data->dev);

	if (!fw) {
		dev_err(data->dev, "no fwnode\n");

		return -ENODEV;
	}

	ret = fwnode_property_read_u32(fw, "kionix,g-range-milli-g", &g_range);
	if (ret) {
		if (ret != -EINVAL)
			return ret;

		dev_warn(data->dev, "No G-range, defaulting 2 g\n");
		g_range = 2000;
	}

	ret = fwnode_property_read_u32_array(fw, "kionix,xyz-axis-map", NULL, 0);
	if (ret != 3) {
		if (ret) {
			dev_err(data->dev, "Bad axis map\n");

			return (ret < 0) ? ret : -EINVAL;
		}
		for (i = 0; i < 3; i++)
			axis_order[i] = i;
	} else {
		ret = fwnode_property_read_u32_array(fw, "kionix,xyz-axis-map",
						     &axis_order[0], 3);
		if (ret)
			return ret;
	}

	ret = sanity_check_axis(&axis_order[0]);
	if (ret) {
		dev_err(data->dev, "Bad axis map [%u %u %u]\n", axis_order[0],
			axis_order[1], axis_order[2]);

		return EINVAL;
	}

	data->x_idx = axis_order[0];
	data->y_idx = axis_order[1];
	data->z_idx = axis_order[2];

	data->invert_x = fwnode_property_present(fw, "kionix,invert-x-axis");
	data->invert_y = fwnode_property_present(fw, "kionix,invert-y-axis");
	data->invert_z = fwnode_property_present(fw, "kionix,invert-z-axis");

	return kx022a_set_g_range(data, g_range);
}

static void kx022a_map_data(struct kx022a_data *data, int *raw_xyz, int *xyz)
{
	xyz[0] = data->invert_x ? -raw_xyz[data->x_idx] : raw_xyz[data->x_idx];
	xyz[1] = data->invert_y ? -raw_xyz[data->y_idx] : raw_xyz[data->y_idx];
	xyz[2] = data->invert_z ? -raw_xyz[data->z_idx] : raw_xyz[data->z_idx];
}

static void kx022a_convert_raw_data(struct kx022a_data *data, s16 *raw_xyz,
				   int *xyz)
{
	int i, tmp[3];

	for (i = 0; i < 3; i++)
		tmp[i] = le16_to_cpu(raw_xyz[i]);

	kx022a_map_data(data, &tmp[0], xyz);
}

static int kx022a_data_read_xyz(struct kx022a_data *data, int *xyz)
{
	int ret;
	s16 raw_xyz[3] = { 0 };

	ret = regmap_bulk_read(data->regmap, KX022_REG_XOUT_L, &raw_xyz[0], 6);
	if (ret)
		return ret;

	kx022a_convert_raw_data(data, &raw_xyz[0], &xyz[0]);

	return 0;
}

#ifdef CALIBRATE
static void kx122_data_calibrate_xyz(struct kx122_data *data, int *xyz)
{
	xyz[0] -= data->accel_cali[0];
	xyz[1] -= data->accel_cali[1];
	xyz[2] -= data->accel_cali[2];

	return 0;
}
#endif
#define KX132_1211_ODCNTL_OSA_0P781
/*
 * The sensor HW can support ODR up to 1600 Hz - which is beyond what most of
 * Linux CPUs can handle w/o dropping samples. Also, the low power mode is not
 * available for higher sample rates. Thus the driver only supports 200 Hz and
 * slower ODRs. Slowest being 0.78 Hz
 */
#define MIN_ODR_INTERVAL_MS 5
#define MAX_ODR_INTERVAL_MS 1280
#define NUM_SUPPORTED_ODR 9
/* static const struct {
	unsigned int cutoff;
	u8 mask;
}
static const int delay_table kx022a_odr_delay_table[] = {
	2560, 1280,
	{ 5, KX132_1211_ODCNTL_OSA_400},
	{ 10, KX132_1211_ODCNTL_OSA_200},
	{ 20, KX132_1211_ODCNTL_OSA_100},
	{ 40, KX132_1211_ODCNTL_OSA_50},
	{ 80, KX132_1211_ODCNTL_OSA_25},
	{ 160, KX132_1211_ODCNTL_OSA_12P5},
	{ 320, KX132_1211_ODCNTL_OSA_6P25},
	{ 640, KX132_1211_ODCNTL_OSA_3P125},
	{ 1280,	KX132_1211_ODCNTL_OSA_1P563},
	{ 2560, KX132_1211_ODCNTL_OSA_0P781},};
*/
static int kx022a_data_delay_set(struct kx022a_data *data, unsigned long delay)
{
	unsigned long tmp = 5;
	int i;

	if (delay < MIN_ODR_INTERVAL_MS)
		delay = MIN_ODR_INTERVAL_MS;
	else if (delay > MAX_ODR_INTERVAL_MS)
		delay = MAX_ODR_INTERVAL_MS;

	for (i = 0; i < NUM_SUPPORTED_ODR; i++, tmp *= 2)
		if (delay <= tmp)
			break;

	if (delay != tmp)
		dev_warn(data->dev,
			 "Unsupported ODR delay %lu, using closest smaller %lu\n",
			 delay, tmp);
	dev_dbg(data->dev, "ODR delay %lu\n", tmp);
//	data->req_sample_interval_ms = delay;
	data->odr_interval_ms = tmp;

//	if (!data->power_enabled)
//		return 0;

	return regmap_update_bits(data->regmap, KX022_REG_ODCNTL,
		KX022_MASK_ODR, i);
}

static int kx022a_fifo_set_wmi(struct kx022a_data *data)
{
	u8 threshold;

	/* how many samples stored to FIFO before wmi */
	if (data->max_latency / data->odr_interval_ms > KX022_FIFO_MAX_WMI_TH)
		threshold = KX022_FIFO_MAX_WMI_TH;
	else
		threshold = data->max_latency / data->odr_interval_ms;

	return regmap_update_bits(data->regmap, KX022_REG_BUF_CNTL1,
				 KX022_MASK_WM_TH, threshold);
}

static int kx022a_fifo_enable(struct kx022a_data *data)
{
	int ret = 0;

	if (data->state & KX022_STATE_FIFO)
		return 0;

	mutex_lock(&data->state_lock);
	if (data->state & KX022_STATE_FIFO)
		goto unlock_out;

	/* update sensor ODR */
	ret = kx022a_data_delay_set(data, data->req_sample_interval_ms);
	if (ret < 0)
		goto unlock_out;

	ret = kx022a_fifo_set_wmi(data);
	if (ret < 0)
		goto unlock_out;

	/* Set watermark threshold */
	if (ret)
		goto unlock_out;

	/* set FIFO buffer on at data res 16bit */
	ret = regmap_update_bits(data->regmap, KX022_REG_BUF_CNTL2,
				 KX022_MASK_BUFE | KX022_MASK_BRES,
				 KX022_MASK_BUFE | KX022_MASK_BRES);
	if (ret)
		goto unlock_out;

	/* NOTE, uses int1_irq */

	/* timestamp when fifo started */
	data->fifo_last_ts = ktime_get_boottime();
//	data->fifo_last_read =	data->fifo_last_ts;

	data->state |= KX022_STATE_FIFO;
	ret = regmap_set_bits(data->regmap, KX022_REG_INC4,
			      KX022_MASK_WMI1);

unlock_out:
	mutex_unlock(&data->state_lock);

	return ret;
}

static void kx022a_input_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts)
{
	input_report_abs(dev, ABS_X, xyz[0]);
	input_report_abs(dev, ABS_Y, xyz[1]);
	input_report_abs(dev, ABS_Z, xyz[2]);
	input_sync(dev);
}

static void kx022a_strm_report_data(struct kx022a_data *data)
{
	int ret;
	int xyz[3];
	ktime_t ts;

	ret = kx022a_data_read_xyz(data, &xyz[0]);

	if (ret) {
		dev_err(data->dev, "i2c read/write error\n");

		return;
	}

	ts = ktime_get_boottime();
#ifdef CALIBRATE
	kx022a_data_calibrate_xyz(data, xyz);
#endif
	kx022a_input_report_xyz(data->accel_input_dev, &xyz[0], ts);
}

#define KX022_FIFO_SIZE_BYTES 256
/* 3 axis, 2 bytes of data for each of the axis */
#define KX022_FIFO_SAMPLES_SIZE_BYTES 6
#define KX022_FIFO_MAX_SAMPLES (KX022_FIFO_SIZE_BYTES/KX022_FIFO_SAMPLES_SIZE_BYTES)

#define KX022_INPUT_DEV_EVENTS_NUM 120

static void kx022a_fifo_report_data(struct kx022a_data *data)
{
	unsigned int fifo_bytes;
	ktime_t interval;
	int ret, num_samples, i;
	s16 raw_xyz[3 * KX022_FIFO_MAX_SAMPLES] = {0};
	int xyz[3];

	/* get number of bytes in fifo */
	ret = regmap_read(data->regmap, KX022_REG_BUF_STATUS_1, &fifo_bytes);
	if (ret) {
		dev_err(data->dev, "FIFO size read failed %d\n", ret);
		return;
	}

	fifo_bytes = le16_to_cpu(fifo_bytes);
	if (fifo_bytes > sizeof(raw_xyz)) {
		dev_warn(data->dev, "fifo too big. missing %u bytes of data\n",
			 fifo_bytes - sizeof(raw_xyz));
		fifo_bytes = sizeof(raw_xyz);
	}
	if (fifo_bytes % num_samples)
		dev_err(data->dev, "Bad FIFO alignment. Data may be corrupt\n");

	interval = ktime_set(0, data->odr_interval_ms * NSEC_PER_MSEC);

	ret = regmap_bulk_read(data->regmap, KX022_REG_BUF_READ,
			       (void *)&raw_xyz[0], fifo_bytes);
	if (ret < 0) {
		dev_err(data->dev, "FIFO read failed %d\n", ret);
		return;
	}
	num_samples = fifo_bytes / KX022_FIFO_SAMPLES_SIZE_BYTES;

	for (i = 0; i < num_samples; i++) {
		data->fifo_last_ts = ktime_add(data->fifo_last_ts, interval);
		kx022a_convert_raw_data(data, &raw_xyz[i*3], &xyz[0]);
	//	kx022a_data_calibrate_xyz(data, xyz);
		kx022a_input_report_xyz(data->accel_input_dev, xyz, data->fifo_last_ts);
	}
}

static irqreturn_t kx022a_irq_thread(int irq, void *d)
{
	struct kx022a_data *data = d;
	int status, ret;

	ret = regmap_read(data->regmap, KX022_REG_INS2, &status);
	if (ret || !status)
		return IRQ_NONE;

	if (status & KX022_MASK_INS2_DRDY)
		kx022a_strm_report_data(data);

	if (status & KX122_MASK_INS2_WMI) {
		mutex_lock(&data->state_lock);
		if (data->state & KX022_STATE_FIFO) {
			kx022a_fifo_report_data(data);
		}
		mutex_unlock(&data->state_lock);
	}

	return IRQ_HANDLED;
}

static void kx022a_clean_sysfs(void *d)
{
	struct kx022a_data *data = (struct kx022a_data *)d;

	sysfs_remove_group(&data->accel_input_dev->dev.kobj,
			   &kx022a_accel_attribute_group);
}

static int kx022a_turn_off(struct kx022a_data *data)
{
	return regmap_clear_bits(data->regmap, KX022_REG_CNTL, KX022_MASK_PC1);
}

static void kx022a_close(struct input_dev *idev)
{
	struct kx022a_data *data = input_get_drvdata(idev);

	mutex_lock(&data->state_lock);
	data->state &= KX022_STATE_RUNNING;
	mutex_unlock(&data->state_lock);

	kx022a_turn_off(data);
}

static int kx022a_prepare_irq(struct kx022a_data *data)
{
	static const int mask =	KX022_MASK_IEN1 | KX022_MASK_IPOL1 |
				KX022_MASK_ITYP;
	static const int val =	KX022_MASK_IEN1 | KX022_IPOL_LOW |
				KX022_ITYP_LEVEL;
	int ret;

	ret = kx022a_fifo_enable(data);
	if (ret)
		return ret;

	return regmap_update_bits(data->regmap, KX022_REG_INC1, mask, val);
}

static int kx022a_unprepare_irq(struct kx022a_data *data)
{
	return regmap_update_bits(data->regmap, KX022_REG_INC1, KX022_MASK_IEN1, 0);
}

static int kx022a_start(struct kx022a_data *data)
{
	int ret;

	ret = kx022a_prepare_irq(data);
	if (ret)
		return ret;

	ret = regmap_set_bits(data->regmap, KX022_REG_CNTL, KX022_MASK_PC1);
	if (ret)
		goto err_start;

	return 0;

err_start:
	kx022a_unprepare_irq(data);

	return ret;
}

static int kx022a_open(struct input_dev *idev)
{
	struct kx022a_data *data = input_get_drvdata(idev);
	int ret;

	mutex_lock(&data->state_lock);
	data->state |= KX022_STATE_RUNNING;
	mutex_unlock(&data->state_lock);

	ret = kx022a_start(data);
	if (ret) {
		mutex_lock(&data->state_lock);
		data->state &= ~KX022_STATE_RUNNING;
		mutex_unlock(&data->state_lock);
	}

	return ret;
}

int kx022a_probe_internal(struct device *dev, int irq, int input_bus)
{
	struct regmap *regmap;
	struct kx022a_data *data;
	unsigned int chip_id;
	struct input_dev *id;
	int ret;

	if (WARN_ON(!dev))
		return -ENODEV;

	regmap = dev_get_regmap(dev, NULL);
	if (!regmap) {
		dev_err(dev, "no regmap\n");

		return -EINVAL;
	}

	ret = regmap_read(regmap, KX022_REG_WHO, &chip_id);
	if (ret) {
		dev_err(dev, "Failed to access sensor\n");

		return ret;
	}

	if (chip_id != KX022_ID) {
		dev_err(dev, "unsupported device 0x%x\n", chip_id);

		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = regmap;
	data->dev = dev;

	/* The sensor must be turned off for configuration */
	ret = kx022a_turn_off(data);
	if (ret)
		return ret;

	mutex_init(&data->state_lock);

	ret = kx022a_parse_dt(data);
	if (ret)
		return ret;

	id = devm_input_allocate_device(dev);
	if (!id)
		return -ENOMEM;

	data->accel_input_dev = id;

	id->name = "kx022-accel";
	id->id.bustype = input_bus;
	id->id.vendor = (int)"KION";
	id->dev.parent = dev;
	id->open = kx022a_open;
	id->close = kx022a_close;
	set_bit(EV_ABS, id->evbit);
	input_set_abs_params(id, ABS_X, INT_MIN, INT_MAX,0,0);
	input_set_abs_params(id, ABS_Y, INT_MIN, INT_MAX,0,0);
	input_set_abs_params(id, ABS_Z, INT_MIN, INT_MAX,0,0);

	input_set_events_per_packet(id, KX022_INPUT_DEV_EVENTS_NUM);

	input_set_drvdata(id, data);

	ret = sysfs_create_group(&data->accel_input_dev->dev.kobj,
					&kx022a_accel_attribute_group);
	if (ret) {
		dev_err(data->dev, "accel sysfs create fail\n");
		return ret;
	}
	ret = devm_add_action_or_reset(dev, &kx022a_clean_sysfs, data);
	if (ret)
		return ret;

	ret = input_register_device(id);
        if (ret) {
                dev_err(dev, "Failed to register device\n");
                return ret;
        }

	/* TODO: Clean sysfs upon failure */

	return devm_request_threaded_irq(data->dev, irq, NULL, &kx022a_irq_thread,
				  IRQF_ONESHOT, "kx022", data);
}
EXPORT_SYMBOL_GPL(kx022a_probe_internal);

/*
static const struct dev_pm_ops kx022a_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(kx022a_suspend, kx022a_resume)
	SET_RUNTIME_PM_OPS(kx022a_runtime_suspend, kx022a_runtime_resume, NULL)
};
*/

MODULE_DESCRIPTION("ROHM/Kionix KX022 accelerometer driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");
