// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2022 ROHM Semiconductors
//
// ROHM/KIONIX KX022 accelerometer driver

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "kionix-kx022a.h"

#define KX022A_FIFO_LENGTH 41

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
	.no_ranges = &kx022a_read_only_ranges[0],
	.n_no_ranges = ARRAY_SIZE(kx022a_read_only_ranges),
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
	.no_ranges = &kx022a_write_only_ranges[0],
	.n_no_ranges = ARRAY_SIZE(kx022a_write_only_ranges),
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
	.rd_table = &kx022a_wo_regs,
	.wr_table = &kx022a_ro_regs,
	.rd_noinc_table = &kx022a_nir_regs,
	.precious_table = &kx022a_precious_regs,
	.max_register = KX022_MAX_REGISTER,
	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(kx022a_regmap);

enum kx022a_trigger_id {
	KX022A_TRIGGER_DATA_READY,
	KX022A_TRIGGERS,
};

struct kx022a_trigger;
struct kx022a_data;

struct kx022a_trigger {
	struct kx022a_data *data;
	struct iio_trigger *indio_trig;
	bool enabled;
	const char *name;
};

static const struct kx022a_trigger kx022a_triggers[KX022A_TRIGGERS] = {
	{
		.name = "%sdata-rdy-dev%d",
	}
};

struct kx022a_data {
	int irq;
	struct regmap *regmap;
	struct kx022a_trigger triggers[KX022A_TRIGGERS];
	struct device *dev;
	unsigned int g_range;
	struct mutex mutex;
	unsigned int state;
	unsigned long req_sample_interval_ms;
	unsigned long odr_interval_ms;

	unsigned int max_latency; /* FIFO flush period in ms */
	int64_t timestamp, old_timestamp; /* Only used in hw fifo mode. */

	struct iio_mount_matrix orientation;
	u8 /* fifo_mode,*/ watermark;
	/* 3 x 16bit accel data + timestamp */
	s16 buffer[8];
	struct {
		__le16 channels[3];
		s64 ts __aligned(8);
	} scan;
};

static const struct iio_mount_matrix *
kx022a_get_mount_matrix(const struct iio_dev *idev,
		       const struct iio_chan_spec *chan)
{
	struct kx022a_data *data = iio_priv(idev);

	return &data->orientation;
}

enum {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
	AXIS_MAX,
};

static const unsigned long kx022a_scan_masks[] = {
					BIT(AXIS_X) | BIT(AXIS_Y) | BIT(AXIS_Z),
					0};

static const struct iio_chan_spec_ext_info kx022a_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_TYPE, kx022a_get_mount_matrix),
	{ },
};

/* TODO: Check bits/scales/intiaanit. */
#define KX022A_ACCEL_CHAN(axis, index)						\
	{								\
		.type = IIO_ACCEL,					\
		.modified = 1,						\
		.channel2 = IIO_MOD_##axis,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
					BIT(IIO_CHAN_INFO_SAMP_FREQ) /*|	\
					BIT(IIO_CHAN_INFO_OFFSET)*/,	\
		.ext_info = kx022a_ext_info,				\
		.address = KX022_REG_##axis##OUT_L,				\
		.scan_index = index,					\
		.scan_type = {                                          \
			.sign = 's',					\
			.realbits = 16,					\
			.storagebits = 16,				\
			.shift = 0,					\
			.endianness = IIO_BE,				\
		},							\
	}


static const struct iio_chan_spec kx022a_channels[] = {
	KX022A_ACCEL_CHAN(X, 0),
	KX022A_ACCEL_CHAN(Y, 1),
	KX022A_ACCEL_CHAN(Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

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
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("0.78 1.563 3.125 6.25 12.5 25 50 100 200");

static struct attribute *kx022a_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group kx022a_attrs_group = {
	.attrs = kx022a_attributes,
};

/*
 * range is typically +-2g/4g/8g/16g, distributed over the amount of bits.
 * The scale table can be calculated using
 *	(range / 2^bits) * g = (range / 2^bits) * 9.80665 m/s^2
 *	=> KX022A uses 16 bit (HiRes mode - assume the low 8 bits are zeroed
 *	in low-power mode(?) )
 *	=> +/-2G  => 4 / 2^16 * 9,80665 * 10^6 (to scale to micro)
 *	=> +/-2G  - 598.550415
 *	   +/-4G  - 1197.10083
 *	   +/-8G  - 2394.20166
 *	   +/-16G - 4788.40332
 */
static const int kx022a_scale_table[] = {599, 1197, 2394, 4788};

/*
 * ODR table. First value represents the integer portion of frequency (Hz), and
 * the second value is the decimal part. Eg, 0.78 Hz, 1.563 Hz, ...
 */
struct kx022a_tuple {
	int val;
	int val2;
};

static const struct kx022a_tuple kx022a_accel_samp_freq_table[] = {
	{0, 780000}, {1, 563000}, {3, 125000}, {6, 250000}, {12, 500000},
	{25, 0}, {50, 0}, {100, 0}, {200, 0}
};

/*
 * Find index of tuple matching the given values
 */
static int kx022a_find_tuple_index(const struct kx022a_tuple *tuples, int n,
				   int val, int val2)
{
	while (n-- > 0)
		if (val == tuples[n].val && tuples[n].val2 == val2)
			return n;

	return -EINVAL;
}

static int kx022a_reg2scale(unsigned int val)
{
	val &= KX022_MASK_GSEL;
	val >>= KX022A_GSEL_SHIFT;

	return kx022a_scale_table[val];
}

static int kx022a_find_scale_regval(int val)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(kx022a_scale_table); i++)
		if (kx022a_scale_table[i] == val)
			return i << KX022A_GSEL_SHIFT;

	return -EINVAL;
}

#define MIN_ODR_INTERVAL_MS 5
#define MAX_ODR_INTERVAL_MS 1280
#define NUM_SUPPORTED_ODR 9

static int __kx022a_turn_on_unlocked(struct kx022a_data *data)
{
	int ret;

	ret = regmap_set_bits(data->regmap, KX022_REG_CNTL, KX022_MASK_PC1);
	if (ret)
		dev_err(data->dev, "Turn ON fail %d\n", ret);

	return ret;
}

static int __kx022a_turn_off_unlocked(struct kx022a_data *data)
{
	int ret;

	ret = regmap_clear_bits(data->regmap, KX022_REG_CNTL, KX022_MASK_PC1);
	if (ret)
		dev_err(data->dev, "Turn OFF fail %d\n", ret);

	return ret;
}

static int kx022a_turn_off_lock(struct kx022a_data *data)
{
	int ret;

	pr_info("LOCK %s()\n", __func__);
	mutex_lock(&data->mutex);
	ret = __kx022a_turn_off_unlocked(data);;
	if (ret) {
		pr_info("UNLOCK %s()\n", __func__);
		mutex_unlock(&data->mutex);
	}

	return ret;
}

static int kx022a_turn_on_unlock(struct kx022a_data *data)
{
	int ret;

	ret = __kx022a_turn_on_unlocked(data);
	if (ret)
		dev_err(data->dev, "Accelerometer start failed\n");
	pr_info("UNLOCK %s()\n", __func__);
	mutex_unlock(&data->mutex);

	return ret;
}

static int kx022a_write_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int val, int val2, long mask)
{
	struct kx022a_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		pr_info("Sample freq %d %d requested\n", val, val2);
		ret = kx022a_find_tuple_index(&kx022a_accel_samp_freq_table[0],
					      ARRAY_SIZE(kx022a_accel_samp_freq_table),
					      val, val2);
		/* Configure we found valid ODR */
		if (ret >= 0) {
			int odr = ret;
			pr_info("YaY! Found ODR reg val 0x%x\n", ret);
			ret = kx022a_turn_off_lock(data);
			if (ret)
				return ret;
			pr_info("writing REG 0x%x val 0x%x\n", KX022_REG_ODCNTL,
				(int)(odr & KX022_MASK_ODR));
			ret = regmap_update_bits(data->regmap, KX022_REG_ODCNTL,
                				 KX022_MASK_ODR, odr);
			kx022a_turn_on_unlock(data);
		}
		else
			pr_info("Crap. No ODR for samp freq %d %d found\n", val, val2);
		break;
	case IIO_CHAN_INFO_SCALE:
		pr_info("Scale %d %d requested\n", val, val2);
		if (val)
			return -EINVAL;

		ret = kx022a_find_scale_regval(val2);
		/* Configure if we found valid scale */
		if (ret > 0) {
			pr_info("YaY! Found SCALE reg val 0x%x\n", ret);
			ret = kx022a_turn_off_lock(data);
			if (ret)
				return ret;
			ret = regmap_update_bits(data->regmap, KX022_REG_CNTL,
                				 KX022_MASK_GSEL, ret);
			kx022a_turn_on_unlock(data);
		}
		else
			pr_info("Crap. No SCALE matching %d %d found\n", val, val2);
		return ret;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int kx022a_fifo_set_wmi(struct kx022a_data *data)
{
	u8 threshold;

	pr_info("Setting threshold %d\n", data->watermark);
	threshold = data->watermark;
	/* how many samples stored to FIFO before wmi */
//	if (data->max_latency / data->odr_interval_ms > KX022_FIFO_MAX_WMI_TH)
//		threshold = KX022_FIFO_MAX_WMI_TH;
//	else
//		threshold = data->max_latency / data->odr_interval_ms;

	return regmap_update_bits(data->regmap, KX022_REG_BUF_CNTL1,
				 KX022_MASK_WM_TH, threshold);
}

#define KX022_FIFO_SIZE_BYTES 256
/* 3 axis, 2 bytes of data for each of the axis */
#define KX022_FIFO_SAMPLES_SIZE_BYTES 6
#define KX022_FIFO_MAX_SAMPLES (KX022_FIFO_SIZE_BYTES/KX022_FIFO_SAMPLES_SIZE_BYTES)

static int kx022a_fifo_report_data(struct kx022a_data *data, void *buffer,
				    int samples)
{
	int ret, fifo_bytes = samples * 3 * 2;

	pr_info("Read %u samples, %u bytes from FIFO\n", samples, fifo_bytes);
	ret = regmap_noinc_read(data->regmap, KX022_REG_BUF_READ,
			       buffer, fifo_bytes);
	if (ret < 0)
		dev_err(data->dev, "FIFO read failed %d\n", ret);

	return ret;
}

static int kx022a_get_axis(struct kx022a_data *data,
			   struct iio_chan_spec const *chan,
			   int *val)
{
	u16 raw_val;
	int ret;

	ret = regmap_bulk_read(data->regmap, chan->address, &raw_val,
			       sizeof(raw_val));
	if (ret)
		return ret;
	*val = raw_val;

	return IIO_VAL_INT;
}
//#define KX022A_ZERO_G_OFFSET -32768§

static int kx022a_read_raw(struct iio_dev *idev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	int ret = -EINVAL;
	struct kx022a_data *data = iio_priv(idev);
	unsigned int regval;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (iio_buffer_enabled(idev)) {
			ret = -EBUSY;
			goto error_ret;
		}

		ret = kx022a_get_axis(data, chan, val);
		break;
#if 0
	case IIO_CHAN_INFO_OFFSET:
		/* This has a bias of -2048 */
		*val = KXSD9_ZERO_G_OFFSET;
		ret = IIO_VAL_INT;
		break;
#endif
	case IIO_CHAN_INFO_SAMP_FREQ:
	{
		const struct kx022a_tuple *freq;

		ret = regmap_read(data->regmap, KX022_REG_ODCNTL, &regval);
		if (ret)
			goto error_ret;

		if ((regval & KX022_MASK_ODR) >
		    ARRAY_SIZE(kx022a_accel_samp_freq_table)) {
			dev_err(data->dev, "Invalid ODR\n");
			return -EINVAL;
		}
		freq = &kx022a_accel_samp_freq_table[regval & KX022_MASK_ODR];

		*val = freq->val;
		*val2 = freq->val2;

		break;
	}
	case IIO_CHAN_INFO_SCALE:
		ret = regmap_read(data->regmap, KX022_REG_CNTL, &regval);
		if (ret < 0)
			goto error_ret;

		*val = kx022a_reg2scale(regval);

		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	}

error_ret:

	return ret;
};

static int kx022a_validate_trigger(struct iio_dev *indio_dev,
				   struct iio_trigger *trig)
{
	struct kx022a_data *data = iio_priv(indio_dev);
	int i;

	for (i = 0; i < KX022A_TRIGGERS; i++) {
		if (data->triggers[i].indio_trig == trig)
			return 0;
	}

	return -EINVAL;
}

static int kx022a_set_watermark(struct iio_dev *indio_dev, unsigned val)
{
	struct kx022a_data *data = iio_priv(indio_dev);

	/* Fifo len as num of samples? */
	if (val > KX022A_FIFO_LENGTH)
		val = KX022A_FIFO_LENGTH;

	pr_info("Setting (caching) watermark to %d\n", val);

	pr_info("LOCK %s()\n", __func__);
	mutex_lock(&data->mutex);
	data->watermark = val;
	pr_info("UNLOCK %s()\n", __func__);
	mutex_unlock(&data->mutex);

	return 0;
}

static ssize_t kx022a_get_fifo_state(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
        struct iio_dev *indio_dev = dev_to_iio_dev(dev);
        struct kx022a_data *data = iio_priv(indio_dev);
        bool state;

	pr_info("LOCK %s()\n", __func__);
        mutex_lock(&data->mutex);
        state = data->state;
	pr_info("UNLOCK %s()\n", __func__);
        mutex_unlock(&data->mutex);

        return sprintf(buf, "%d\n", state);
}

static ssize_t kx022a_get_fifo_watermark(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct kx022a_data *data = iio_priv(indio_dev);
	int wm;

	pr_info("LOCK %s()\n", __func__);
	mutex_lock(&data->mutex);
	wm = data->watermark;
	pr_info("UNLOCK %s()\n", __func__);
	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", wm);
}

static IIO_CONST_ATTR(matti, "YES - MATTIMATTIMATTIMATTTI");
static IIO_CONST_ATTR(vaittinen, "YES - VAITTINENVAITTINENVAITTINENVAITTINEN");
static IIO_CONST_ATTR(hwfifo_watermark_min, "1");
static IIO_CONST_ATTR(hwfifo_watermark_max,
		      __stringify(KX022A_FIFO_LENGTH));
static IIO_DEVICE_ATTR(hwfifo_enabled, S_IRUGO,
                       kx022a_get_fifo_state, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, S_IRUGO,
		       kx022a_get_fifo_watermark, NULL, 0);

static const struct attribute *kx022a_fifo_attributes[] = {
	&iio_const_attr_matti.dev_attr.attr,
	&iio_const_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_const_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_const_attr_vaittinen.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	NULL,
};

static int __kx022a_fifo_flush(struct iio_dev *indio_dev,
				     unsigned samples, bool irq)
{
	struct kx022a_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	int ret, i;
	int count, fifo_bytes;
	u16 buffer[KX022A_FIFO_LENGTH * 3];
	int64_t tstamp;
	uint64_t sample_period;

	ret = regmap_read(data->regmap, KX022_REG_BUF_STATUS_1, &fifo_bytes);
	if (ret < 0) {
		dev_err(dev, "Error reading buffer status\n");
		return ret;
	}

	/* Let's not owerflow if we for some reason get bogus value from i2c */
	if (fifo_bytes > ARRAY_SIZE(buffer)) {
		dev_warn(data->dev, "Bad amount of data\n");
		fifo_bytes = ARRAY_SIZE(buffer);
	}

	pr_info("Flushing %u bytes from fifo\n", fifo_bytes);

	if (fifo_bytes % 3 * 2)
		dev_err(data->dev, "Bad FIFO alignment. Data may be corrupt\n");

	count = fifo_bytes / (3 * 2);
	if (!count)
		return 0;

	/*
	 * If we getting called from IRQ handler we know the stored timestamp is
	 * fairly accurate for the last stored sample. Otherwise, if we are
	 * called as a result of a read operation from userspace and hence
	 * before the watermark interrupt was triggered, take a timestamp
	 * now. We can fall anywhere in between two samples so the error in this
	 * case is at most one sample period.
	 */
	if (!irq) {
		data->old_timestamp = data->timestamp;
		data->timestamp = iio_get_time_ns(indio_dev);
	}

	/*
	 * Approximate timestamps for each of the sample based on the sampling
	 * frequency, timestamp for last sample and number of samples.
	 *
	 * Note that we can't use the current bandwidth settings to compute the
	 * sample period because the sample rate varies with the device
	 * (e.g. between 31.70ms to 32.20ms for a bandwidth of 15.63HZ). That
	 * small variation adds when we store a large number of samples and
	 * creates significant jitter between the last and first samples in
	 * different batches (e.g. 32ms vs 21ms).
	 *
	 * To avoid this issue we compute the actual sample period ourselves
	 * based on the timestamp delta between the last two flush operations.
	 */
	sample_period = (data->timestamp - data->old_timestamp);
	do_div(sample_period, count);
	tstamp = data->timestamp - (count - 1) * sample_period;

	if (samples && count > samples)
		count = samples;

	ret = kx022a_fifo_report_data(data, buffer, count);
	if (ret)
		return ret;

	/*
	 * Ideally we want the IIO core to handle the demux when running in fifo
	 * mode but not when running in triggered buffer mode. Unfortunately
	 * this does not seem to be possible, so stick with driver demux for
	 * now.
	 */
	for (i = 0; i < count; i++) {
		int j, bit;

		j = 0;
		for_each_set_bit(bit, indio_dev->active_scan_mask,
				 indio_dev->masklength)
			memcpy(&data->scan.channels[j++], &buffer[i * 3 + bit],
			       sizeof(data->scan.channels[0]));

		iio_push_to_buffers_with_timestamp(indio_dev, &data->scan,
						   tstamp);

		tstamp += sample_period;
	}

	return count;
}

static int kx022a_fifo_flush(struct iio_dev *indio_dev, unsigned samples)
{
	struct kx022a_data *data = iio_priv(indio_dev);
	int ret;

	pr_info("LOCK %s()\n", __func__);
	mutex_lock(&data->mutex);
	ret = __kx022a_fifo_flush(indio_dev, samples, false);
	pr_info("UNLOCK %s()\n", __func__);
	mutex_unlock(&data->mutex);

	return ret;
}

static const struct iio_info kx022a_info = {
	.read_raw = &kx022a_read_raw,
	.write_raw = &kx022a_write_raw,
	.attrs = &kx022a_attrs_group,

	.validate_trigger	= kx022a_validate_trigger,
	.hwfifo_set_watermark	= kx022a_set_watermark,
	.hwfifo_flush_to_buffer	= kx022a_fifo_flush,
};

static int kx022a_set_drdy_irq(struct kx022a_data *data, bool en)
{
	if (en)
		return regmap_set_bits(data->regmap, KX022_REG_CNTL,
				       KX022_MASK_DRDY);

	return regmap_clear_bits(data->regmap, KX022_REG_CNTL, KX022_MASK_DRDY);
}

static int kx022a_prepare_irq_pin(struct kx022a_data *data)
{
	/* Enable IRQ1 pin. Set polarity to active low */
	int mask = KX022_MASK_IEN1 | KX022_MASK_IPOL1 |
		   KX022_MASK_ITYP;
	int val = KX022_MASK_IEN1 | KX022_IPOL_LOW |
		  KX022_ITYP_LEVEL;
	int ret;

	ret = regmap_update_bits(data->regmap, KX022_REG_INC1, mask, val);
	if (ret)
		return ret;

	mask = KX022_MASK_INS2_DRDY | KX122_MASK_INS2_WMI;

	return regmap_set_bits(data->regmap, KX022_REG_INC4, mask);
}

static int kx022a_fifo_disable(struct kx022a_data *data)
{
	int ret = 0;

	/* PC1 to 0 */
	ret = kx022a_turn_off_lock(data);
	if (ret)
		return ret;

	ret = regmap_clear_bits(data->regmap, KX022_REG_INC4,
			        KX022_MASK_WMI1);
	if (ret)
		goto unlock_out;

	/* disable buffer */
	ret = regmap_clear_bits(data->regmap, KX022_REG_BUF_CNTL2,
			      KX022_MASK_BUF_EN);
	if (ret)
		goto unlock_out;

	/* timestamp when fifo started */
	data->state &= (~KX022_STATE_FIFO);

	return kx022a_turn_on_unlock(data);

unlock_out:
	pr_info("UNLOCK %s()\n", __func__);
	mutex_unlock(&data->mutex);

	return ret;
}


static int kx022a_buffer_predisable(struct iio_dev *indio_dev)
{
	struct kx022a_data *data = iio_priv(indio_dev);

	pr_info("kx022a_buffer_predisable() called\n");

	if (iio_device_get_current_mode(indio_dev) == INDIO_BUFFER_TRIGGERED) {
		pr_info("Triggered mode => skip fifo_disable\n");
		return 0;
	}

	return kx022a_fifo_disable(data);
}

static int kx022a_fifo_enable(struct kx022a_data *data)
{
	int ret = 0;

	ret = kx022a_turn_off_lock(data);
	if (ret)
		return ret;

	/* Write WMI to HW */
	ret = kx022a_fifo_set_wmi(data);
	if (ret)
		goto unlock_out;

	/* Enable buffer */
	ret = regmap_set_bits(data->regmap, KX022_REG_BUF_CNTL2,
			      KX022_MASK_BUF_EN);
	if (ret)
		goto unlock_out;

	data->state |= KX022_STATE_FIFO;
	ret = regmap_set_bits(data->regmap, KX022_REG_INC4,
			      KX022_MASK_WMI1);
	if (ret)
		goto unlock_out;

	return kx022a_turn_on_unlock(data);

unlock_out:
	pr_info("UNLOCK %s()\n", __func__);
	mutex_unlock(&data->mutex);

	return ret;
}

static int kx022a_buffer_postenable(struct iio_dev *indio_dev)
{
	struct kx022a_data *data = iio_priv(indio_dev);

	pr_info("kx022a_buffer_postenable() called\n");

	/*
	 * If we use triggers, then the IRQs should be handled by trigger
	 * enable and buffer is not used but we just add results to buffer
	 * when data-ready triggers.
	 */
	if (iio_device_get_current_mode(indio_dev) == INDIO_BUFFER_TRIGGERED) {
		pr_info("Triggered mode => skip fifo_enable\n");
		return 0;
	}

	return kx022a_fifo_enable(data);
}

static const struct iio_buffer_setup_ops kx022a_buffer_ops = {
	.postenable = kx022a_buffer_postenable,
	.predisable = kx022a_buffer_predisable,
};

static irqreturn_t kx022a_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct kx022a_data *data = iio_priv(indio_dev);
	int ret;
	static int tmp = 0;

	if (!(tmp % 1000))
		pr_info("Reading sample %d ts %lld\n", tmp + 1, (long long)pf->timestamp);

	tmp++;

	if (in_hardirq()) {
		pr_info("Oh No. Trigger handler called in hardirq - context\n");
		return IRQ_NONE;
	}
	pr_info("LOCK %s()\n", __func__);
	mutex_lock(&data->mutex);
	ret = regmap_bulk_read(data->regmap, KX022_REG_XOUT_L, data->buffer, 6);
	pr_info("UNLOCK %s()\n", __func__);
	mutex_unlock(&data->mutex);
	if (ret < 0)
		goto err_read;

	iio_push_to_buffers_with_timestamp(indio_dev, data->buffer,
					   pf->timestamp);
err_read:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

/* Get timestamps and wake the thread if we need to read data */
static irqreturn_t kx022a_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct kx022a_data *data = iio_priv(indio_dev);
	bool ack = false;
	int i;
	static int foo 	 = 0;

	data->old_timestamp = data->timestamp;
	data->timestamp = iio_get_time_ns(indio_dev);

	if (!(foo % 1000))
		pr_info("IRQ handler invoked %d\n", foo + 1);

	foo++;

	for (i = 0; i < KX022A_TRIGGERS; i++) {
		if (data->triggers[i].enabled) {
			pr_info("triggering IIO trigger for drdy\n");
			iio_trigger_poll(data->triggers[i].indio_trig);
			ack = true;
			break;
		}
	}

	if (data->state & KX022_STATE_FIFO)
		return IRQ_WAKE_THREAD;

	if (ack) {
		/*
		 * The IRQ is not acked until data is read. We need to disable
		 * the IRQ in order to schedule the trigger thread. Enabling
		 * is done in reenable.
		 *
		 * It would be possible to set the IRQ to 50uS pulse. If we are
		 * losing data due to the disabled IRQ we can evaluate the
		 * option of using edge triggered IRQs with the pulse mode.
		 */
		disable_irq_nosync(irq);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/*
 * Read read the data from the fifo and fill IIO buffers */
static irqreturn_t kx022a_irq_thread_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct kx022a_data *data = iio_priv(indio_dev);
	bool ack = false;
	int ret;

	pr_info("LOCK %s()\n", __func__);
	mutex_lock(&data->mutex);

	if (data->state & KX022_STATE_FIFO) {
		ret = __kx022a_fifo_flush(indio_dev,
					  KX022A_FIFO_LENGTH, true);
		if (ret > 0)
			ack = true;
	}
/*
 * 	WMI and data-ready IRQs are acked when results are read. If we add
 * 	TILT/WAKE or other IRQs - then we may need to implement the acking
 * 	(which is racy).
*/
	if (ack)
		ret = IRQ_HANDLED;
	else
		ret = IRQ_NONE;

	pr_info("UNLOCK %s()\n", __func__);
	mutex_unlock(&data->mutex);

	return ret;
}

static int kx022a_trigger_set_state(struct iio_trigger *trig,
				    bool state)
{
	struct kx022a_trigger *t = iio_trigger_get_drvdata(trig);
	struct kx022a_data *data = t->data;
	int ret = 0;

	pr_info("LOCK %s()\n", __func__);
	mutex_lock(&data->mutex);

	if (t->enabled == state)
		goto unlock_out;

	/* TODO: Take data-ready in use here */
	pr_info("YaY! I now %s data-ready IRQ/trigger\n", (state)? "enable":"disable");

	ret = __kx022a_turn_off_unlocked(data);
	if (ret)
		goto unlock_out;

	t->enabled = state;
	ret = kx022a_set_drdy_irq(data, state);
	if (ret)
		goto unlock_out;

	pr_info("DRDY enabled - turning on\n");

	ret = __kx022a_turn_on_unlocked(data);

unlock_out:
	pr_info("UNLOCK %s()\n", __func__);
	mutex_unlock(&data->mutex);

	return ret;
}
static void kx022a_trig_reen(struct iio_trigger *trig)
{
	struct kx022a_trigger *t = iio_trigger_get_drvdata(trig);
	struct kx022a_data *data = t->data;

	pr_info("Re-enabling DRDY\n");
	enable_irq(data->irq);
}

static const struct iio_trigger_ops kx022a_trigger_ops = {
	.set_trigger_state = kx022a_trigger_set_state,
	.reenable = kx022a_trig_reen,
};

static int kx022a_chip_init(struct kx022a_data *data)
{
	u16 buffer[KX022A_FIFO_LENGTH * 3];
	int ret, fifo_bytes, dummy;

	/*
	 * Disable IRQs because if the IRQs are left on (for example by
	 * a shutdown which did not deactivate the accelerometer) we do
	 * most probably end up flooding the system with unhandled IRQs
	 * and get the line disabled from SOC side.
	 */
	ret = regmap_write(data->regmap, KX022_REG_INC4, 0);
	if (ret) {
		dev_err(data->dev, "Failed to init IRQ states\n");
		return ret;
	}

	ret = kx022a_set_drdy_irq(data, false);
	if (ret) {
		dev_err(data->dev, "Failed to init DRDY\n");
		return ret;
	}

	/* Clear any pending IRQs */
	ret = regmap_read(data->regmap, KX022_REG_INT_REL, &dummy);
	if (ret) {
		dev_err(data->dev, "Failed to ACK IRQs\n");
		return ret;
	}
	/* set data res 16bit */
	ret = regmap_set_bits(data->regmap, KX022_REG_BUF_CNTL2,
			      KX022_MASK_BRES);
	if (ret) {
		dev_err(data->dev, "Failed to set data resolution\n");
		return ret;
	}

	ret = kx022a_prepare_irq_pin(data);
	if (ret) {
		dev_err(data->dev, "Failed to configure IRQ pin\n");
		return ret;
	}

	/* disable buffer */
	ret = regmap_clear_bits(data->regmap, KX022_REG_BUF_CNTL2,
			      KX022_MASK_BUF_EN);
	if (ret)
		return ret;

	/* Clean the old data from FIFO - this should also clear the WMI/BFI */
	ret = regmap_read(data->regmap, KX022_REG_BUF_STATUS_1, &fifo_bytes);
	if (ret < 0)
		return ret;

	/* Let's not owerflow if we for some reason get bogus value from i2c */
	if (fifo_bytes > ARRAY_SIZE(buffer)) {
		dev_warn(data->dev, "Bad amount of data\n");
		fifo_bytes = ARRAY_SIZE(buffer);
	}

	if (fifo_bytes)
		ret = regmap_noinc_read(data->regmap, KX022_REG_BUF_READ,
				        buffer, fifo_bytes);

	return ret;
}

int kx022a_probe_internal(struct device *dev, int irq)
{
	struct regmap *regmap;
	struct kx022a_data *data;
	unsigned int chip_id;
	struct iio_dev *idev;
	int ret, i;
	static const char *regulator_names[] = {"io_vdd", "vdd"};

	pr_info("MVA mod 1\n");

	if (WARN_ON(!dev))
		return -ENODEV;

	regmap = dev_get_regmap(dev, NULL);
	if (!regmap) {
		dev_err(dev, "no regmap\n");

		return -EINVAL;
	}

	idev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!idev)
		return -ENOMEM;

	data = iio_priv(idev);

	/*
	 * VDD   is the analog and digital domain voltage supply
	 * IO_VDD is the digital I/O voltage supply
	 */
	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(regulator_names),
					     regulator_names);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to enable regulator\n");

	ret = regmap_read(regmap, KX022_REG_WHO, &chip_id);
	if (ret) {
		dev_err(dev, "Failed to access sensor\n");

		return ret;
	}

	if (chip_id != KX022_ID) {
		dev_err(dev, "unsupported device 0x%x\n", chip_id);

		return -EINVAL;
	}

	data->regmap = regmap;
	data->dev = dev;
	data->irq = irq;
	mutex_init(&data->mutex);

	idev->channels = kx022a_channels;
	idev->num_channels = ARRAY_SIZE(kx022a_channels);
	idev->name = "kx022-accel";
	idev->info = &kx022a_info;
	idev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	/* TODO: Check the scan masks */
	idev->available_scan_masks = kx022a_scan_masks;

	/* Read the mounting matrix, if present */
	ret = iio_read_mount_matrix(dev, &data->orientation);
	if (ret)
		return ret;

	/* The sensor must be turned off for configuration */
	ret = kx022a_turn_off_lock(data);
	if (ret)
		return ret;

	ret = kx022a_chip_init(data);
	if (ret)
		return ret;

	ret = kx022a_turn_on_unlock(data);
	if (ret)
		return ret;

	udelay(100);

	ret = iio_triggered_buffer_setup_ext(idev,
					     &iio_pollfunc_store_time,
					     kx022a_trigger_handler,
					     IIO_BUFFER_DIRECTION_IN,
					     &kx022a_buffer_ops,
					     kx022a_fifo_attributes);

	if (ret)
		return dev_err_probe(data->dev, ret, "iio_triggered_buffer_setup_ext FAIL %d\n", ret);

/* TODO: Clean trigger setup. Remove arry if there is only 1 trigger. Call
 * iio_trigger_unregister() upon failure */
	for (i = 0; i < KX022A_TRIGGERS; i++) {
		struct kx022a_trigger *t = &data->triggers[i];

		t->indio_trig = devm_iio_trigger_alloc(dev,
							kx022a_triggers[i].name,
							idev->name,
							iio_device_id(idev));
		if (!t->indio_trig)
			return -ENOMEM;

		t->indio_trig->ops = &kx022a_trigger_ops;
		t->data = data;
		iio_trigger_set_drvdata(t->indio_trig, t);

		ret = iio_trigger_register(t->indio_trig);
		if (ret)
			return dev_err_probe(data->dev, ret, "Trigger %d reg failed\n", i);
	}
	ret = devm_iio_device_register(data->dev, idev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Unable to register iio device\n");

	ret = devm_request_threaded_irq(data->dev, irq, kx022a_irq_handler,
					 &kx022a_irq_thread_handler,
					 IRQF_ONESHOT, "kx022", idev);
	if (ret)
		return dev_err_probe(data->dev, ret, "Could not request IRQ\n");

	return ret;
}
EXPORT_SYMBOL_GPL(kx022a_probe_internal);

MODULE_DESCRIPTION("ROHM/Kionix KX022 accelerometer driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");
