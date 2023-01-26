// SPDX-License-Identifier: GPL-2.0-only
/*
 * BU27008 ROHM Colour Sensor
 *
 * Copyright (c) 2023, ROHM Semiconductor.
 */

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <linux/iio/iio.h>
#include <linux/iio/iio-gts-helper.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define BU27008_REG_SYSTEM_CONTROL	0x40
#define BU27008_MASK_SW_RESET		BIT(7)
#define BU27008_MASK_PART_ID		GENMASK(5, 0)
#define BU27008_ID			0x1a
#define BU27008_REG_MODE_CONTROL1	0x41
#define BU27008_MASK_MEAS_MODE		GENMASK(2, 0)
#define BU27008_MASK_CHAN_SEL		GENMASK(3, 2)

#define BU27008_BLUE2_CLEAR3		0x0
#define BU27008_CLEAR2_IR3		0x1
#define BU27008_BLUE2_IR3		0x2

#define BU27008_REG_MODE_CONTROL2	0x42
#define BU27008_MASK_RGBC_GAIN		GENMASK(7, 3)
#define BU27008_SHIFT_RGBC_GAIN		3
#define BU27008_MASK_IR_GAIN_HI		GENMASK(7, 6)
#define BU27008_MASK_IR_GAIN_LO		GENMASK(2, 0)
#define BU27008_SHIFT_IR_GAIN		3
#define BU27008_MASK_IR_GAIN		(BU27008_MASK_IR_GAIN_HI | \
					 BU27008_MASK_IR_GAIN_LO)

#define BU27008_REG_MODE_CONTROL3	0x43
#define BU27008_MASK_VALID		BIT(7)
#define BU27008_MASK_INT_EN		BIT(1)
#define BU27008_MASK_MEAS_EN		BIT(0)

#define BU27008_REG_DATA0_LO		0x50
#define BU27008_REG_DATA1_LO		0x52
#define BU27008_REG_DATA2_LO		0x54
#define BU27008_REG_DATA3_LO		0x56
#define BU27008_REG_DATA3_HI		0x57
#define BU27008_REG_MANUFACTURER_ID	0x92
#define BU27008_REG_MAX BU27008_REG_MANUFACTURER_ID

/*
 * Max amplification is (HWGAIN * MAX integration-time multiplier) 1024 * 8
 * = 8192. With NANO scale we get rid of accuracy loss when we start with the
 * scale 16.0 for HWGAIN1, INT-TIME 55 mS. This way the nano scale for MAX
 * total gain 8192 will be 1953125
 */
enum {
	BU27008_RED,	/* Always data0 */
	BU27008_GREEN,	/* Always data1 */
	BU27008_BLUE,	/* data2, configurable (blue / clear) */
	BU27008_CLEAR,	/* data2 or data3 */
	BU27008_IR,	/* data3 */
	BU27008_NUM_CHANS
};

enum {
	BU27008_DATA0, /* Always RED */
	BU27008_DATA1, /* Always GREEN */
	BU27008_DATA2, /* Blue or Clear */
	BU27008_DATA3, /* IR or Clear */
	BU27008_NUM_HW_CHANS
};

/* We can always measure red and green at same time */
#define ALWAYS_SCANNABLE (BIT(BU27008_RED) | BIT(BU27008_GREEN))

#define BU27008_CHAN_DATA_SIZE		2 /* Each channel has 16bits of data */
#define BU27008_BUF_DATA_SIZE (BU27008_NUM_CHANS * BU27008_CHAN_DATA_SIZE)
#define BU27008_HW_DATA_SIZE (BU27008_NUM_HW_CHANS * BU27008_CHAN_DATA_SIZE)
#define NUM_U16_IN_TSTAMP (sizeof(s64) / sizeof(u16))

static const unsigned long bu27008_scan_masks[] = {
	ALWAYS_SCANNABLE | BIT(BU27008_CLEAR) | BIT(BU27008_IR),
	ALWAYS_SCANNABLE | BIT(BU27008_CLEAR) | BIT(BU27008_BLUE),
	ALWAYS_SCANNABLE | BIT(BU27008_BLUE) | BIT(BU27008_IR),
	0
};

/*
 * Available scales with gain 1x - 1024x, timings 55, 100, 200, 400 mS
 * Time impacts to gain: 1x, 2x, 4x, 8x.
 *
 * => Max total gain is HWGAIN * gain by integration time (8 * 1024) = 8192
 *
 * Using NANO precision for scale we must use scale 16x corresponding gain 1x
 * to avoid precision loss.
 */
#define BU27008_SCALE_1X 16

/* See the data sheet for the "Gain Setting" table */
#define BU27008_GSEL_1X		0x00
#define BU27008_GSEL_4X		0x08
#define BU27008_GSEL_8X		0x09
#define BU27008_GSEL_16X	0x0a
#define BU27008_GSEL_32X	0x0b
#define BU27008_GSEL_64X	0x0c
#define BU27008_GSEL_256X	0x18
#define BU27008_GSEL_512X	0x19
#define BU27008_GSEL_1024X	0x1a

static const struct iio_gain_sel_pair bu27008_gains[] = {
	GAIN_SCALE_GAIN(1, BU27008_GSEL_1X),
	GAIN_SCALE_GAIN(4, BU27008_GSEL_4X),
	GAIN_SCALE_GAIN(8, BU27008_GSEL_8X),
	GAIN_SCALE_GAIN(16, BU27008_GSEL_16X),
	GAIN_SCALE_GAIN(32, BU27008_GSEL_32X),
	GAIN_SCALE_GAIN(64, BU27008_GSEL_64X),
	GAIN_SCALE_GAIN(256, BU27008_GSEL_256X),
	GAIN_SCALE_GAIN(512, BU27008_GSEL_512X),
	GAIN_SCALE_GAIN(1024, BU27008_GSEL_1024X),
};

static const struct iio_gain_sel_pair bu27008_gains_ir[] = {
	GAIN_SCALE_GAIN(2, BU27008_GSEL_1X),
	GAIN_SCALE_GAIN(4, BU27008_GSEL_4X),
	GAIN_SCALE_GAIN(8, BU27008_GSEL_8X),
	GAIN_SCALE_GAIN(16, BU27008_GSEL_16X),
	GAIN_SCALE_GAIN(32, BU27008_GSEL_32X),
	GAIN_SCALE_GAIN(64, BU27008_GSEL_64X),
	GAIN_SCALE_GAIN(256, BU27008_GSEL_256X),
	GAIN_SCALE_GAIN(512, BU27008_GSEL_512X),
	GAIN_SCALE_GAIN(1024, BU27008_GSEL_1024X),
};



#define BU27008_MEAS_MODE_100MS		0x00
#define BU27008_MEAS_MODE_55MS		0x01
#define BU27008_MEAS_MODE_200MS		0x02
#define BU27008_MEAS_MODE_400MS		0x04

static const struct iio_itime_sel_mul bu27008_itimes[] = {
        GAIN_SCALE_ITIME_US(400000, BU27008_MEAS_MODE_400MS, 8),
        GAIN_SCALE_ITIME_US(200000, BU27008_MEAS_MODE_200MS, 4),
        GAIN_SCALE_ITIME_US(100000, BU27008_MEAS_MODE_100MS, 2),
        GAIN_SCALE_ITIME_US(55000, BU27008_MEAS_MODE_55MS, 1),
};

/*
 * All the RGBC channels share the same gain.
 * IR gain can be fine-tuned from the gain set for the RGBC by 2 bits
 * but this would yield quite complex gain setting. Especially since not all
 * bit compinations are supported. And in any case setting GAIN for RGBC will
 * always also change the IR-gain.
 *
 * This is something that should be considered by one who implements the
 * support for the IR channel.
 */
#define BU27008_CHAN(color, data, separate_avail)				\
{										\
	.type = IIO_INTENSITY,							\
	.modified = 1,								\
	.channel2 = IIO_MOD_LIGHT_##color,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |				\
			      BIT(IIO_CHAN_INFO_SCALE),				\
	.info_mask_separate_available = (separate_avail),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_INT_TIME),			\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_INT_TIME),	\
	.address = BU27008_REG_##data##_LO,					\
	.scan_index = BU27008_##color,						\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = 16,							\
		.storagebits = 16,						\
		.endianness = IIO_LE,						\
	},									\
}

/* For raw reads we always configure DATA3 for CLEAR */
static const struct iio_chan_spec bu27008_channels[] = {
	BU27008_CHAN(RED, DATA0, BIT(IIO_CHAN_INFO_SCALE)),
	BU27008_CHAN(GREEN, DATA1, BIT(IIO_CHAN_INFO_SCALE)),
	BU27008_CHAN(BLUE, DATA2, BIT(IIO_CHAN_INFO_SCALE)),
	BU27008_CHAN(CLEAR, DATA2, BIT(IIO_CHAN_INFO_SCALE)),
	BU27008_CHAN(IR, DATA3, 0),	/* We don't allow settin scale for IR */
	IIO_CHAN_SOFT_TIMESTAMP(BU27008_NUM_CHANS),
};

struct bu27008_data {
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct device *dev;
	struct iio_gts gts;
	struct iio_gts gts_ir;
	int64_t timestamp, old_timestamp;

	int irq;

	/*
	 * Prevent changing gain/time config when scale is read/written.
	 * Prevent changing gain/time when raw data is read.
	 */
	struct mutex mutex;
	bool trigger_enabled;

	__le16 buffer[BU27008_NUM_CHANS];

	struct {
		__le16 channels[BU27008_NUM_CHANS];
		s64 ts __aligned(8);
	} scan;

};

static const struct regmap_range bu27008_volatile_ranges[] = {
	{
		.range_min = BU27008_REG_SYSTEM_CONTROL,	/* SWRESET */
		.range_max = BU27008_REG_SYSTEM_CONTROL,
	}, {
		.range_min = BU27008_REG_MODE_CONTROL3,		/* VALID */
		.range_max = BU27008_REG_MODE_CONTROL3,
	}, {
		.range_min = BU27008_REG_DATA0_LO,		/* DATA */
		.range_max = BU27008_REG_DATA3_HI,
	},
};

static const struct regmap_access_table bu27008_volatile_regs = {
	.yes_ranges = &bu27008_volatile_ranges[0],
	.n_yes_ranges = ARRAY_SIZE(bu27008_volatile_ranges),
};

static const struct regmap_range bu27008_read_only_ranges[] = {
	{
		.range_min = BU27008_REG_DATA0_LO,
		.range_max = BU27008_REG_DATA3_HI,
	}, {
		.range_min = BU27008_REG_MANUFACTURER_ID,
		.range_max = BU27008_REG_MANUFACTURER_ID,
	}
};

static const struct regmap_access_table bu27008_ro_regs = {
	.no_ranges = &bu27008_read_only_ranges[0],
	.n_no_ranges = ARRAY_SIZE(bu27008_read_only_ranges),
};

static const struct regmap_config bu27008_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = BU27008_REG_MAX,
	.cache_type = REGCACHE_RBTREE,
	.volatile_table = &bu27008_volatile_regs,
	.wr_table = &bu27008_ro_regs,
};



#define BU27008_MAX_VALID_RESULT_WAIT_US	50000
#define BU27008_VALID_RESULT_WAIT_QUANTA_US	1000

static int bu27008_chan_read_data(struct bu27008_data *data, int reg, int *val)
{
	int ret, valid;
	__le16 tmp;

	ret = regmap_read_poll_timeout(data->regmap, BU27008_REG_MODE_CONTROL3,
				       valid, (valid & BU27008_MASK_VALID),
				       BU27008_VALID_RESULT_WAIT_QUANTA_US,
				       BU27008_MAX_VALID_RESULT_WAIT_US);
	if (ret)
		return ret;

	ret = regmap_bulk_read(data->regmap, reg, &tmp, sizeof(tmp));
	if (ret)
		dev_err(data->dev, "Reading channel data failed\n");

	*val = le16_to_cpu(tmp);

	return ret;
}

static int bu27008_get_gain(struct bu27008_data *data, struct iio_gts *gts, int *gain)
{
	int ret, sel;

	ret = regmap_read(data->regmap, BU27008_REG_MODE_CONTROL2, &sel);
	if (ret)
		return ret;

	sel = FIELD_GET(BU27008_MASK_RGBC_GAIN, sel);

	ret = iio_gts_find_gain_by_sel(gts, sel);

	if (ret < 0) {
		dev_err(data->dev, "unknown gain value 0x%x\n", sel);

		return ret;
	}

	*gain = ret;

	return 0;
}

static int bu27008_write_gain_sel(struct bu27008_data *data, int sel)
{
	int regval;

	regval = FIELD_PREP(BU27008_MASK_RGBC_GAIN, sel);

	/*
	 * We do always set also the LOW bits of IR-gain because othervice we
	 * would risk resulting an invalid GAIN register value.
	 */
	regval |= FIELD_PREP(BU27008_MASK_IR_GAIN_LO, sel);

	return regmap_update_bits(data->regmap, BU27008_REG_MODE_CONTROL2,
				  BU27008_MASK_RGBC_GAIN, regval);
}

static int bu27008_set_gain(struct bu27008_data *data, int gain)
{
	int ret;

	ret = iio_gts_find_sel_by_gain(&data->gts, gain);
	if (ret < 0)
		return ret;

	return bu27008_write_gain_sel(data, ret);
}

static int bu27008_get_int_time_sel(struct bu27008_data *data, int *sel)
{
	int ret, val;

	ret = regmap_read(data->regmap, BU27008_REG_MODE_CONTROL1, &val);
	*sel = FIELD_GET(BU27008_MASK_MEAS_MODE, val);

	return ret;
}

static int bu27008_set_int_time_sel(struct bu27008_data *data, int sel)
{
		return regmap_update_bits(data->regmap,
					  BU27008_REG_MODE_CONTROL1,
					  BU27008_MASK_MEAS_MODE, sel);
}

static int bu27008_get_int_time(struct bu27008_data *data)
{
	int ret, sel;

	ret = bu27008_get_int_time_sel(data, &sel);
	if (ret)
		return ret;

	return iio_gts_find_int_time_by_sel(&data->gts,
					    sel & BU27008_MASK_MEAS_MODE);
}

static int _bu27008_get_scale(struct bu27008_data *data, bool ir, int *val,
			      int *val2)
{
	struct iio_gts *gts;
	int gain, ret;

	if (ir)
		gts = &data->gts_ir;
	else
		gts = &data->gts;

	ret = bu27008_get_gain(data, gts, &gain);
	if (ret)
		return ret;

	ret = bu27008_get_int_time(data);
	if (ret < 0)
		return ret;

	return iio_gts_get_scale(gts, gain, ret, val, val2);
}

static int bu27008_get_scale(struct bu27008_data *data, bool ir, int *val, int *val2)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = _bu27008_get_scale(data, ir, val, val2);
	mutex_unlock(&data->mutex);

	return ret;
}

static int bu27008_set_int_time(struct bu27008_data *data, int time)
{
	int ret;

	ret = iio_gts_find_sel_by_int_time(&data->gts, time);
	if (ret < 0)
		return ret;

	return regmap_update_bits(data->regmap, BU27008_REG_MODE_CONTROL1,
				  BU27008_MASK_MEAS_MODE, ret);
}

/* Try to change the time so that the scale is maintained */
static int bu27008_try_set_int_time(struct bu27008_data *data, int int_time_new)
{
	int ret, old_time_sel, new_time_sel,  old_gain, new_gain;

	mutex_lock(&data->mutex);

	ret = bu27008_get_int_time_sel(data, &old_time_sel);
	if (ret < 0)
		goto unlock_out;

	if (!iio_gts_valid_time(&data->gts, int_time_new)) {
		dev_dbg(data->dev, "Unsupported integration time %u\n",
			int_time_new);

		ret = -EINVAL;
		goto unlock_out;
	}
	new_time_sel = iio_gts_find_sel_by_int_time(&data->gts, int_time_new);
	if (new_time_sel == old_time_sel) {
		ret = 0;
		goto unlock_out;
	}

	ret = bu27008_get_gain(data, &data->gts, &old_gain);
	if (ret)
		goto unlock_out;

	ret = iio_gts_find_new_gain_sel_by_old_gain_time(&data->gts, old_gain,
				old_time_sel, new_time_sel, &new_gain);
	if (ret) {
		int scale1, scale2;
		bool ok;

		_bu27008_get_scale(data, false, &scale1, &scale2);
		dev_dbg(data->dev,
			"Can't support time %u with current scale %u %u\n",
			int_time_new, scale1, scale2);

		if (new_gain < 0)
			goto unlock_out;

		/*
		 * If caller requests for integration time change and we
		 * can't support the scale - then the caller should be
		 * prepared to 'pick up the pieces and deal with the
		 * fact that the scale changed'.
		 */
		ret = iio_find_closest_gain_low(&data->gts, new_gain, &ok);
		if (!ok)
			dev_dbg(data->dev, "optimal gain out of range\n");

		if (ret < 0) {
			dev_dbg(data->dev,
				 "Total gain increase. Risk of saturation");
			ret = iio_gts_get_min_gain(&data->gts);
			if (ret < 0)
				goto unlock_out;
		}
		new_gain = ret;
		dev_dbg(data->dev, "scale changed, new gain %u\n", new_gain);
	}

	ret = bu27008_set_gain(data, new_gain);
	if (ret)
		goto unlock_out;

	ret = bu27008_set_int_time(data, int_time_new);

unlock_out:
	mutex_unlock(&data->mutex);

	return ret;
}

static int bu27008_meas_set(struct bu27008_data *data, bool enable)
{
	if (enable)
		return regmap_set_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
				       BU27008_MASK_MEAS_EN);

	return regmap_clear_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
				 BU27008_MASK_MEAS_EN);
}

static int bu27008_meas_en(struct bu27008_data *data)
{
	pr_info("Enable measurement\n");
	return bu27008_meas_set(data, true);
}

static int bu27008_meas_dis(struct bu27008_data *data)
{
	pr_info("Disable measurement\n");
	return bu27008_meas_set(data, false);
}

static int bu27008_chan_cfg(struct bu27008_data *data,
			    struct iio_chan_spec const *chan)
{
	int chan_sel;

	if (chan->scan_index == BU27008_BLUE)
		chan_sel = BU27008_BLUE2_CLEAR3;
	else
		chan_sel = BU27008_CLEAR2_IR3;

	chan_sel = FIELD_PREP(BU27008_MASK_CHAN_SEL, chan_sel);

	return regmap_update_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
				  BU27008_MASK_CHAN_SEL, chan_sel);
}

static int bu27008_read_one(struct bu27008_data *data, struct iio_dev *idev,
			    struct iio_chan_spec const *chan, int *val, int *val2)
{
	int ret, int_time;

	ret = bu27008_chan_cfg(data, chan);
	if (ret)
		return ret;

	ret = bu27008_meas_en(data);
	if (ret)
		return ret;

	int_time = bu27008_get_int_time(data);
	if (int_time < 0)
		int_time = 400000;

	msleep((int_time + 500) / 1000);

	ret = bu27008_chan_read_data(data, chan->address, val);
	if (!ret)
		ret = IIO_VAL_INT;

	if (bu27008_meas_dis(data))
		dev_warn(data->dev, "measurement disabling failed\n");

	return ret;
}

static int bu27008_read_raw(struct iio_dev *idev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct bu27008_data *data = iio_priv(idev);
	int busy, ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	{
		busy = iio_device_claim_direct_mode(idev);
		if (busy)
			return -EBUSY;

		mutex_lock(&data->mutex);
		ret = bu27008_read_one(data, idev, chan, val, val2);
		mutex_unlock(&data->mutex);

		iio_device_release_direct_mode(idev);

		return ret;
	}
	case IIO_CHAN_INFO_SCALE:
		ret = bu27008_get_scale(data, chan->scan_index == BU27008_IR,
					val, val2);
		if (ret)
			return ret;

		return IIO_VAL_INT_PLUS_NANO;

	case IIO_CHAN_INFO_INT_TIME:
		ret = bu27008_get_int_time(data);
		if (ret < 0)
			return ret;

		*val = ret;

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int bu27008_set_scale(struct bu27008_data *data,
			     struct iio_chan_spec const *chan,
			     int val, int val2)
{
	int ret, gain_sel, time_sel, i;

	if (chan->scan_index == BU27008_IR)
		return -EINVAL;

	mutex_lock(&data->mutex);

	ret = bu27008_get_int_time_sel(data, &time_sel);
	if (ret < 0)
		goto unlock_out;


	ret = iio_gts_find_gain_sel_for_scale_using_time(&data->gts, time_sel,
						val, val2 * 1000, &gain_sel);
	if (ret) {
		/* Could not support new scale with existing int-time */
		int new_time_sel;

		for (i = 0; i < data->gts.num_itime; i++) {
			new_time_sel = data->gts.itime_table[i].sel;
			ret = iio_gts_find_gain_sel_for_scale_using_time(
				&data->gts, new_time_sel, val, val2 * 1000,
				&gain_sel);
			if (!ret)
				break;
		}
		if (i == data->gts.num_itime) {
			dev_err(data->dev, "Can't support scale %u %u\n", val,
				val2);

			ret = -EINVAL;
			goto unlock_out;
		}

		ret = bu27008_set_int_time_sel(data, new_time_sel);
		if (ret)
			goto unlock_out;
	}


	ret = bu27008_write_gain_sel(data, gain_sel);

unlock_out:
	mutex_unlock(&data->mutex);

	return ret;
}

static int bu27008_write_raw(struct iio_dev *idev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct bu27008_data *data = iio_priv(idev);
	int ret;

	/*
	 * We should not allow changing scale when measurement is ongoing.
	 * This could make values in buffer inconsistent.
	 */
	ret = iio_device_claim_direct_mode(idev);
	if (ret)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		ret = bu27008_set_scale(data, chan, val, val2);
		break;
	case IIO_CHAN_INFO_INT_TIME:
		ret = bu27008_try_set_int_time(data, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	iio_device_release_direct_mode(idev);

	return ret;
}

static int bu27008_validate_trigger(struct iio_dev *idev,
				   struct iio_trigger *trig)
{
	struct bu27008_data *data = iio_priv(idev);

	if (data->trig != trig)
		return -EINVAL;

	return 0;
}

static int bu27008_read_avail(struct iio_dev *idev,
			      struct iio_chan_spec const *chan, const int **vals,
			      int *type, int *length, long mask)
{
	struct bu27008_data *data = iio_priv(idev);

	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		return iio_gts_avail_times(&data->gts, vals, type, length);
	case IIO_CHAN_INFO_SCALE:
		if (chan->channel2 == IIO_MOD_LIGHT_IR)
			return iio_gts_all_avail_scales(&data->gts_ir, vals,
							type, length);
		return iio_gts_all_avail_scales(&data->gts, vals, type, length);
	default:
		return -EINVAL;
	}
}

static const struct iio_info bu27008_info = {
	.read_raw = &bu27008_read_raw,
	.write_raw = &bu27008_write_raw,
	.read_avail = &bu27008_read_avail,
	.validate_trigger = bu27008_validate_trigger,
};

static int bu27008_chip_init(struct bu27008_data *data)
{
	int ret;

	/* Reset */
	ret = regmap_update_bits(data->regmap, BU27008_REG_SYSTEM_CONTROL,
			   BU27008_MASK_SW_RESET, BU27008_MASK_SW_RESET);
	if (ret)
		return dev_err_probe(data->dev, ret, "Sensor reset failed\n");

	msleep(1);

	return ret;
}

static int bu27008_set_drdy_irq(struct bu27008_data *data, bool state)
{
	if (state)
		return regmap_set_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
					BU27008_MASK_INT_EN);
	return regmap_clear_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
				 BU27008_MASK_INT_EN);
}

static int bu27008_trigger_set_state(struct iio_trigger *trig,
				     bool state)
{
	struct bu27008_data *data = iio_trigger_get_drvdata(trig);
	int ret = 0;

	pr_info("trigger %s requested\n", state ? "enable" : "disable");
	mutex_lock(&data->mutex);

	if (data->trigger_enabled != state) {
		pr_info("%s trigger IRQ\n", state? "enabling" : "disabling");
		data->trigger_enabled = state;
		ret = bu27008_set_drdy_irq(data, state);
		if (ret)
			dev_err(data->dev, "Failed to set trigger state\n");
	}
	mutex_unlock(&data->mutex);

	return ret;
}

static const struct iio_trigger_ops bu27008_trigger_ops = {
	.set_trigger_state = bu27008_trigger_set_state,
};

static irqreturn_t bu27008_irq_handler(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct bu27008_data *data = iio_priv(idev);

	data->old_timestamp = data->timestamp;
	data->timestamp = iio_get_time_ns(idev);

	if (data->trigger_enabled)
		return IRQ_WAKE_THREAD;

	return IRQ_NONE;
}

static irqreturn_t bu27008_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *idev = pf->indio_dev;
	struct bu27008_data *data = iio_priv(idev);
	int ret;
	__le16 raw[BU27008_NUM_CHANS + NUM_U16_IN_TSTAMP];

	memset(&raw, 0, sizeof(raw));

	pr_info("Reading data...\n");

	ret = regmap_bulk_read(data->regmap, BU27008_REG_DATA0_LO, data->buffer,
			       BU27008_HW_DATA_SIZE);
	if (ret < 0)
		goto err_read;

	/* Red and green are always in dedicated channels. */
	if (*idev->active_scan_mask & BIT(BU27008_RED))
		raw[BU27008_RED] = data->buffer[BU27008_RED];
	if (*idev->active_scan_mask & BIT(BU27008_GREEN))
		raw[BU27008_GREEN] = data->buffer[BU27008_GREEN];

 	/*
 	 * We need to check the scan mask to determine which of the
 	 * BLUE/CLEAR/IR are enabled so we know which channel is used to
 	 * measure which data.
	 */
	if (*idev->active_scan_mask & BIT(BU27008_BLUE)) {
		raw[BU27008_BLUE] = data->buffer[BU27008_DATA2];

		if (*idev->active_scan_mask & BIT(BU27008_CLEAR))
			raw[BU27008_CLEAR] = data->buffer[BU27008_DATA3];
	} else {
		if (*idev->active_scan_mask & BIT(BU27008_CLEAR))
			raw[BU27008_CLEAR] = data->buffer[BU27008_DATA2];
	}
	if (*idev->active_scan_mask & BIT(BU27008_IR))
		raw[BU27008_IR] = data->buffer[BU27008_DATA3];

	iio_push_to_buffers_with_timestamp(idev, raw, pf->timestamp);
err_read:
	iio_trigger_notify_done(idev->trig);

	return IRQ_HANDLED;
}

static irqreturn_t bu27008_irq_thread_handler(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct bu27008_data *data = iio_priv(idev);
	irqreturn_t ret = IRQ_NONE;

	mutex_lock(&data->mutex);

	if (data->trigger_enabled) {
		iio_trigger_poll_chained(data->trig);
		ret = IRQ_HANDLED;
	}

	mutex_unlock(&data->mutex);

	return ret;
}

static int bu27008_buffer_preenable(struct iio_dev *idev)
{
	struct bu27008_data *data = iio_priv(idev);
	int chan_sel, ret;

	pr_info("Enabling buffer, scan mask 0x%x\n", *idev->active_scan_mask);
	/* Configure channel selection */
	if (*idev->active_scan_mask & BIT(BU27008_BLUE)) {
		if (*idev->active_scan_mask & BIT(BU27008_CLEAR))
			chan_sel = BU27008_BLUE2_CLEAR3;
		else
			chan_sel = BU27008_BLUE2_IR3;
	} else {
		chan_sel = BU27008_CLEAR2_IR3;
	}

	chan_sel = FIELD_PREP(BU27008_MASK_CHAN_SEL, chan_sel);

	ret = regmap_update_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
				 BU27008_MASK_CHAN_SEL, chan_sel);
	if (ret)
		return ret;

	return bu27008_meas_en(data);
}

static int bu27008_buffer_postdisable(struct iio_dev *idev)
{
	struct bu27008_data *data = iio_priv(idev);

	pr_info("Disabling buffer\n");

	return bu27008_meas_dis(data);
}

static const struct iio_buffer_setup_ops bu27008_buffer_ops = {
	.preenable = bu27008_buffer_preenable,
	.postdisable = bu27008_buffer_postdisable,
};

static int bu27008_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct iio_trigger *indio_trig;
	struct bu27008_data *data;
	struct regmap *regmap;
	unsigned int part_id, reg;
	struct iio_dev *idev;
	char *name;
	int ret;

	if (!i2c->irq) {
		dev_err(dev, "No IRQ configured\n");
		return -EINVAL;
	}

	regmap = devm_regmap_init_i2c(i2c, &bu27008_regmap);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to initialize Regmap\n");

	idev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!idev)
		return -ENOMEM;

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulator\n");

	data = iio_priv(idev);

	ret = regmap_read(regmap, BU27008_REG_SYSTEM_CONTROL, &reg);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to access sensor\n");

	part_id = FIELD_GET(BU27008_MASK_PART_ID, reg);

	if (part_id != BU27008_ID)
		dev_warn(dev, "unknown device 0x%x\n", part_id);

	ret = devm_iio_init_iio_gts(dev, BU27008_SCALE_1X, 0, bu27008_gains,
				    ARRAY_SIZE(bu27008_gains), bu27008_itimes,
				    ARRAY_SIZE(bu27008_itimes), &data->gts);
	if (ret)
		return ret;

	ret = devm_iio_init_iio_gts(dev, BU27008_SCALE_1X, 0, bu27008_gains_ir,
				    ARRAY_SIZE(bu27008_gains_ir), bu27008_itimes,
				    ARRAY_SIZE(bu27008_itimes), &data->gts_ir);
	if (ret)
		return ret;

	mutex_init(&data->mutex);
	data->regmap = regmap;
	data->dev = dev;
	data->irq = i2c->irq;

	idev->channels = bu27008_channels;
	idev->num_channels = ARRAY_SIZE(bu27008_channels);
	idev->name = "bu27008";
	idev->info = &bu27008_info;
	idev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	idev->available_scan_masks = bu27008_scan_masks;

	ret = bu27008_chip_init(data);
	if (ret)
		return ret;
	ret = devm_iio_triggered_buffer_setup_ext(dev, idev,
						  &iio_pollfunc_store_time,
						  bu27008_trigger_handler,
						  IIO_BUFFER_DIRECTION_IN,
						  &bu27008_buffer_ops,
						  NULL);

	if (ret)
		return dev_err_probe(data->dev, ret,
				     "iio_triggered_buffer_setup_ext FAIL\n");

	indio_trig = devm_iio_trigger_alloc(dev, "%sdata-rdy-dev%d", idev->name,
					    iio_device_id(idev));
	if (!indio_trig)
		return -ENOMEM;

	data->trig = indio_trig;

	indio_trig->ops = &bu27008_trigger_ops;
	iio_trigger_set_drvdata(indio_trig, data);

	/*
	 * No need to check for NULL. request_threaded_irq() defaults to
	 * dev_name() should the alloc fail.
	 */
	name = devm_kasprintf(data->dev, GFP_KERNEL, "%s-bu27008",
			      dev_name(data->dev));

	ret = devm_request_threaded_irq(data->dev, i2c->irq, bu27008_irq_handler,
					&bu27008_irq_thread_handler,
					IRQF_ONESHOT, name, idev);
	if (ret)
		return dev_err_probe(data->dev, ret, "Could not request IRQ\n");


	ret = devm_iio_trigger_register(dev, indio_trig);
	if (ret)
		return dev_err_probe(data->dev, ret,
				     "Trigger registration failed\n");

	ret = devm_iio_device_register(data->dev, idev);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Unable to register iio device\n");

	return ret;
}

static const struct of_device_id bu27008_of_match[] = {
	{ .compatible = "rohm,bu27008", },
	{ }
};
MODULE_DEVICE_TABLE(of, bu27008_of_match);

static struct i2c_driver bu27008_i2c_driver = {
	.driver = {
		.name = "bu27008",
		.of_match_table = bu27008_of_match,
	},
	.probe_new = bu27008_probe,
};
module_i2c_driver(bu27008_i2c_driver);

MODULE_DESCRIPTION("ROHM BU27008 colour sensor driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_GTS_HELPER);
