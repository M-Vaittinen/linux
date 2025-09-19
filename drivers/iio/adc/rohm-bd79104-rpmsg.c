// SPDX-License-Identifier: GPL-2.0-only
/*
 * ROHM ADC driver for BD79104 ADC/GPO device when the SPI access to the ADC
 * is handled by a co-processor accessible via RPMSG channel.
 *
 * Copyright (c) 2025, ROHM Semiconductor.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/rpmsg.h>
#include <linux/types.h>
#include <linux/wait.h>

//#include <uapi/linux/rpmsg.h>

#include <asm/byteorder.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>

#define BD79104_ADC_BITS 12
#define BD79104_MAX_NUM_CHANNELS 8

/* For now we support only case where all channels are measured */
/*
static const unsigned long bd79104_rpmsg_scan_masks[] = {
	GENMASK(0, 8), 0
};
*/

static DECLARE_WAIT_QUEUE_HEAD(bd79104_rpmsg_wq);

struct bd79104_rpmsg_data {
//	s64 timestamp;
	struct iio_dev *idev;
	struct device *dev;
	int vref_mv;
	bool enreq;
	bool disreq;
	unsigned int errors;
	//struct iio_trigger *trig;
	struct rpmsg_device *rpdev;
	spinlock_t msg_lock;
//	struct mutex mutex;
};

/*
 * TODO: Support capturing only some channels
 * 	- Requires the PRU to clean data when channel config is
 * 	changed and to only capture data for given channels.
 */
#define BD79104_RPMSG_ADC_CHAN(_index)				\
{								\
	.type = IIO_VOLTAGE,					\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.indexed = 1,						\
	.channel = _index,						\
	.scan_index = (_index),					\
	.scan_type = { 						\
		.sign = 'u',					\
		.realbits = 12,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,				\
	},							\
}

static const struct iio_chan_spec bd79104_rpmsg_channels[] = {
	BD79104_RPMSG_ADC_CHAN(0),
	BD79104_RPMSG_ADC_CHAN(1),
	BD79104_RPMSG_ADC_CHAN(2),
	BD79104_RPMSG_ADC_CHAN(3),
	BD79104_RPMSG_ADC_CHAN(4),
	BD79104_RPMSG_ADC_CHAN(5),
	BD79104_RPMSG_ADC_CHAN(6),
	BD79104_RPMSG_ADC_CHAN(7),
	IIO_CHAN_SOFT_TIMESTAMP(8),
};

static int bd79104_rpmsg_read_raw(struct iio_dev *iio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long m)
{
	struct bd79104_rpmsg_data *data = iio_priv(iio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		*val = data->vref_mv;
		*val2 = BD79104_ADC_BITS;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

#if 0
static int bd79104_rpmsg_fifo_flush(struct iio_dev *idev, unsigned int samples)
{
	struct bd79104_rpmsg_data *data = iio_priv(idev);
	int ret;

	mutex_lock(&data->mutex);
	/*
	 * Send the flush rpmsg and return 'no data' from here?
	 * The data will be then received by the virtio RX hook.
	 */
	ret = __bd79104_rpmsg_fifo_flush(idev, samples, false);
	mutex_unlock(&data->mutex);

	return ret;
}
#endif

static int bd79104_rpmsg_buffer_predisable(struct iio_dev *idev)
{
	struct bd79104_rpmsg_data *d = iio_priv(idev);
	int ret;

	ret = rpmsg_send(d->rpdev->ept, "0", 2);
	if (ret) {
		dev_err(d->dev, "RPMSG send failed %d\n", ret);
		return ret;
	}

	d->disreq = 1;
	wait_event_interruptible_timeout(bd79104_rpmsg_wq, !d->disreq, msecs_to_jiffies(1500));
	if (d->disreq) {
		dev_err(d->dev, "Disable request to PRU timed-out\n");
		return -EIO;
	}

	return 0;
}

static int bd79104_rpmsg_buffer_preenable(struct iio_dev *idev)
{
	struct bd79104_rpmsg_data *d = iio_priv(idev);
	int ret;
	unsigned char msg[2];

	msg[0] = '1';
	msg[1] = *idev->active_scan_mask;

	ret = rpmsg_send(d->rpdev->ept, msg, 2);
	if (ret) {
		dev_err(d->dev, "RPMSG send failed %d\n", ret);
		return ret;
	}

	d->enreq = 1;
	wait_event_interruptible_timeout(bd79104_rpmsg_wq, !d->enreq, msecs_to_jiffies(1500));
	if (d->enreq) {
		dev_err(d->dev, "Enable request to PRU timed-out\n");
		return -EIO;
	}

	return ret;
}

static const struct iio_buffer_setup_ops bd79104_rpmsg_buffer_ops = {
	.preenable = bd79104_rpmsg_buffer_preenable,
	.predisable = bd79104_rpmsg_buffer_predisable,
};

static const struct iio_info bd79104_rpmsg_info = {
	.read_raw = bd79104_rpmsg_read_raw,
/*	.hwfifo_flush_to_buffer	= bd79104_rpmsg_fifo_flush, */
};

static int bd79104_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct bd79104_rpmsg_data *data;
	struct iio_dev *iio_dev;
	int ret, i;

	pr_info("probed\n");

	iio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!iio_dev)
		return -ENOMEM;

	data = iio_priv(iio_dev);
	data->idev = iio_dev;

	spin_lock_init(&data->msg_lock);
	rpdev->ept->priv = data;
	data->rpdev = rpdev;

	data->dev = dev;
/*
	ret = devm_regulator_get_enable_read_voltage(dev, "vdd");
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get the Vdd\n");

	data->vref_mv = ret / 1000;
	*/

	data->vref_mv = 3300;

	ret = devm_regulator_get_enable(dev, "iovdd");
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to enable I/O voltage\n");

	iio_dev->channels = bd79104_rpmsg_channels;
	iio_dev->num_channels = ARRAY_SIZE(bd79104_rpmsg_channels);
	iio_dev->info = &bd79104_rpmsg_info;
	iio_dev->name = "bd79104_rpmsg";
//	iio_dev->modes = INDIO_DIRECT_MODE;

	for (i = 0; i < iio_dev->num_channels; i++)
		pr_info("ch %d\n",iio_dev->channels->channel);

	ret = devm_iio_kfifo_buffer_setup(dev, iio_dev,
					  &bd79104_rpmsg_buffer_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(data->dev, iio_dev);
	if (ret)
		dev_err_probe(data->dev, ret, "Failed to register ADC\n");

	return ret;
}

#define RPMSG_MESSAGE_SIZE                      496
#define MAX_ADC_DATA_BUF (RPMSG_MESSAGE_SIZE - 2)
#define MAX_ADC_DATA_BUF_IDX (MAX_ADC_DATA_BUF / 2)

#define WINDEX(_msg) ((_msg)[RPMSG_MESSAGE_SIZE - 1])
//#define RINDEX(_msg) ((_msg)[RPMSG_MESSAGE_SIZE - 2])
#define MEASURED_CHANS(_msg) ((_msg)[RPMSG_MESSAGE_SIZE - 2])

struct bd79104_rpmsg_sample_1 {
	uint16_t chdata[1];
	s64 ts;
};

struct bd79104_rpmsg_sample_2 {
	uint16_t chdata[2];
	s64 ts;
};

struct bd79104_rpmsg_sample_3 {
	uint16_t chdata[3];
	s64 ts;
};

struct bd79104_rpmsg_sample_4 {
	uint16_t chdata[4];
	s64 ts;
};

struct bd79104_rpmsg_sample_5 {
	uint16_t chdata[5];
	s64 ts;
};

struct bd79104_rpmsg_sample_6 {
	uint16_t chdata[6];
	s64 ts;
};

struct bd79104_rpmsg_sample_7 {
	uint16_t chdata[7];
	s64 ts;
};

struct bd79104_rpmsg_sample_8 {
	uint16_t chdata[8];
	s64 ts;
};

/* This should be the amount of time PRU spends reading a single data entry. */
#define PRU_SINGLE_SAMP_TIME 50
/* This should be the the time PRU spends handling the RPMSGs between the data reads */
#define PRU_CHUNCK_SWITCH_TIME 2

static void bd79104_rpmsg_copy_data(struct bd79104_rpmsg_data *d, char *buf,
				    s64 timestamp)
{
	uint16_t *data = (uint16_t *)buf;
	unsigned long channels = MEASURED_CHANS(buf);
	unsigned long chan;
	unsigned int chunk, samples, num_chans = 0, chunks, idx = 0;
	int ret;

	samples = WINDEX(buf);

	for_each_set_bit(chan, &channels, 8) {
		num_chans++;
	}

//	pr_info("Num channels %u\n", num_chans);
	if (unlikely(!num_chans)) {
		pr_info("WOOT?? Num channels %u\n", num_chans);
        	spin_lock(&d->msg_lock);
		d->errors++;
        	spin_unlock(&d->msg_lock);
		return;
	}

	chunks = samples / num_chans;
	if (unlikely(chunks * num_chans != samples)) {
		pr_info("Data and sample amount not matching. Samples %u, chans %u\n",
			samples, num_chans);
        	spin_lock(&d->msg_lock);
		d->errors++;
        	spin_unlock(&d->msg_lock);
		return;
	}

	if (unlikely(samples * sizeof(uint16_t) > MAX_ADC_DATA_BUF)) {
		pr_err("Bad RPMSG size. Samples %u (size %u, max %u)\n",
			 samples, samples * sizeof(uint16_t), MAX_ADC_DATA_BUF);
        	spin_lock(&d->msg_lock);
		d->errors++;
        	spin_unlock(&d->msg_lock);
		return;
	}
		
	for (chunk = 0; chunk < chunks; chunk++) {
		struct bd79104_rpmsg_sample_1 smp1;
		struct bd79104_rpmsg_sample_2 smp2;
		struct bd79104_rpmsg_sample_3 smp3;
		struct bd79104_rpmsg_sample_4 smp4;
		struct bd79104_rpmsg_sample_5 smp5;
		struct bd79104_rpmsg_sample_6 smp6;
		struct bd79104_rpmsg_sample_7 smp7;
		struct bd79104_rpmsg_sample_8 smp8;
		void *start[] = {
			&smp1,
			&smp2,
			&smp3,
			&smp4,
			&smp5,
			&smp6,
			&smp7,
			&smp8,
		};
		size_t dsize[] = {
			sizeof(smp1),
			sizeof(smp2),
			sizeof(smp3),
			sizeof(smp4),
			sizeof(smp5),
			sizeof(smp6),
			sizeof(smp7),
			sizeof(smp8),
		};
		uint16_t *firstch[] = {
			&smp1.chdata[0],
			&smp2.chdata[0],
			&smp3.chdata[0],
			&smp4.chdata[0],
			&smp5.chdata[0],
			&smp6.chdata[0],
			&smp7.chdata[0],
			&smp8.chdata[0],
		};
		
		for_each_set_bit(chan, &channels, 8) {
			*firstch[num_chans - 1] = data[idx++];
			firstch[num_chans - 1]++;
		}

		ret = iio_push_to_buffers_with_ts(d->idev,
					      start[num_chans - 1], dsize[num_chans - 1],
					      timestamp + idx * PRU_SINGLE_SAMP_TIME
						+ chunk * PRU_CHUNCK_SWITCH_TIME);
		if (ret) {
			dev_err(d->dev, "Failed to push buffers, %d\n", ret);
			return;
		}
	}
}

static void dbgmsg(char *buf, int len)
{
	int i, i2 = 0;
	char tmp[255];

	pr_info("MSG: (len %u)\n", len);

	for (i = 0; i < len && i < 20; i++)
		i2 += sprintf(&tmp[i2], "%02x ", buf[i]);

	tmp[254] = '\0';

	pr_info("%s\n", tmp);
}

static int bd79104_rpmsg_ept_cb(struct rpmsg_device *rpdev, void *buf, int len,
                        void *priv, u32 addr)
{
	struct bd79104_rpmsg_data *d = priv;
	s64 ts, ts_start, ts_end;
	unsigned long tmp;

	ts_start = iio_get_time_ns(d->idev);

	if (len != RPMSG_MESSAGE_SIZE)
		dbgmsg(buf, len);

	if (len == RPMSG_MESSAGE_SIZE) {
		ts = iio_get_time_ns(d->idev);
		bd79104_rpmsg_copy_data(d, buf, ts);
	}
	if (len == 2) {
 		uint8_t * msg = buf;

		if (*msg == '0') {
			d->disreq = 0;
		} else if (*msg == '1') {
			d->enreq = 0;
		} else {
	        	spin_lock(&d->msg_lock);
			d->errors++;
	        	spin_unlock(&d->msg_lock);
			return 0;
		}
		wake_up_interruptible(&bd79104_rpmsg_wq);
	}

	ts_end = iio_get_time_ns(d->idev);
	tmp = ts_end - ts_start;
	pr_info("ISR took %llu ns, %lu us\n", ts_end - ts_start, tmp / 1000);

        return 0;
}

static struct rpmsg_device_id bd79104_rpmsg_id_table[] = {
        { .name = "rpmsg-bd79104" },
        { },
};

static struct rpmsg_driver bd79104_rpmsg_driver = {
        .probe = bd79104_rpmsg_probe,
        .callback = bd79104_rpmsg_ept_cb,
        .id_table = bd79104_rpmsg_id_table,
        .drv.name = "bd79104_rpmsg",
};

//static int bd79104_rpmsg_init(void)
static int bd79104_plat_probe(struct platform_device *pdev)
{
	int ret;

	pr_info("Platform driver probed\n");
	/*
	 * TODO: Enable supply regulators and deliver voltage data to
	 * rpmsg instance.
	 */
	ret = register_rpmsg_driver(&bd79104_rpmsg_driver);
	if (ret < 0)
		pr_err("rpmsg: failed to register rpmsg raw driver\n");

	return ret;
}
//postcore_initcall(bd79104_rpmsg_init);

//static void bd79104_rpmsg_exit(void)
static void bd79104_plat_remove(struct platform_device *pdev)
{
	unregister_rpmsg_driver(&bd79104_rpmsg_driver);
}
//module_exit(bd79104_rpmsg_exit);

static const struct of_device_id bd79104_rpmsg_plat_id[] = {
	{ .compatible = "rohm,bd79104-rpmsg", },
	{ },
};
MODULE_DEVICE_TABLE(of, bd79104_rpmsg_plat_id);

static struct platform_driver bd79104_rpmsg = {
	.driver = {
		.name = "bd79104-rpmsg",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = bd79104_rpmsg_plat_id,
	},
	.probe = bd79104_plat_probe,
	.remove = bd79104_plat_remove,
//	.id_table = bd79104_rpmsg_plat_id,
};
module_platform_driver(bd79104_rpmsg);

MODULE_AUTHOR("Matti Vaittinen <mazziesaccount@gmail.com>");
MODULE_DESCRIPTION("RPMSG Driver for ROHM BD79104 ADC");
MODULE_LICENSE("GPL");
