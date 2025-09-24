/*
 * Copyright (C) 2016-2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include "resource_table.h"
#include "intc_map_0.h"

#ifndef BIT
	#define BIT(_n) (1 << (_n))
#endif

/* Mapping Constant Table (CT) registers to variables */
volatile far uint8_t CT_MCSPI0 __attribute__((cregister("MCSPI0", near), peripheral));

/*
 * Used SPI Pins:
 *
 * SPI0
 *  - spi0_d0 (P9_21). Offset 0x954 MISO (INPUT, PULL-UP, MODE 0)
 *  - spi0_d1 (P9_18). Offset 0x958 MOSI (OUTPUT, PULL-UP, MODE 0)
 *  - spi0_sclk (P9_22). Offset 0x950 SCLK (INPUT(!), PULL-UP, MODE 0)
 *  - spi0_cs0 (P9_17). Offset 0x95C CS/EN (INPUT(?), PULL-UP, MODE 0)
 */

/*
 * CPU speed is 200 MHz. Assuming this equals to cycles..
 * Our SPI Max freq is 20 MHz. So, 10 cycles delay for 1 clk.
 * One data transfer is 16 clks => 160 cycles. Let's see how our ARM dies...
 *
 * To just calculate further, we can fit 240 samples (when 8 channels enabled)
 * in the buffer.
 *
 * 1 clk cycle is 1 000 000 000 nsec / 20 000 000 MHz
 * => 50 ns.
 * 1 sample is 16 cycles =>
 * 50 ns * 16 => 800 ns.
 *
 * Filling the buffer takes 800 ns * 240 = 192000 ns which is
 * 192 us.
 */
#define TEST_DELAY 16000

/*
 * Let's verify above assumption. If cycles equal to 200 MHz CPU speed -
 * then there should be 200 000 000 cycles in a second.
 */
#define ONESEC_TEST_DELAY 200000000

#define MVA_PINCFG_BASE ((volatile uint8_t *)(0x44e10000))

#define PIN_MISO_OFFSET 0x954
#define PIN_MOSI_OFFSET 0x958
#define PIN_SCLK_OFFSET 0x950
#define PIN_CSEN_OFFSET 0x95C
 
#define PIN_MISO_CFG (MVA_PINCFG_BASE + PIN_MISO_OFFSET)
#define PIN_MOSI_CFG (MVA_PINCFG_BASE + PIN_MOSI_OFFSET)
#define PIN_SCLK_CFG (MVA_PINCFG_BASE + PIN_SCLK_OFFSET)
#define PIN_CSEN_CFG (MVA_PINCFG_BASE + PIN_CSEN_OFFSET)

/* PRCM Registers */
#define CM_PER_BASE		((volatile uint8_t *)(0x44E00000))
#define SPI0_CLKCTRL		(0x4C)
#define ON			(0x2)

#define MCSPI0_REVISION		(*((volatile uint32_t*)(&CT_MCSPI0)))
#define REG_MCSPI0_MODULCTRL	(*((volatile uint32_t*)(&CT_MCSPI0 + 0x128)))
#define REG_MCSPI_CH0CONF	(*((volatile uint32_t*)(&CT_MCSPI0 + 0x12C)))
#define REG_MCSPI_XFERLEVEL	(*((volatile uint32_t*)(&CT_MCSPI0 + 0x17c)))

#define REG_MCSPI_CH0STAT	(*((volatile uint32_t*)(&CT_MCSPI0 + 0x130)))
#define REG_MCSPI_CH0CTRL	(*((volatile uint32_t*)(&CT_MCSPI0 + 0x134)))
#define MCSPI_TX0		(*((volatile uint32_t*)(&CT_MCSPI0 + 0x138)))
#define MCSPI_RX0		(*((volatile uint32_t*)(&CT_MCSPI0 + 0x13C)))

#define MASK_XFERLEVEL_WCNT		0x0000FFFF
#define MASK_CHCTRL_EXTCLK		0xFF00
#define MASK_CHCTRL_CHEN		0x1
#define MASK_CHCONF_CLKG		BIT(29)
#define MASK_CHCONF_POL			BIT(1)
#define MASK_CHCONF_PHA			BIT(0)
#define MASK_CHCONF_CLKD		0x3c
#define DO_CLKD(_clkd)			(((_clkd) << 2) & MASK_CHCONF_CLKD)
#define MASK_CHCONF_FORCE		BIT(20)
#define MASK_CHCONF_TURBO		BIT(19)
#define MASK_CHCONF_INSIG		BIT(18)
#define DATAINLINE(_n)			((_n) ? MASK_CHCONF_INSIG : 0)
#define MASK_CHCONF_D0_TXEN		BIT(17)
#define MASK_CHCONF_D1_TXEN		BIT(16)
#define MASK_CHCONF_FIFO_RX_EN		BIT(28)
#define MASK_CHCONF_FIFO_TX_EN		BIT(27)
#define MASK_CHCONF_FIFO_CS_TIME	0x6000000
#define CS_TIME_05			0
#define CS_TIME_15			(1 << 25)
#define CS_TIME_25			(2 << 25)
#define CS_TIME_35			(3 << 25)
#define MASK_CHCONF_STARTB_USE		BIT(23)
#define MASK_CHCONF_STARTB_POL_HI	BIT(24)
#define MASK_CHCONF_DMA_READREQ_EN	BIT(15)
#define MASK_CHCONF_DMA_WRITEREQ_EN	BIT(14)
#define MASK_CHCONF_TXRX_MODE		0x3000
#define TXRX_MODE_TXRX			0
#define TXRX_MODE_TX_ONLY		(2 << 12)
#define TXRX_MODE_RX_ONLY		(1 << 12)
#define MASK_CHCONF_SPI_WORDLEN		0xF80
#define DO_SPI_WORDLEN(_bits)		(((_bits) - 1) << 7)
#define MASK_CHCONF_CS_POL		BIT(6)
#define CS_ACTIVE_HI			0
#define CS_ACTIVE_LOW			(1 << 6)

#define MASK_MODULCTRL_SPI_SLAVE	BIT(2)
#define MASK_MODULCTRL_SPI_NO_CS	BIT(1)

#define MASK_CHSTAT_TXS			BIT(1)
#define MASK_CHSTAT_RXS			BIT(0)

volatile register uint32_t __R31;

uint32_t swap32(uint32_t sw)
{
	/* return ((sw & 0xff000000) >> 24) |
	       ((sw & 0x00ff0000) >> 8) |
	       ((sw & 0x0000ff00) << 8) |
	       ((sw & 0x000000ff) << 24);
	       */
	return sw;
}

static inline void ch0_endis(int en)
{
	volatile uint32_t reg = REG_MCSPI_CH0CTRL;

	if (en)
		reg |= swap32(MASK_CHCTRL_CHEN);
	else
		reg &= swap32(~MASK_CHCTRL_CHEN);

	REG_MCSPI_CH0CTRL = reg;
}

/* Host-0 Interrupt sets bit 30 in register R31 */
#define HOST_INT			((uint32_t) 1 << 30)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			16
#define FROM_ARM_HOST			17

/*
 * Using the name 'rpmsg-raw' will probe the rpmsg_char driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_char.c
 */
#define CHAN_NAME			"rpmsg-bd79104"
#define CHAN_PORT			30

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

#define PRU_START_MSG '1'
#define PRU_STOP_MSG '0'
#define PRU_ADC_DATA_FLUSH '2'

uint8_t payload_rx[RPMSG_MESSAGE_SIZE];
uint16_t payload_tx[RPMSG_MESSAGE_SIZE / 2];

#define WINDEX (((char *)payload_tx)[RPMSG_MESSAGE_SIZE - 1])
//#define RINDEX (((char *)payload_tx)[RPMSG_MESSAGE_SIZE - 2])
#define MEASURED_CHANS (((char *)payload_tx)[RPMSG_MESSAGE_SIZE - 2])

/* The last two bytes in message are read and write index */
#define MAX_ADC_DATA_BUF (RPMSG_MESSAGE_SIZE - 2)
#define MAX_ADC_DATA_BUF_IDX (MAX_ADC_DATA_BUF / 2)

enum pru_state {
	PRU_IDLE = 0,
	PRU_STARTED,
	PRU_STOPPED,
	PRU_NUM_STATES,
};

struct pru_data {
	struct pru_rpmsg_transport  *transport;
	uint32_t src;
	uint32_t dst;
	uint8_t measured_channels;
	enum pru_state state;
};

static struct pru_data g_data = {0};
static unsigned int WATERMARK;

static void mva_debug(void *buf, int len)
{
	if (len > MAX_ADC_DATA_BUF)
		len = MAX_ADC_DATA_BUF;
	memcpy(payload_rx, buf, len);
	pru_rpmsg_send(g_data.transport, g_data.src, g_data.dst, payload_rx, len);
	__delay_cycles(ONESEC_TEST_DELAY);
}

#if 0
static void read_test_mcspi_revision(void)
{
	uint32_t revision = MCSPI0_REVISION;

	mva_debug((uint8_t *)&revision, 4);
}

void hiawathatest(void)
{
	uint16_t test = 1;
	uint8_t *one = (uint8_t *)&test;
	uint8_t msg;

	if (*one)
		msg = 0x1e;
	else
		msg = 0xbe;

	mva_debug(&msg, 1);
}

static void mva_pin_dbg(void)
{
	uint32_t reg[4];
	int i;

	reg[0] = *((volatile uint32_t *)(PIN_MISO_CFG));
	reg[1] = *((volatile uint32_t *)(PIN_MOSI_CFG));
	reg[2] = *((volatile uint32_t *)(PIN_SCLK_CFG));
	reg[3] = *((volatile uint32_t *)(PIN_CSEN_CFG));

	for (i = 0; i < 4; i++)
		mva_debug(&reg[i], sizeof(reg[i]));
}
#endif

static void spi_init(void)
{
	volatile uint8_t *ptr_cm;
	int result;
	uint32_t mask, val;
       	
	ptr_cm = CM_PER_BASE;

	/* Read IEPCLK[OCP_EN] for IEP clock source */
	result = CT_CFG.IEPCLK_bit.OCP_EN;

	/*****************************************************************/
	/* Access SoC peripherals using Constant Table                   */
	/*****************************************************************/

	/* Access PRCM (without CT) to initialize McSPI0 clock */
	ptr_cm[SPI0_CLKCTRL] = ON;

	ch0_endis(0);

	/*
	 * REG_MCSPI0_MODULCTRL
	 * Master MODE. use CS.
	 */
//	mask = ~(MASK_MODULCTRL_SPI_SLAVE | MASK_MODULCTRL_SPI_NO_CS);
//	val = REG_MCSPI0_MODULCTRL;
//	val &= swap32(~mask);
//	REG_MCSPI0_MODULCTRL = val;

	/*
	 * Single master mode. CS controlled by MCSPI. No test mode. No DMA.
	 * TODO: See if INITDLY needs to be added
	 */
	REG_MCSPI0_MODULCTRL = 0;


	/* CLK conf:
	 * REG_MCSPI_CH0CTRL: EXTCLK 0
	 * => 16MHz, MODE 3
	 */
	val = REG_MCSPI_CH0CTRL;
	val &= swap32(~MASK_CHCTRL_EXTCLK);

	REG_MCSPI_CH0CTRL = val;

	/*
	 * REG_MCSPI_CH0CONF:
	 * CLKD 2, CLKG, 1, PHA 1, POL 1 (16 MHz, MODE 3)
	 * Ensure FORCE, TURBO, D0/D1_UNUSED, STARTBIT, DMA_REQs are cleared.
	 * Set TXRX mode, 16bit SPI WORDs and CS_ACTIVE_LOW.
	 */
	mask = MASK_CHCONF_CLKD | MASK_CHCONF_CLKG | MASK_CHCONF_POL |
	       MASK_CHCONF_PHA | MASK_CHCONF_FORCE | MASK_CHCONF_TURBO |
	       MASK_CHCONF_STARTB_USE | MASK_CHCONF_FIFO_CS_TIME |
	       MASK_CHCONF_DMA_READREQ_EN | MASK_CHCONF_DMA_WRITEREQ_EN |
	       MASK_CHCONF_TXRX_MODE | MASK_CHCONF_SPI_WORDLEN |
	       MASK_CHCONF_CS_POL | MASK_CHCONF_INSIG | MASK_CHCONF_D0_TXEN |
	       MASK_CHCONF_D1_TXEN;

	val = REG_MCSPI_CH0CONF;
	val = swap32(val);
	
	val &= ~mask;
	val |= MASK_CHCONF_CLKG | MASK_CHCONF_POL | MASK_CHCONF_PHA |
	       DO_CLKD(2) | TXRX_MODE_TXRX | DATAINLINE(0) |
	       MASK_CHCONF_D1_TXEN | DO_SPI_WORDLEN(16) | CS_ACTIVE_LOW |
	       CS_TIME_35;

	val = swap32(val);
	//mva_debug(&val, 4);

	REG_MCSPI_CH0CONF = val;

	val = REG_MCSPI_CH0CONF;
	//mva_debug(&val, 4);

	val = REG_MCSPI_XFERLEVEL;
	val = swap32(val);

	mask = MASK_XFERLEVEL_WCNT;
	val &= ~mask;
	/*
	 * TODO: If we need deeper FIFO, we can assign this when ARM configures
	 * the enabled channels. That way we can write / read all channels
	 * to/from the FIFO, before polling for the STATUS bit
	 */
//	val |= (1 << 16);

	REG_MCSPI_XFERLEVEL = swap32(val);

}

static inline int tx0_avail(void)
{
//	uint32_t mask = 0;
//	uint8_t *mp = ((uint8_t *)mask);


	return REG_MCSPI_CH0STAT & MASK_CHSTAT_TXS;
}

/*
 * Since our FIFO is 1 WORD, the 'data avail' trigs to 'register full' status */
static inline int rx0_data_avail(unsigned int try)
{
	if (try > 1000) {
		uint32_t dbg[2];

		dbg[0] = try - 1000;
		dbg[1] = REG_MCSPI_CH0STAT;

		mva_debug(&dbg[0], sizeof(dbg));
	}
/*	uint32_t mask = 0;
	uint8_t *mp = ((uint8_t *)mask);

	mp[3] = MASK_CHSTAT_RXS;
*/
	return REG_MCSPI_CH0STAT & MASK_CHSTAT_RXS;
}

static int pru_spi_read_adc_data(int chan, uint16_t *data)
{
	uint32_t tx = 0 , rx, dbg;
	volatile uint8_t *txp = (uint8_t *)&tx;
	unsigned int try = 0;

//	dbg = REG_MCSPI_CH0STAT;
//	mva_debug(&dbg, 4);

	ch0_endis(1);
	txp[2] = chan << 3;

	//mva_debug(&tx, 4);
//	dbg = REG_MCSPI_CH0STAT;

//	mva_debug(&dbg, 4);

	//mva_debug(&dbg, 1);
	while (!tx0_avail())
		;

//	dbg = REG_MCSPI_CH0STAT;
//	mva_debug(&dbg, 4);

	//dbg = 1;
	//mva_debug(&dbg, 1);

	MCSPI_TX0 = tx;

	while (!rx0_data_avail(try++))
		;

	//dbg = 3;
	//mva_debug(&dbg, 1);

	rx = MCSPI_RX0;
	/* mva_debug(&rx, 4); */

	//*data = MCSPI_RX0 >> 16;
	*data = swap32(rx);
	
	ch0_endis(0);

	/* Just try out some delay */
	//__delay_cycles(0xffff);
	__delay_cycles(TEST_DELAY);

	return 0;
}

static void pru_add_data(uint16_t data)
{
	//uint8_t dbg[3];

	payload_tx[WINDEX] = data;

	WINDEX++;
	//dbg[0] = WINDEX;
	//(*((uint16_t *)&dbg[1])) = data;
	//mva_debug(&dbg, 3);
	/*
	 * Should not happend as the data should be sent and indexes cleared
	 * when the WATERMARK is reached
	 */
	if (WINDEX == MAX_ADC_DATA_BUF_IDX)
		WINDEX = 0;
	/*
	 * Should not happend as the data should be sent and indexes cleared
	 * when the WATERMARK is reached
	 */
//	if (WINDEX == RINDEX)
//		RINDEX = (RINDEX + 1) % MAX_ADC_DATA_BUF_IDX;
}

static void pru_adc_data_send(struct pru_data *d)
{
	/* ATM, the RINDEX is always expected to be 0 */
	pru_rpmsg_send(g_data.transport, g_data.src, g_data.dst, payload_tx, sizeof(payload_tx));
	/* RINDEX = 0; */
	WINDEX = 0;
	__delay_cycles(ONESEC_TEST_DELAY);
}

static void pru_adc_read(struct pru_data *d)
{
	uint16_t data;
	int ret;
	int chan;

	for (chan = 0; chan < 8; chan++)
		if (MEASURED_CHANS & (1 << chan))
			ret = pru_spi_read_adc_data(chan, &data);
		if (!ret)
			pru_add_data(data);
		else
			pru_add_data(0xffff);

	if (WINDEX >= WATERMARK)
		pru_adc_data_send(d);
}

static void pru_adc_cleanup(void)
{
	WINDEX /*= RINDEX */ = 0;
	return;
}

/*
 * main.c
 */
void main(void)
{
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;
	volatile uint8_t *status;

	/* Allow OCP master port access by the PRU so the PRU can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Initialize the RPMsg transport structure */
	pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

	/* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_PORT) != PRU_RPMSG_SUCCESS);

	g_data.transport = &transport;

//	spi_init();

	while (1) {
		/* Check bit 30 of register R31 to see if the ARM has kicked us */
		if (__R31 & HOST_INT) {
			static int foo = 0;
			/* Clear the event status */
			CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
			/* Receive all available messages, multiple messages can be sent per kick */
			while (pru_rpmsg_receive(g_data.transport, &src, &dst, payload_rx, &len) == PRU_RPMSG_SUCCESS) {
				bool ack = false;

				if (len >= 2) {
					if (payload_rx[0] == PRU_START_MSG &&
					    payload_rx[1] && g_data.state == PRU_IDLE) {
						unsigned int i, samplesize;
						uint8_t enabled = 0;

						g_data.measured_channels |= payload_rx[1];
						MEASURED_CHANS = payload_rx[1];

						for (i = 0; i < 8; i++)
							if ((1 << i) & payload_rx[1])
								enabled++;

						/*
						 * Calculate sample size and watermark
						 * position
						 */
						samplesize = sizeof(uint16_t) * enabled;
						/*
						 * Watermark position will be 'How many
						 * samples we can store * samplesize'.
						 */
						WATERMARK = MAX_ADC_DATA_BUF_IDX / samplesize * samplesize;

						g_data.state = PRU_STARTED;
						g_data.dst = src;
						g_data.src = dst;
						ack = true;

			/* Do SPI init here to be able to send debug RPMSGs */
				if (!foo) {
					//hiawathatest();
					spi_init();
				}

				foo++;
					//	read_test_mcspi_revision();
					} else if (payload_rx[0] == PRU_STOP_MSG && g_data.state == PRU_STARTED) {
						/*
						 * For now we don't allow changing channels when measurement is running. This simplifies the buffer handling as we always know which channels have data in the buffer.
						g_data.measured_channels &= (~payload_rx[1]);

						if (!g_data.measured_channels)
							g_data.state = PRU_STOPPED;
						*/
						MEASURED_CHANS = 0;
						g_data.state = PRU_STOPPED;
						ack = true;
					}
					else if (payload_rx[0] == PRU_ADC_DATA_FLUSH && g_data.state == PRU_STARTED)
						pru_adc_data_send(&g_data);
				}
	
				/* Echo the message back to the same address from which we just received */
				if (ack) {
					//int test;

//					for (test = 0; test < 10; test++) {
						//__delay_cycles(ONESEC_TEST_DELAY);
						pru_rpmsg_send(g_data.transport, g_data.src, g_data.dst, payload_rx, len);
//					}
				}
			}
		}
		if (g_data.state == PRU_STOPPED) {
			pru_adc_cleanup();
			g_data.state = PRU_IDLE;
		}
		if (g_data.state == PRU_STARTED)
			pru_adc_read(&g_data);
	}
}
