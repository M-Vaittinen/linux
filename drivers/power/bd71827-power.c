/*
 * bd71827-power.c
 * @file ROHM BD71827 Charger driver
 *
 * Copyright 2016.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

//#define DEBUG
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/mfd/rohm-bd71827.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define JITTER_DEFAULT			3000		/* hope 3s is enough */
#define JITTER_REPORT_CAP		10000		/* 10 seconds */
#define BATTERY_CAP_MAH_DEFAULT	910
#define MAX_VOLTAGE_DEFAULT		ocv_table_default[0]
#define MIN_VOLTAGE_DEFAULT		3400000
#define THR_VOLTAGE_DEFAULT		4100000
#define MAX_CURRENT_DEFAULT		890000		/* uA */
#define AC_NAME					"bd71827_ac"
#define BAT_NAME				"bd71827_bat"
#define BATTERY_FULL_DEFAULT	100

#define BY_BAT_VOLT				0
#define BY_VBATLOAD_REG			1
#define INIT_COULOMB			BY_VBATLOAD_REG

#define CALIB_CURRENT_A2A3		0xCE9E

//VBAT Low voltage detection Threshold 
#define VBAT_LOW_TH				0x00D4 // 0x00D4*16mV = 212*0.016 = 3.392v
#define RS_30mOHM
#ifdef RS_30mOHM
#define A10s_mAh(s)		((s) * 1000 / (360 * 3))
#define mAh_A10s(m)		((m) * (360 * 3) / 1000)
#else
#define A10s_mAh(s)		((s) * 1000 / 360)
#define mAh_A10s(m)		((m) * 360 / 1000)
#endif

#define THR_RELAX_CURRENT_DEFAULT	5		/* mA */
#define THR_RELAX_TIME_DEFAULT		(60 * 60)	/* sec. */

#define DGRD_CYC_CAP_DEFAULT		26	/* 1 micro Ah unit */

#define DGRD_TEMP_H_DEFAULT			45	/* 1 degrees C unit */
#define DGRD_TEMP_M_DEFAULT			25	/* 1 degrees C unit */
#define DGRD_TEMP_L_DEFAULT			5	/* 1 degrees C unit */
#define DGRD_TEMP_VL_DEFAULT		0	/* 1 degrees C unit */

#define SOC_EST_MAX_NUM_DEFAULT 	1
#define DGRD_TEMP_CAP_H_DEFAULT		(0)	/* 1 micro Ah unit */
#define DGRD_TEMP_CAP_M_DEFAULT		(1187)	/* 1 micro Ah unit */
#define DGRD_TEMP_CAP_L_DEFAULT		(5141)	/* 1 micro Ah unit */

#define PWRCTRL_NORMAL				0x22
#define PWRCTRL_RESET				0x23

static int ocv_table_default[23] = {
	4200000,
	4167456,
	4109781,
	4065242,
	4025618,
	3989877,
	3958031,
	3929302,
	3900935,
	3869637,
	3838475,
	3815196,
	3799778,
	3788385,
	3779627,
	3770675,
	3755368,
	3736049,
	3713545,
	3685118,
	3645278,
	3465599,
	2830610
};	/* unit 1 micro V */

static int soc_table_default[23] = {
	1000,
	1000,
	950,
	900,
	850,
	800,
	750,
	700,
	650,
	600,
	550,
	500,
	450,
	400,
	350,
	300,
	250,
	200,
	150,
	100,
	50,
	0,
	-50
	/* unit 0.1% */
};

static int vdr_table_h_default[23] = {
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100
};

static int vdr_table_m_default[23] = {
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100
};

static int vdr_table_l_default[23] = {
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100
};

static int vdr_table_vl_default[23] = {
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100
};

int use_load_bat_params;

int battery_cap_mah;
static int battery_cap;
int max_voltage;
int min_voltage;
int thr_voltage;
int max_current;
unsigned int battery_full;

unsigned int thr_relax_current;
unsigned int thr_relax_time;

int dgrd_cyc_cap;

int dgrd_temp_h;
int dgrd_temp_m;
int dgrd_temp_l;
int dgrd_temp_vl;

int soc_est_max_num;

int dgrd_temp_cap_h;
int dgrd_temp_cap_m;
int dgrd_temp_cap_l;

static unsigned int battery_cycle;

int ocv_table[23];
int soc_table[23];
int vdr_table_h[23];
int vdr_table_m[23];
int vdr_table_l[23];
int vdr_table_vl[23];

/** @brief power deivce */
struct bd71827_power {
	struct device *dev;
	struct bd71827 *mfd;			/**< parent for access register */
	struct power_supply *ac;			/**< alternating current power */
	struct power_supply *bat;		/**< battery power */
	struct delayed_work bd_work;		/**< delayed work for timed work */
	int gauge_delay;		/**< Schedule to call gauge algorithm */

	int	reg_index;			/**< register address saved for sysfs */

	int    vbus_status;			/**< last vbus status */
	int    charge_status;			/**< last charge status */
	int    bat_status;			/**< last bat status */

	int	hw_ocv1;			/**< HW ocv1 */
	int	hw_ocv2;			/**< HW ocv2 */
	int	bat_online;			/**< battery connect */
	int	charger_online;			/**< charger connect */
	int	vcell;				/**< battery voltage */
	int	vsys;				/**< system voltage */
	int	vcell_min;			/**< minimum battery voltage */
	int	vsys_min;			/**< minimum system voltage */
	int	rpt_status;			/**< battery status report */
	int	prev_rpt_status;		/**< previous battery status report */
	int	bat_health;			/**< battery health */
	int	designed_cap;			/**< battery designed capacity */
	int	full_cap;			/**< battery capacity */
	int	curr;				/**< battery current from DS-ADC */
	int	curr_sar;			/**< battery current from VM_IBAT */
	int	temp;				/**< battery tempature */
	u32	coulomb_cnt;			/**< Coulomb Counter */
	int	state_machine;			/**< initial-procedure state machine */

	u32	soc_org;			/**< State Of Charge using designed capacity without by load */
	u32	soc_norm;			/**< State Of Charge using full capacity without by load */
	u32	soc;				/**< State Of Charge using full capacity with by load */
	u32	clamp_soc;			/**< Clamped State Of Charge using full capacity with by load */

	int	relax_time;			/**< Relax Time */

	u32	cycle;				/**< Charging and Discharging cycle number */
	volatile int calib_current;		/**< calibration current */
};

struct bd71827 *pmic_data;

#define CALIB_NORM			0
#define CALIB_START			1
#define CALIB_GO			2

enum {
	STAT_POWER_ON,
	STAT_INITIALIZED,
};

static int bd71827_calc_soc_org(struct bd71827_power* pwr);

/** @brief read a register group once
 *  @param mfd bd71827 device
 *  @param reg	 register address of lower register
 *  @return register value
 */

u8 ext_bd71827_reg_read8(u8 reg)
{
	struct bd71827* mfd = pmic_data;
	u8 v;
	v = (u8)bd71827_reg_read(mfd, reg);

	return v;
}

int ext_bd71827_reg_write8(int reg, u8 val)
{
	struct bd71827* mfd = pmic_data;
	return bd71827_reg_write(mfd, reg, val);
}

/** @brief read a register group once
 *  @param mfd bd71827 device
 *  @param reg	 register address of lower register
 *  @return register value
 */
#ifdef __BD71827_REGMAP_H__
u16 ext_bd71827_reg_read16(int reg)
{
	struct bd71827* mfd = pmic_data;
	u16 v;

	v = (u16)bd71827_reg_read(mfd, reg) << 8;
	v |= (u16)bd71827_reg_read(mfd, reg + 1) << 0;
	return v;
}
#else
u16 ext_bd71827_reg_read16(int reg)
{
	struct bd71827* mfd = pmic_data;
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return be16_to_cpu(u.long_type);
}
#endif

/** @brief write a register group once
 * @param mfd bd71827 device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
int ext_bd71827_reg_write16(int reg, u16 val)
{
	struct bd71827* mfd = pmic_data;
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	u.long_type = cpu_to_be16(val);
	// printk("write16 0x%.4X 0x%.4X\n", val, u.long_type);
#ifdef __BD71827_REGMAP_H__
	r = mfd->write(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return 0;	
}

/** @brief read quad register once
 *  @param mfd bd71827 device
 *  @param reg	 register address of lower register
 *  @return register value
 */
int ext_bd71827_reg_read32(int reg)
{
	struct bd71827* mfd = pmic_data;
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

#ifdef __BD71827_REGMAP_H__
	r = mfd->read(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return be32_to_cpu(u.long_type);
}

#ifdef __BD71827_REGMAP_H__
static u16 bd71827_reg_read16(struct bd71827* mfd, int reg)
{
	u16 v;

	v = (u16)bd71827_reg_read(mfd, reg) << 8;
	v |= (u16)bd71827_reg_read(mfd, reg + 1) << 0;
	return v;
}
#else
static u16 bd71827_reg_read16(struct bd71827* mfd, int reg)
{
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return be16_to_cpu(u.long_type);
}
#endif

/** @brief write a register group once
 * @param mfd bd71827 device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd71827_reg_write16(struct bd71827 *mfd, int reg, u16 val)
{
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	u.long_type = cpu_to_be16(val);
	// printk("write16 0x%.4X 0x%.4X\n", val, u.long_type);
#ifdef __BD71827_REGMAP_H__
	r = mfd->write(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return 0;	
}

/** @brief read quad register once
 *  @param mfd bd71827 device
 *  @param reg	 register address of lower register
 *  @return register value
 */
static int bd71827_reg_read32(struct bd71827 *mfd, int reg)
{
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

#ifdef __BD71827_REGMAP_H__
	r = mfd->read(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return be32_to_cpu(u.long_type);
}

#if 0
/** @brief write quad register once
 * @param mfd bd71827 device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd71827_reg_write32(struct bd71827 *mfd, int reg, unsigned val)
{
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

	u.long_type = cpu_to_be32(val);
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return 0;
}
#endif

#if INIT_COULOMB == BY_VBATLOAD_REG
/** @brief get initial battery voltage and current
 * @param pwr power device
 * @return 0
 */
static int bd71827_get_init_bat_stat(struct bd71827_power *pwr)
{
	struct bd71827 *mfd = pwr->mfd;
	int vcell;

	vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_OCV_PRE_U) * 1000;
	dev_dbg(pwr->dev, "VM_OCV_PRE = %d\n", vcell);
	pwr->hw_ocv1 = vcell;

	vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_OCV_PST_U) * 1000;
	dev_dbg(pwr->dev, "VM_OCV_PST = %d\n", vcell);
	pwr->hw_ocv2 = vcell;

	return 0;
}
#endif

#if INIT_COULOMB == BY_BAT_VOLT
/** @brief get battery average voltage and current
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @param curr  pointer to return back current in unit uA.
 * @return 0
 */
static int bd71827_get_vbat_curr(struct bd71827_power *pwr, int *vcell, int *curr)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;

	tmp_vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VBAT_U);

	*vcell = tmp_vcell * 1000;
	*curr = 0;
	
	return 0;
}
#endif

/** @brief get battery average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd71827_get_vbat(struct bd71827_power *pwr, int *vcell)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;
	tmp_vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VBAT_U);

	*vcell = tmp_vcell * 1000;

	return 0;
}

/** @brief get battery current and battery average current from DS-ADC
 * @param pwr power device
 * @param current in unit uA
 * @param average current in unit uA
 * @return 0
 */
static int bd71827_get_current_ds_adc(struct bd71827_power *pwr, int *curr, int *curr_avg)
{
	int tmp_curr, tmp_curr_avg;

	tmp_curr = 0;
	tmp_curr_avg = 0;
	tmp_curr = bd71827_reg_read16(pwr->mfd, BD71827_REG_CC_CURCD_U);
	if (tmp_curr & CURDIR_Discharging) {
		tmp_curr = -(tmp_curr & ~CURDIR_Discharging);
	}
	*curr = tmp_curr * 1000;

	tmp_curr_avg = bd71827_reg_read16(pwr->mfd, BD71827_REG_CC_SA_CURCD_U);
	if (tmp_curr_avg & CURDIR_Discharging) {
		tmp_curr_avg = -(tmp_curr_avg & ~CURDIR_Discharging);
	}
	*curr_avg = tmp_curr_avg * 1000;

	return 0;
}

/** @brief get system average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd71827_get_vsys(struct bd71827_power *pwr, int *vsys)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vsys;

	tmp_vsys = 0;

	tmp_vsys = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VSYS_U);

	*vsys = tmp_vsys * 1000;

	return 0;
}

/** @brief get battery minimum average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd71827_get_vbat_min(struct bd71827_power *pwr, int *vcell)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;

	tmp_vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VBAT_MIN_U);
	bd71827_set_bits(pwr->mfd, BD71827_REG_VM_SA_MINMAX_CLR, VBAT_SA_MIN_CLR);

	*vcell = tmp_vcell * 1000;

	return 0;
}

/** @brief get system minimum average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd71827_get_vsys_min(struct bd71827_power *pwr, int *vcell)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;

	tmp_vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VSYS_MIN_U);
	bd71827_set_bits(pwr->mfd, BD71827_REG_VM_SA_MINMAX_CLR, VSYS_SA_MIN_CLR);

	*vcell = tmp_vcell * 1000;

	return 0;
}

/** @brief get battery capacity
 * @param ocv open circuit voltage
 * @return capcity in unit 0.1 percent
 */
static int bd71827_voltage_to_capacity(int ocv)
{
	int i = 0;
	int soc;

	if (ocv > ocv_table[0]) {
		soc = soc_table[0];
	} else {
		i = 0;
		while (soc_table[i] != -50) {
			if ((ocv <= ocv_table[i]) && (ocv > ocv_table[i+1])) {
				soc = (soc_table[i] - soc_table[i+1]) * (ocv - ocv_table[i+1]) / (ocv_table[i] - ocv_table[i+1]);
				soc += soc_table[i+1];
				break;
			}
			i++;
		}
		if (soc_table[i] == -50)
			soc = soc_table[i];
	}
	return soc;
}

/** @brief get battery temperature
 * @param pwr power device
 * @return temperature in unit deg.Celsius
 */
static int bd71827_get_temp(struct bd71827_power *pwr)
{
	struct bd71827* mfd = pwr->mfd;
	int t;

	t = 200 - (int)bd71827_reg_read(mfd, BD71827_REG_VM_BTMP);

	// battery temperature error
	t = (t > 200)? 200: t;
	
	return t;
}

static int bd71827_reset_coulomb_count(struct bd71827_power* pwr);

/** @brief get battery charge status
 * @param pwr power device
 * @return temperature in unit deg.Celsius
 */
static int bd71827_charge_status(struct bd71827_power *pwr)
{
	u8 state;
	int ret = 1;

	pwr->prev_rpt_status = pwr->rpt_status;

	state = bd71827_reg_read(pwr->mfd, BD71827_REG_CHG_STATE);
	dev_dbg(pwr->dev, "%s(): CHG_STATE %d\n", __func__, state);

	switch (state) {
	case 0x00:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x0E:
		pwr->rpt_status = POWER_SUPPLY_STATUS_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x0F:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_FULL;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x13:
	case 0x14:
	case 0x20:
	case 0x21:
	case 0x22:
	case 0x23:
	case 0x24:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case 0x30:
	case 0x31:
	case 0x32:
	case 0x40:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x7f:
	default:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_DEAD;
		break;	
	}

	bd71827_reset_coulomb_count(pwr);

	return ret;
}

#if INIT_COULOMB == BY_BAT_VOLT
static int bd71827_calib_voltage(struct bd71827_power* pwr, int* ocv)
{
	int r, curr, volt;

	bd71827_get_vbat_curr(pwr, &volt, &curr);

	r = bd71827_reg_read(pwr->mfd, BD71827_REG_CHG_STATE);
	if (r >= 0 && curr > 0) {
		// voltage increment caused by battery inner resistor
		if (r == 3) volt -= 100 * 1000;
		else if (r == 2) volt -= 50 * 1000;
	}
	*ocv = volt;

	return 0;
}
#endif

/** @brief set initial coulomb counter value from battery voltage
 * @param pwr power device
 * @return 0
 */
static int calibration_coulomb_counter(struct bd71827_power* pwr)
{
	u32 bcap;
	int soc, ocv;

#if INIT_COULOMB == BY_VBATLOAD_REG
	/* Get init OCV by HW */
	bd71827_get_init_bat_stat(pwr);

	ocv = (pwr->hw_ocv1 >= pwr->hw_ocv2)? pwr->hw_ocv1: pwr->hw_ocv2;
	dev_dbg(pwr->dev, "ocv %d\n", ocv);
#elif INIT_COULOMB == BY_BAT_VOLT
	bd71827_calib_voltage(pwr, &ocv);
#endif

	/* Get init soc from ocv/soc table */
	soc = bd71827_voltage_to_capacity(ocv);
	dev_dbg(pwr->dev, "soc %d[0.1%%]\n", soc);
	if (soc < 0)
		soc = 0;
	bcap = pwr->designed_cap * soc / 1000;

	bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
	bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, ((bcap + pwr->designed_cap / 200) & 0x0FFFUL));

	pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
	dev_dbg(pwr->dev, "%s() CC_CCNTD = %d\n", __func__, pwr->coulomb_cnt);

	/* Start canceling offset of the DS ADC. This needs 1 second at least */
	bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCCALIB);

	return 0;
}

/** @brief adjust coulomb counter values at relaxed state
 * @param pwr power device
 * @return 0
 */
static int bd71827_adjust_coulomb_count(struct bd71827_power* pwr)
{
	int relax_ocv=0;

	relax_ocv = bd71827_reg_read16(pwr->mfd, BD71827_REG_REX_SA_VBAT_U) * 1000;
	dev_dbg(pwr->dev,  "%s(): relax_ocv = 0x%x\n", __func__, relax_ocv);
	if (relax_ocv != 0) {
		u32 bcap;
		int soc;

		/* Clear Relaxed Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_REX_CTRL_1, REX_CLR);

		/* Get soc at relaxed state from ocv/soc table */
		soc = bd71827_voltage_to_capacity(relax_ocv);
		dev_dbg(pwr->dev,  "soc %d[0.1%%]\n", soc);
		if (soc < 0)
			soc = 0;

		bcap = pwr->designed_cap * soc / 1000;
		bcap = (bcap + pwr->designed_cap / 200) & 0x0FFFUL;

		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, bcap);

		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		dev_dbg(pwr->dev,  "Adjust Coulomb Counter at Relaxed State\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);
		dev_dbg(pwr->dev, "relaxed_ocv:%d, bcap:%d, soc:%d, coulomb_cnt:0x%d\n", relax_ocv, bcap, soc, pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		/* If the following commented out code is enabled, the SOC is not clamped at the relax time. */
		/* Reset SOCs */
		/* bd71827_calc_soc_org(pwr); */
		/* pwr->soc_norm = pwr->soc_org; */
		/* pwr->soc = pwr->soc_norm; */
		/* pwr->clamp_soc = pwr->soc; */
	}

	return 0;
}

/** @brief reset coulomb counter values at full charged state
 * @param pwr power device
 * @return 0
 */
static int bd71827_reset_coulomb_count(struct bd71827_power* pwr)
{
	u32 full_charged_coulomb_cnt;

	full_charged_coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_FULL_CCNTD_3) & 0x0FFFFFFFUL;
	dev_dbg(pwr->dev, "%s(): full_charged_coulomb_cnt=0x%x\n", __func__, full_charged_coulomb_cnt);
	if (full_charged_coulomb_cnt != 0) {
		int diff_coulomb_cnt;

		/* Clear Full Charged Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_FULL_CTRL, FULL_CLR);

		diff_coulomb_cnt = full_charged_coulomb_cnt - (bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL);
		diff_coulomb_cnt = diff_coulomb_cnt >> 16;
		if (diff_coulomb_cnt > 0) {
			diff_coulomb_cnt = 0;
		}
		dev_dbg(pwr->dev,  "diff_coulomb_cnt = %d\n", diff_coulomb_cnt);

		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, ((pwr->designed_cap + pwr->designed_cap / 200) & 0x0FFFUL) + diff_coulomb_cnt);

		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		dev_dbg(pwr->dev,  "Reset Coulomb Counter at POWER_SUPPLY_STATUS_FULL\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);
	}

	return 0;
}

/** @brief get battery parameters, such as voltages, currents, temperatures.
 * @param pwr power device
 * @return 0
 */
static int bd71827_get_voltage_current(struct bd71827_power* pwr)
{
	int p_id;

	p_id = bd71827_reg_read(pwr->mfd, BD71827_REG_PRODUCT) & PRODUCT_VERSION;

	if (p_id == 0) { /* BD7181x */
		/* Read detailed vcell and current */
	}
	else { /* BD7182x */
		/* Read detailed vcell and current */
		bd71827_get_vbat(pwr, &pwr->vcell);

		bd71827_get_current_ds_adc(pwr, &pwr->curr_sar, &pwr->curr);
	}
	/* Read detailed vsys */
	bd71827_get_vsys(pwr, &pwr->vsys);
	dev_dbg(pwr->dev,  "VM_VSYS = %d\n", pwr->vsys);

	/* Read detailed vbat_min */
	bd71827_get_vbat_min(pwr, &pwr->vcell_min);
	dev_dbg(pwr->dev,  "VM_VBAT_MIN = %d\n", pwr->vcell_min);

	/* Read detailed vsys_min */
	bd71827_get_vsys_min(pwr, &pwr->vsys_min);
	dev_dbg(pwr->dev,  "VM_VSYS_MIN = %d\n", pwr->vsys_min);

	/* Get tempature */
	pwr->temp = bd71827_get_temp(pwr);
	// dev_dbg(pwr->dev,  "Temperature %d degrees C\n", pwr->temp);

	return 0;
}

/** @brief adjust coulomb counter values at relaxed state by SW
 * @param pwr power device
 * @return 0
 */
static int bd71827_adjust_coulomb_count_sw(struct bd71827_power* pwr)
{
	int tmp_curr_mA;

	tmp_curr_mA = pwr->curr / 1000;
	if ((tmp_curr_mA * tmp_curr_mA) <= (thr_relax_current * thr_relax_current)) { /* No load */
		pwr->relax_time += (JITTER_DEFAULT / 1000);
	}
	else {
		pwr->relax_time = 0;
	}
	dev_dbg(pwr->dev,  "%s(): pwr->relax_time = 0x%x\n", __func__, pwr->relax_time);
	if (pwr->relax_time >= thr_relax_time) { /* Battery is relaxed. */
		u32 bcap;
		int soc, ocv;

		pwr->relax_time = 0;

		/* Get OCV */
		ocv = pwr->vcell;

		/* Get soc at relaxed state from ocv/soc table */
		soc = bd71827_voltage_to_capacity(ocv);
		dev_dbg(pwr->dev,  "soc %d[0.1%%]\n", soc);
		if (soc < 0)
			soc = 0;

		bcap = pwr->designed_cap * soc / 1000;

		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, ((bcap + pwr->designed_cap / 200) & 0x0FFFUL));

		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		dev_dbg(pwr->dev,  "Adjust Coulomb Counter by SW at Relaxed State\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		/* If the following commented out code is enabled, the SOC is not clamped at the relax time. */
		/* Reset SOCs */
		/* bd71827_calc_soc_org(pwr); */
		/* pwr->soc_norm = pwr->soc_org; */
		/* pwr->soc = pwr->soc_norm; */
		/* pwr->clamp_soc = pwr->soc; */
	}

	return 0;
}

/** @brief get coulomb counter values
 * @param pwr power device
 * @return 0
 */
static int bd71827_coulomb_count(struct bd71827_power* pwr)
{
	dev_dbg(pwr->dev, "%s(): pwr->state_machine = 0x%x\n", __func__, pwr->state_machine);
	if (pwr->state_machine == STAT_POWER_ON) {
		pwr->state_machine = STAT_INITIALIZED;
		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);
	} else if (pwr->state_machine == STAT_INITIALIZED) {
		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		// dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);
	}
	return 0;
}

/** @brief calc cycle
 * @param pwr power device
 * @return 0
 */
static int bd71827_update_cycle(struct bd71827_power* pwr)
{
	int charged_coulomb_cnt;

	charged_coulomb_cnt = bd71827_reg_read16(pwr->mfd, BD71827_REG_CCNTD_CHG_3);
	dev_dbg(pwr->dev, "%s(): charged_coulomb_cnt = 0x%x\n", __func__, charged_coulomb_cnt);
	if (charged_coulomb_cnt >= pwr->designed_cap) {
		pwr->cycle++;
		dev_dbg(pwr->dev,  "Update cycle = %d\n", pwr->cycle);
		battery_cycle = pwr->cycle;
		charged_coulomb_cnt -= pwr->designed_cap;
		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CCNTD_CHG_3, charged_coulomb_cnt);

		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);
	}
	return 0;
}

/** @brief calc full capacity value by Cycle and Temperature
 * @param pwr power device
 * @return 0
 */
static int bd71827_calc_full_cap(struct bd71827_power* pwr)
{
	u32 designed_cap_uAh;
	u32 full_cap_uAh;

	/* Calculate full capacity by cycle */
	designed_cap_uAh = A10s_mAh(pwr->designed_cap) * 1000;
	full_cap_uAh = designed_cap_uAh - dgrd_cyc_cap * pwr->cycle;
	pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	dev_dbg(pwr->dev,  "Calculate full capacity by cycle\n");
	dev_dbg(pwr->dev,  "%s() pwr->full_cap = %d\n", __func__, pwr->full_cap);

	/* Calculate full capacity by temperature */
	dev_dbg(pwr->dev,  "Temperature = %d\n", pwr->temp);
	if (pwr->temp >= dgrd_temp_m) {
		full_cap_uAh += (pwr->temp - dgrd_temp_m) * dgrd_temp_cap_h;
		pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	}
	else if (pwr->temp >= dgrd_temp_l) {
		full_cap_uAh += (pwr->temp - dgrd_temp_m) * dgrd_temp_cap_m;
		pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	}
	else {
		full_cap_uAh += (dgrd_temp_l - dgrd_temp_m) * dgrd_temp_cap_m;
		full_cap_uAh += (pwr->temp - dgrd_temp_l) * dgrd_temp_cap_l;
		pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	}
	
	if (pwr->full_cap < 1)
		pwr->full_cap = 1;
	
	dev_dbg(pwr->dev,  "Calculate full capacity by cycle and temperature\n");
	dev_dbg(pwr->dev,  "%s() pwr->full_cap = %d\n", __func__, pwr->full_cap);

	return 0;
}

/** @brief calculate SOC values by designed capacity
 * @param pwr power device
 * @return 0
 */
static int bd71827_calc_soc_org(struct bd71827_power* pwr)
{
	pwr->soc_org = (pwr->coulomb_cnt >> 16) * 100 /  pwr->designed_cap;
	if (pwr->soc_org > 100) {
		pwr->soc_org = 100;
		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, ((pwr->designed_cap + pwr->designed_cap / 200) & 0x0FFFUL));

		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		dev_dbg(pwr->dev,  "Limit Coulomb Counter\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);
	}
	dev_dbg(pwr->dev, "%s(): pwr->soc_org = %d\n", __func__, pwr->soc_org);
	return 0;
}

/** @brief calculate SOC values by full capacity
 * @param pwr power device
 * @return 0
 */
static int bd71827_calc_soc_norm(struct bd71827_power* pwr)
{
	int lost_cap;
	int mod_coulomb_cnt;

	lost_cap = pwr->designed_cap - pwr->full_cap;
	dev_dbg(pwr->dev,  "%s() lost_cap = %d\n", __func__, lost_cap);
	mod_coulomb_cnt = (pwr->coulomb_cnt >> 16) - lost_cap;
	if ((mod_coulomb_cnt > 0) && (pwr->full_cap > 0)) {
		pwr->soc_norm = mod_coulomb_cnt * 100 /  pwr->full_cap;
	}
	else {
		pwr->soc_norm = 0;
	}
	if (pwr->soc_norm > 100) {
		pwr->soc_norm = 100;
	}
		dev_dbg(pwr->dev,  "%s() pwr->soc_norm = %d\n", __func__, pwr->soc_norm);
	return 0;
}

/** @brief get OCV value by SOC
 * @param pwr power device
 * @return 0
 */
int bd71827_get_ocv(struct bd71827_power* pwr, int dsoc)
{
	int i = 0;
	int ocv = 0;

	if (dsoc > soc_table[0]) {
		ocv = max_voltage;
	}
	else if (dsoc == 0) {
			ocv = ocv_table[21];
	}
	else {
		i = 0;
		while (i < 22) {
			if ((dsoc <= soc_table[i]) && (dsoc > soc_table[i+1])) {
				ocv = (ocv_table[i] - ocv_table[i+1]) * (dsoc - soc_table[i+1]) / (soc_table[i] - soc_table[i+1]) + ocv_table[i+1];
				break;
			}
			i++;
		}
		if (i == 22)
			ocv = ocv_table[22];
	}
	dev_dbg(pwr->dev,  "%s() ocv = %d\n", __func__, ocv);
	return ocv;
}


/** @brief get VDR(Voltage Drop Rate) value by SOC
 * @param pwr power device
 * @return 0
 */
static int bd71827_get_vdr(struct bd71827_power* pwr, int dsoc)
{
	int i = 0;
	int vdr = 100;
	int vdr_table[23];

	/* Calculate VDR by temperature */
	if (pwr->temp >= dgrd_temp_h) {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_h[i];
		}
	}
	else if (pwr->temp >= dgrd_temp_m) {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_m[i] + 
			(pwr->temp - dgrd_temp_m) * (vdr_table_h[i] - vdr_table_m[i]) / (dgrd_temp_h - dgrd_temp_m);
		}
	}
	else if (pwr->temp >= dgrd_temp_l) {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_l[i] + 
			(pwr->temp - dgrd_temp_l) * (vdr_table_m[i] - vdr_table_l[i]) / (dgrd_temp_m - dgrd_temp_l);
		}
	}
	else if (pwr->temp >= dgrd_temp_vl) {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_vl[i] + 
			(pwr->temp - dgrd_temp_vl) * (vdr_table_l[i] - vdr_table_vl[i]) / (dgrd_temp_l - dgrd_temp_vl);
		}
	}
	else {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_vl[i];
		}
	}

	if (dsoc > soc_table[0]) {
		vdr = 100;
	}
	else if (dsoc == 0) {
		vdr = vdr_table[21];
	}
	else {
		i = 0;
		while (i < 22) {
			if ((dsoc <= soc_table[i]) && (dsoc > soc_table[i+1])) {
				vdr = (vdr_table[i] - vdr_table[i+1]) * (dsoc - soc_table[i+1]) / (soc_table[i] - soc_table[i+1]) + vdr_table[i+1];
				break;
			}
			i++;
		}
		if (i == 22)
			vdr = vdr_table[22];
	}
	dev_dbg(pwr->dev, "%s() vdr = %d\n", __func__, vdr);
	return vdr;
}

/** @brief calculate SOC value by full_capacity and load
 * @param pwr power device
 * @return OCV
 */
static int bd71827_calc_soc(struct bd71827_power* pwr)
{
	int ocv_table_load[23];

	pwr->soc = pwr->soc_norm;

	switch (pwr->rpt_status) { /* Adjust for 0% between thr_voltage and min_voltage */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->vsys_min <= thr_voltage) {
			int i;
			int ocv;
			int lost_cap;
			int mod_coulomb_cnt;
			int dsoc;

			lost_cap = pwr->designed_cap - pwr->full_cap;
			mod_coulomb_cnt = (pwr->coulomb_cnt >> 16) - lost_cap;
			dsoc = mod_coulomb_cnt * 1000 /  pwr->full_cap;
			dev_dbg(pwr->dev,  "%s() dsoc = %d\n", __func__, dsoc);
			ocv = bd71827_get_ocv(pwr, dsoc);
			for (i = 1; i < 23; i++) {
				ocv_table_load[i] = ocv_table[i] - (ocv - pwr->vsys_min);
				if (ocv_table_load[i] <= min_voltage) {
					dev_dbg(pwr->dev,  "%s() ocv_table_load[%d] = %d\n", __func__, i, ocv_table_load[i]);
					break;
				}
			}
			if (i < 23) {
				int j, k, m;
				int dv;
				int lost_cap2, new_lost_cap2;
				int mod_coulomb_cnt2, mod_full_cap;
				int dsoc0;
				int vdr, vdr0;
				dv = (ocv_table_load[i-1] - ocv_table_load[i]) / 5;
				for (j = 1; j < 5; j++){
					if ((ocv_table_load[i] + dv * j) > min_voltage) {
						break;
					}
				}
				lost_cap2 = ((21 - i) * 5 + (j - 1)) * pwr->full_cap / 100;
				dev_dbg(pwr->dev, "%s() lost_cap2-1 = %d\n", __func__, lost_cap2);
				for (m = 0; m < soc_est_max_num; m++) {
					new_lost_cap2 = lost_cap2;
				dsoc0 = lost_cap2 * 1000 / pwr->full_cap;
				if (dsoc >= 0) {
					if (dsoc0 > dsoc)
						dsoc0 = dsoc;
				}
				else {
					if (dsoc0 < dsoc)
						dsoc0 = dsoc;
				}
				dev_dbg(pwr->dev, "%s() dsoc0(%d) = %d\n", __func__, m, dsoc0);

				vdr = bd71827_get_vdr(pwr, dsoc);
				vdr0 = bd71827_get_vdr(pwr, dsoc0);

				for (k = 1; k < 23; k++) {
					ocv_table_load[k] = ocv_table[k] - (ocv - pwr->vsys_min) * vdr0 / vdr;
					if (ocv_table_load[k] <= min_voltage) {
						dev_dbg(pwr->dev, "%s() ocv_table_load[%d] = %d\n", __func__, k, ocv_table_load[k]);
						break;
					}
				}
				if (k < 23) {
					dv = (ocv_table_load[k-1] - ocv_table_load[k]) / 5;
					for (j = 1; j < 5; j++){
						if ((ocv_table_load[k] + dv * j) > min_voltage) {
							break;
						}
					}
						new_lost_cap2 = ((21 - k) * 5 + (j - 1)) * pwr->full_cap / 100;
						if (soc_est_max_num == 1) {
						lost_cap2 = new_lost_cap2;
						}
						else {
							lost_cap2 += (new_lost_cap2 - lost_cap2) / (2 * (soc_est_max_num - m));
						}
						dev_dbg(pwr->dev, "%s() lost_cap2-2(%d) = %d\n", __func__, m, lost_cap2);
				}
					if (new_lost_cap2 == lost_cap2) {
						break;
					}
				}
				mod_coulomb_cnt2 = mod_coulomb_cnt - lost_cap2;
				mod_full_cap = pwr->full_cap - lost_cap2;
				if ((mod_coulomb_cnt2 > 0) && (mod_full_cap > 0)) {
					pwr->soc = mod_coulomb_cnt2 * 100 / mod_full_cap;
				}
				else {
					pwr->soc = 0;
				}
				dev_dbg(pwr->dev,  "%s() pwr->soc(by load) = %d\n", __func__, pwr->soc);
			}
		}
		break;
	default:
		break;
	}

	switch (pwr->rpt_status) {/* Adjust for 0% and 100% */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->vsys_min <= min_voltage) {
			pwr->soc = 0;
		}
		else {
			if (pwr->soc == 0) {
				pwr->soc = 1;
			}
		}
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		if (pwr->soc == 100) {
			pwr->soc = 99;
		}
		break;
	default:
		break;
	}
	dev_dbg(pwr->dev,  "%s() pwr->soc = %d\n", __func__, pwr->soc);
	return 0;
}

/** @brief calculate Clamped SOC value by full_capacity and load
 * @param pwr power device
 * @return OCV
 */
static int bd71827_calc_soc_clamp(struct bd71827_power* pwr)
{
	switch (pwr->rpt_status) {/* Adjust for 0% and 100% */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->soc <= pwr->clamp_soc) {
			pwr->clamp_soc = pwr->soc;
		}
		break;
	default:
		pwr->clamp_soc = pwr->soc;
		break;
	}
	dev_dbg(pwr->dev,  "%s() pwr->clamp_soc = %d\n", __func__, pwr->clamp_soc);
	return 0;
}

/** @brief get battery and DC online status
 * @param pwr power device
 * @return 0
 */
static int bd71827_get_online(struct bd71827_power* pwr)
{
	int r;

#if 0
#define TS_THRESHOLD_VOLT	0xD9
	r = bd71827_reg_read(pwr->mfd, BD71827_REG_VM_VTH);
	pwr->bat_online = (r > TS_THRESHOLD_VOLT);
#endif
#if 0
	r = bd71827_reg_read(pwr->mfd, BD71827_REG_BAT_STAT);
	if (r >= 0 && (r & BAT_DET_DONE)) {
		pwr->bat_online = (r & BAT_DET) != 0;
	}
#endif
#if 1
#define BAT_OPEN	0x7
	r = bd71827_reg_read(pwr->mfd, BD71827_REG_BAT_TEMP);
	pwr->bat_online = (r != BAT_OPEN);
#endif	
	r = bd71827_reg_read(pwr->mfd, BD71827_REG_DCIN_STAT);
	if (r >= 0) {
		pwr->charger_online = (r & DCIN_DET) != 0;
	}
	dev_dbg(pwr->dev, "%s(): pwr->bat_online = %d, pwr->charger_online = %d\n", __func__, pwr->bat_online, pwr->charger_online);
	return 0;
}

/** @brief init bd71827 sub module charger
 * @param pwr power device
 * @return 0
 */
static int bd71827_init_hardware(struct bd71827_power *pwr)
{
	struct bd71827 *mfd = pwr->mfd;
	int r;

	r = bd71827_reg_write(mfd, BD71827_REG_DCIN_CLPS, 0x36);

#define XSTB		0x02
	r = bd71827_reg_read(mfd, BD71827_REG_CONF);

#if 0
	for (i = 0; i < 300; i++) {
		r = bd71827_reg_read(pwr->mfd, BD71827_REG_BAT_STAT);
		if (r >= 0 && (r & BAT_DET_DONE)) {
			break;
		}
		msleep(5);
	}
#endif
	if ((r & XSTB) == 0x00) {
	//if (r & BAT_DET) {
		/* Init HW, when the battery is inserted. */

		bd71827_reg_write(mfd, BD71827_REG_CONF, r | XSTB);

#define TEST_SEQ_00		0x00
#define TEST_SEQ_01		0x76
#define TEST_SEQ_02		0x66
#define TEST_SEQ_03		0x56
#if 0
		bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_01);
		bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_02);
		bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_03);
		bd71827_reg_write16(pwr->mfd, 0xA2, CALIB_CURRENT_A2A3);
		bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_00);
#endif

		/* Stop Coulomb Counter */
		bd71827_clear_bits(mfd, BD71827_REG_CC_CTRL, CCNTENB);

		/* Set Coulomb Counter Reset bit*/
		bd71827_set_bits(mfd, BD71827_REG_CC_CTRL, CCNTRST);

		/* Clear Coulomb Counter Reset bit*/
		bd71827_clear_bits(mfd, BD71827_REG_CC_CTRL, CCNTRST);

		/* Clear Relaxed Coulomb Counter */
		bd71827_set_bits(mfd, BD71827_REG_REX_CTRL_1, REX_CLR);

		/* Set default Battery Capacity */
		pwr->designed_cap = battery_cap;
		pwr->full_cap = battery_cap;

		/* Set initial Coulomb Counter by HW OCV */
		calibration_coulomb_counter(pwr);

		/* WDT_FST auto set */
		bd71827_set_bits(mfd, BD71827_REG_CHG_SET1, WDT_AUTO);

		/* VBAT Low voltage detection Setting, added by John Zhang*/
		bd71827_reg_write16(mfd, BD71827_REG_ALM_VBAT_TH_U, VBAT_LOW_TH);

		/* Set Battery Capacity Monitor threshold1 as 90% */
		bd71827_reg_write16(mfd, BD71827_REG_CC_BATCAP1_TH_U, (battery_cap * 9 / 10)); 
		dev_dbg(pwr->dev, "BD71827_REG_CC_BATCAP1_TH = %d\n", (battery_cap * 9 / 10));

		/* Enable LED ON when charging */
		bd71827_set_bits(pwr->mfd, BD71827_REG_LED_CTRL, CHGDONE_LED_EN);

		pwr->state_machine = STAT_POWER_ON;
	} else {
		pwr->designed_cap = battery_cap;
		pwr->full_cap = battery_cap;	// bd71827_reg_read16(pwr->mfd, BD71827_REG_CC_BATCAP_U);
		pwr->state_machine = STAT_INITIALIZED;	// STAT_INITIALIZED
	}

	pwr->temp = bd71827_get_temp(pwr);
	dev_dbg(pwr->dev,  "Temperature = %d\n", pwr->temp);
	bd71827_adjust_coulomb_count(pwr);
	bd71827_reset_coulomb_count(pwr);
	pwr->coulomb_cnt = bd71827_reg_read32(mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
	bd71827_calc_soc_org(pwr);
	pwr->soc_norm = pwr->soc_org;
	pwr->soc = pwr->soc_norm;
	pwr->clamp_soc = pwr->soc;
	dev_dbg(pwr->dev,  "%s() CC_CCNTD = %d\n", __func__, pwr->coulomb_cnt);
	dev_dbg(pwr->dev,  "%s() pwr->soc = %d\n", __func__, pwr->soc);
	dev_dbg(pwr->dev,  "%s() pwr->clamp_soc = %d\n", __func__, pwr->clamp_soc);

	pwr->cycle = battery_cycle;
	pwr->curr = 0;
	pwr->relax_time = 0;

	return 0;
}

/** @brief set bd71827 battery parameters
 * @param pwr power device
 * @return 0
 */
static int bd71827_set_battery_parameters(struct bd71827_power *pwr)
{
	//struct bd71827 *mfd = pwr->mfd;
	int i;

	if (use_load_bat_params == 0) {
		battery_cap_mah = BATTERY_CAP_MAH_DEFAULT;
		dgrd_cyc_cap = DGRD_CYC_CAP_DEFAULT;
		soc_est_max_num = SOC_EST_MAX_NUM_DEFAULT;
		dgrd_temp_cap_h = DGRD_TEMP_CAP_H_DEFAULT;
		dgrd_temp_cap_m = DGRD_TEMP_CAP_M_DEFAULT;
		dgrd_temp_cap_l = DGRD_TEMP_CAP_L_DEFAULT;
		for (i = 0; i < 23; i++) {
			ocv_table[i] = ocv_table_default[i];
			soc_table[i] = soc_table_default[i];
			vdr_table_h[i] = vdr_table_h_default[i];
			vdr_table_m[i] = vdr_table_m_default[i];
			vdr_table_l[i] = vdr_table_l_default[i];
			vdr_table_vl[i] = vdr_table_vl_default[i];
		}
	}
	max_voltage = MAX_VOLTAGE_DEFAULT;
	min_voltage = MIN_VOLTAGE_DEFAULT;
	thr_voltage = THR_VOLTAGE_DEFAULT;
	max_current = MAX_CURRENT_DEFAULT;
	battery_full = BATTERY_FULL_DEFAULT;
	thr_relax_current = THR_RELAX_CURRENT_DEFAULT;
	thr_relax_time = THR_RELAX_TIME_DEFAULT;
	dgrd_temp_h = DGRD_TEMP_H_DEFAULT;
	dgrd_temp_m = DGRD_TEMP_M_DEFAULT;
	dgrd_temp_l = DGRD_TEMP_L_DEFAULT;
	dgrd_temp_vl = DGRD_TEMP_VL_DEFAULT;
	for (i = 0; i < 23; i++) {
		soc_table[i] = soc_table_default[i];
	}
	battery_cap = mAh_A10s(battery_cap_mah);

	return 0;
}

/**@brief timed work function called by system
 *  read battery capacity,
 *  sense change of charge status, etc.
 * @param work work struct
 * @return  void
 */

static void bd_work_callback(struct work_struct *work)
{
	struct bd71827_power *pwr;
	struct delayed_work *delayed_work;
	int status, changed = 0;
	static int cap_counter = 0;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd71827_power, bd_work);

	dev_dbg(pwr->dev, "%s(): in\n", __func__);
	status = bd71827_reg_read(pwr->mfd, BD71827_REG_DCIN_STAT);
	if (status != pwr->vbus_status) {
    	dev_dbg(pwr->dev,"DCIN_STAT CHANGED from 0x%X to 0x%X\n", pwr->vbus_status, status);
		pwr->vbus_status = status;
		changed = 1;
	}

	status = bd71827_reg_read(pwr->mfd, BD71827_REG_BAT_STAT);
	status &= ~BAT_DET_DONE;
	if (status != pwr->bat_status) {
		dev_dbg(pwr->dev, "BAT_STAT CHANGED from 0x%X to 0x%X\n", pwr->bat_status, status);
		pwr->bat_status = status;
		changed = 1;
	}

	status = bd71827_reg_read(pwr->mfd, BD71827_REG_CHG_STATE);
	if (status != pwr->charge_status) {
		dev_dbg(pwr->dev, "CHG_STATE CHANGED from 0x%X to 0x%X\n", pwr->charge_status, status);
		pwr->charge_status = status;
		//changed = 1;
	}

	bd71827_get_voltage_current(pwr);
	bd71827_adjust_coulomb_count(pwr);
	bd71827_reset_coulomb_count(pwr);
	bd71827_adjust_coulomb_count_sw(pwr);
	bd71827_coulomb_count(pwr);
	bd71827_update_cycle(pwr);
	bd71827_calc_full_cap(pwr);
	bd71827_calc_soc_org(pwr);
	bd71827_calc_soc_norm(pwr);
	bd71827_calc_soc(pwr);
	bd71827_calc_soc_clamp(pwr);
	bd71827_get_online(pwr);
	bd71827_charge_status(pwr);

	if (changed || cap_counter++ > JITTER_REPORT_CAP / JITTER_DEFAULT) {
		power_supply_changed(pwr->ac);
		power_supply_changed(pwr->bat);
		cap_counter = 0;
	}

	if (pwr->calib_current == CALIB_NORM) {
		pwr->gauge_delay = JITTER_DEFAULT;
		schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(JITTER_DEFAULT));
	} else if (pwr->calib_current == CALIB_START) {
		pwr->calib_current = CALIB_GO;
	}
}

/**@brief bd71827 power interrupt
 * @param irq system irq
 * @param pwrsys bd71827 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 */
static irqreturn_t bd71827_power_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	// struct bd71827_power *pwr = dev_get_drvdata(dev);
	int reg, r;

	dev_info(mfd->dev, "bd71827_power_interrupt() in.\n");
	
	reg = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_03);
	if (reg < 0)
		return IRQ_NONE;

	if(reg & POWERON_PRESS)
	{
		kobject_uevent(&(mfd->dev->kobj), KOBJ_ONLINE); 
		dev_info(mfd->dev, "POWERON_PRESS\n");
	}
	if(reg & POWERON_SHORT)
	{
		kobject_uevent(&(mfd->dev->kobj), KOBJ_OFFLINE);
		dev_info(mfd->dev, "POWERON_SHORT\n");
	}
	if(reg & POWERON_MID)
	{
		kobject_uevent(&(mfd->dev->kobj), KOBJ_OFFLINE); 
		dev_info(mfd->dev, "POWERON_MID\n");
	}
	if(reg & POWERON_LONG)
	{
		kobject_uevent(&(mfd->dev->kobj), KOBJ_OFFLINE);
		dev_info(mfd->dev, "POWERON_LONG\n");
	}

	r = bd71827_reg_write(mfd, BD71827_REG_INT_STAT_03, reg);
	if (r)
		return IRQ_NONE;

	if (reg & DCIN_MON_DET) {
		dev_info(mfd->dev, "\n~~~DCIN removed\n");
	} else if (reg & DCIN_MON_RES) {
		dev_info(mfd->dev, "\n~~~DCIN inserted\n");
	}

	return IRQ_HANDLED;
}

/**@brief bd71827 vbat low voltage detection interrupt
 * @param irq system irq
 * @param pwrsys bd71827 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 * added by John Zhang at 2015-07-22
 */
static irqreturn_t bd71827_vbat_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	// struct bd71827_power *pwr = dev_get_drvdata(dev);
	int reg, r;

	dev_info(mfd->dev, "bd71827_vbat_interrupt() in.\n");
	
	reg = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_08);
	if (reg < 0)
		return IRQ_NONE;

	dev_info(mfd->dev, "INT_STAT_08 = 0x%.2X\n", reg);

	r = bd71827_reg_write(mfd, BD71827_REG_INT_STAT_08, reg);
	if (r)
		return IRQ_NONE;

	if (reg & VBAT_MON_DET) {
		dev_info(mfd->dev, "\n~~~ VBAT LOW Detected ... \n");
		
	} else if (reg & VBAT_MON_RES) {
		dev_info(mfd->dev, "\n~~~ VBAT LOW Resumed ... \n");
	}

	return IRQ_HANDLED;
	
}

/**@brief bd71827 int_stat_11 detection interrupt
 * @param irq system irq
 * @param pwrsys bd71827 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 * added 2015-12-26
 */
static irqreturn_t bd71827_int_11_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	// struct bd71827_power *pwr = dev_get_drvdata(dev);
	int reg, r;
	
	dev_info(mfd->dev, "bd71827_int_11_interrupt() in.\n");
	
	reg = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_11);
	if (reg < 0)
		return IRQ_NONE;

	dev_info(mfd->dev, "INT_STAT_11 = 0x%.2X\n", reg);

	r = bd71827_reg_write(mfd, BD71827_REG_INT_STAT_11, reg);
	if (r) {
		return IRQ_NONE;
	}

	if (reg & INT_STAT_11_VF_DET) {
		dev_info(mfd->dev, "\n~~~ VF Detected ... \n");
	} else if (reg & INT_STAT_11_VF_RES) {
		dev_info(mfd->dev, "\n~~~ VF Resumed ... \n");
	} else if (reg & INT_STAT_11_VF125_DET) {
		dev_info(mfd->dev, "\n~~~ VF125 Detected ... \n");
	} else if (reg & INT_STAT_11_VF125_RES) {
		dev_info(mfd->dev, "\n~~~ VF125 Resumed ... \n");
	} else if (reg & INT_STAT_11_OVTMP_DET) {
		dev_info(mfd->dev, "\n~~~ Overtemp Detected ... \n");
	} else if (reg & INT_STAT_11_OVTMP_RES) {
		dev_info(mfd->dev, "\n~~~ Overtemp Detected ... \n");
	} else if (reg & INT_STAT_11_LOTMP_DET) {
		dev_info(mfd->dev, "\n~~~ Lowtemp Detected ... \n");
	} else if (reg & INT_STAT_11_LOTMP_RES) {
		dev_info(mfd->dev, "\n~~~ Lowtemp Detected ... \n");
	}

	return IRQ_HANDLED;

}

/** @brief get property of power supply ac
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */
static int bd71827_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd71827_power *pwr = dev_get_drvdata(psy->dev.parent);
	u32 vot;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pwr->charger_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd71827_reg_read16(pwr->mfd, BD71827_REG_VM_DCIN_U);
		val->intval = 5000 * vot;		// 5 milli volt steps
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/** @brief get property of power supply bat
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */

static int bd71827_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd71827_power *pwr = dev_get_drvdata(psy->dev.parent);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pwr->rpt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = pwr->bat_health;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (pwr->rpt_status == POWER_SUPPLY_STATUS_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pwr->bat_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pwr->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = pwr->clamp_soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		{
		u32 t;

		t = pwr->coulomb_cnt >> 16;
		t = A10s_mAh(t);
		if (t > A10s_mAh(pwr->designed_cap)) t = A10s_mAh(pwr->designed_cap);
		val->intval = t * 1000;		/* uA to report */
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pwr->bat_online;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = battery_full * A10s_mAh(pwr->designed_cap) * 10;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = battery_full * A10s_mAh(pwr->full_cap) * 10;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = pwr->curr;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = pwr->temp * 10; /* 0.1 degrees C unit */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = max_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = min_voltage;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = max_current;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/** @brief ac properties */
static enum power_supply_property bd71827_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/** @brief bat properies */
static enum power_supply_property bd71827_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

/** @brief directly set raw value to chip register, format: 'register value' */
static ssize_t bd71827_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret < 1) {
		pwr->reg_index = -1;
		dev_err(pwr->dev, "registers set: <reg> <value>\n");
		return count;
	}

	if (ret == 1 && reg <= BD71827_MAX_REGISTER) {
		pwr->reg_index = reg;
		dev_dbg(pwr->dev, "registers set: reg=0x%x\n", reg);
		return count;
	}

	if (reg > BD71827_MAX_REGISTER || val > 255)
		return -EINVAL;

	dev_dbg(pwr->dev, "registers set: reg=0x%x, val=0x%x\n", reg, val);
	ret = bd71827_reg_write(pwr->mfd, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/** @brief print value of chip register, format: 'register=value' */
static ssize_t bd71827_sysfs_print_reg(struct bd71827_power *pwr,
				       u8 reg,
				       char *buf)
{
	int ret = bd71827_reg_read(pwr->mfd, reg);

	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "[0x%.2X] = %.2X\n", reg, ret);
}

/** @brief show all raw values of chip register, format per line: 'register=value' */
static ssize_t bd71827_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	int i;

	dev_dbg(pwr->dev, "register: index[%d]\n", pwr->reg_index);
	if (pwr->reg_index >= 0) {
		ret += bd71827_sysfs_print_reg(pwr, pwr->reg_index, buf + ret);
	} else {
		for (i = 0; i < BD71827_MAX_REGISTER; i++) {
			ret += bd71827_sysfs_print_reg(pwr, i, buf + ret);
		}
	}
	return ret;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bd71827_sysfs_show_registers, bd71827_sysfs_set_registers);

/** @brief directly set charging status, set 1 to enable charging, set 0 to disable charging */
static ssize_t bd71827_sysfs_set_charging(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	//unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x", &val);
	printk(KERN_ERR "%s val=%x\n",__FUNCTION__, val);
	if (ret < 1) {
		return count;
	}

	if (ret == 1 && val >1) {
		return count;
	}

	if(val == 1)
		bd71827_set_bits(pwr->mfd, BD71827_REG_CHG_SET1, CHG_EN);
	else
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CHG_SET1, CHG_EN);

	return count;

}
/** @brief show charging status' */
static ssize_t bd71827_sysfs_show_charging(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	int reg_value=0;
	//ssize_t ret = 0;
	//unsigned int reg;

	reg_value = bd71827_reg_read(pwr->mfd, BD71827_REG_CHG_SET1);

//	printk(KERN_ERR "%s charger_online:%x, reg_value:%x\n",__FUNCTION__, pwr->charger_online, reg_value);
	return sprintf(buf, "%x\n", pwr->charger_online && reg_value & CHG_EN);
}

static DEVICE_ATTR(charging, S_IWUSR | S_IRUGO,
		bd71827_sysfs_show_charging, bd71827_sysfs_set_charging);

static int first_offset(struct bd71827_power *pwr)
{
	unsigned char ra2, ra3, ra6, ra7;
	unsigned char ra2_temp;
	struct bd71827 *mfd = pwr->mfd;

	bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_01);
	bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_02);
	bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_03);


	ra2 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_11);	// I want to know initial A2 & A3.
	ra3 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_12);	// I want to know initial A2 & A3.
	ra6 = bd71827_reg_read(mfd, 0xA6);
	ra7 = bd71827_reg_read(mfd, 0xA7);

	bd71827_reg_write(mfd, BD71827_REG_INT_STAT_11, 0x00);
	bd71827_reg_write(mfd, BD71827_REG_INT_STAT_12, 0x00);

	dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2);
	dev_info(pwr->dev, "TEST[A3] = 0x%.2X\n", ra3);
	dev_info(pwr->dev, "TEST[A6] = 0x%.2X\n", ra6);
	dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);

	//-------------- First Step -------------------
	dev_info(pwr->dev, "Frist Step begginning \n");

	// delay some time , Make a state of IBAT=0mA
	// mdelay(1000 * 10);

	ra2_temp = ra2;

	if (ra7 != 0) {
		//if 0<0xA7<20 decrease the Test register 0xA2[7:3] until 0xA7 becomes 0x00.
		if ((ra7 > 0) && (ra7 < 20)) {
			do {
				ra2 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_11);
				ra2_temp = ra2 >> 3;
				ra2_temp -= 1;
				ra2_temp <<= 3;
				bd71827_reg_write(mfd, BD71827_REG_INT_STAT_11, ra2_temp);
				dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2_temp);

				ra7 = bd71827_reg_read(mfd, 0xA7);
				dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);
				mdelay(1000);	// 1sec?
			} while (ra7);

			dev_info(pwr->dev, "A7 becomes 0 . \n");

		}		// end if((ra7 > 0)&&(ra7 < 20)) 
		else if ((ra7 > 0xDF) && (ra7 < 0xFF))
			//if DF<0xA7<FF increase the Test register 0xA2[7:3] until 0xA7 becomes 0x00.
		{
			do {
				ra2 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_11);
				ra2_temp = ra2 >> 3;
				ra2_temp += 1;
				ra2_temp <<= 3;

				bd71827_reg_write(mfd, BD71827_REG_INT_STAT_11, ra2_temp);
				dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2_temp);

				ra7 = bd71827_reg_read(mfd, 0xA7);
				dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);
				mdelay(1000);	// 1sec?                           
			} while (ra7);

			dev_info(pwr->dev, "A7 becomes 0 . \n");
		}
	}

	// please use "ra2_temp" at step2.
	return ra2_temp;
}

static int second_step(struct bd71827_power *pwr, u8 ra2_temp)
{
	u16 ra6, ra7;
	u8 aft_ra2, aft_ra3;
	u8 r79, r7a;
	unsigned int LNRDSA_FUSE;
	long ADC_SIGN;
	long DSADGAIN1_INI;
	struct bd71827 *mfd = pwr->mfd;

	//-------------- Second Step -------------------
	dev_info(pwr->dev, "Second Step begginning \n");

	// need to change boad setting ( input 1A tio 10mohm)
	// delay some time , Make a state of IBAT=1000mA
	// mdelay(1000 * 10);

// rough adjust
	dev_info(pwr->dev, "ra2_temp = 0x%.2X\n", ra2_temp);

	ra6 = bd71827_reg_read(mfd, 0xA6);
	ra7 = bd71827_reg_read(mfd, 0xA7);
	ra6 <<= 8;
	ra6 |= ra7;		// [0xA6 0xA7]
	dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);

	bd71827_reg_write(mfd, BD71827_REG_INT_STAT_11, ra2_temp);	// this value from step1
	bd71827_reg_write(mfd, BD71827_REG_INT_STAT_12, 0x00);

	bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_00);

	r79 = bd71827_reg_read(mfd, 0x79);
	r7a = bd71827_reg_read(mfd, 0x7A);

	ADC_SIGN = r79 >> 7;
	ADC_SIGN = 1 - (2 * ADC_SIGN);
	DSADGAIN1_INI = r79 << 8;
	DSADGAIN1_INI = DSADGAIN1_INI + r7a;
	DSADGAIN1_INI = DSADGAIN1_INI & 0x7FFF;
	DSADGAIN1_INI = DSADGAIN1_INI * ADC_SIGN; //  unit 0.001

	// unit 0.000001
	DSADGAIN1_INI *= 1000;
	{
	if (DSADGAIN1_INI > 1000001) {
		DSADGAIN1_INI = 2048000000UL - (DSADGAIN1_INI - 1000000) * 8187;
	} else if (DSADGAIN1_INI < 999999) {
		DSADGAIN1_INI = -(DSADGAIN1_INI - 1000000) * 8187;
	} else {
		DSADGAIN1_INI = 0;
	}
	}

	LNRDSA_FUSE = (int) DSADGAIN1_INI / 1000000;

	dev_info(pwr->dev, "LNRDSA_FUSE = 0x%.8X\n", LNRDSA_FUSE);

	aft_ra2 = (LNRDSA_FUSE >> 8) & 255;
	aft_ra3 = (LNRDSA_FUSE) & 255;

	aft_ra2 = aft_ra2 + ra2_temp;

	bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_01);
	bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_02);
	bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_03);

	bd71827_reg_write(mfd, BD71827_REG_INT_STAT_11, aft_ra2);
	bd71827_reg_write(mfd, BD71827_REG_INT_STAT_12, aft_ra3);

	return 0;
}

static int third_step(struct bd71827_power *pwr, unsigned thr)
{
	u16 ra2_a3, ra6, ra7;
	u8 ra2, ra3;
	u8 aft_ra2, aft_ra3;
	struct bd71827 *mfd = pwr->mfd;

// fine adjust
	ra2 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_11);	//
	ra3 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_12);	//

	ra6 = bd71827_reg_read(mfd, 0xA6);
	ra7 = bd71827_reg_read(mfd, 0xA7);
	ra6 <<= 8;
	ra6 |= ra7;		// [0xA6 0xA7]
	dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);


	if (ra6 > thr) {
		do {
			ra2_a3 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_11);
			ra2_a3 <<= 8;
			ra3 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_12);
			ra2_a3 |= ra3;
			//ra2_a3 >>= 3; // ? 0xA3[7:3] , or 0xA3[7:0]

			ra2_a3 -= 1;
			//ra2_a3 <<= 3;
			ra3 = ra2_a3;
			bd71827_reg_write(mfd, BD71827_REG_INT_STAT_12, ra3);

			ra2_a3 >>= 8;
			ra2 = ra2_a3;
			bd71827_reg_write(mfd, BD71827_REG_INT_STAT_11, ra2);

			dev_info(pwr->dev, "TEST[A2] = 0x%.2X , TEST[A3] = 0x%.2X \n", ra2, ra3);

			mdelay(1000);	// 1sec?

			ra6 = bd71827_reg_read(mfd, 0xA6);
			ra7 = bd71827_reg_read(mfd, 0xA7);
			ra6 <<= 8;
			ra6 |= ra7;	// [0xA6 0xA7]
			dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);
		} while (ra6 > thr);
	} else if (ra6 < thr) {
		do {
			ra2_a3 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_11);
			ra2_a3 <<= 8;
			ra3 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_12);
			ra2_a3 |= ra3;
			//ra2_a3 >>= 3; // ? 0xA3[7:3] , or 0xA3[7:0]

			ra2_a3 += 1;
			//ra2_a3 <<= 3;
			ra3 = ra2_a3;
			bd71827_reg_write(mfd, BD71827_REG_INT_STAT_12, ra3);

			ra2_a3 >>= 8;
			ra2 = ra2_a3;
			bd71827_reg_write(mfd, BD71827_REG_INT_STAT_11, ra2);

			dev_info(pwr->dev, "TEST[A2] = 0x%.2X , TEST[A3] = 0x%.2X \n", ra2, ra3);

			mdelay(1000);	// 1sec?

			ra6 = bd71827_reg_read(mfd, 0xA6);
			ra7 = bd71827_reg_read(mfd, 0xA7);
			ra6 <<= 8;
			ra6 |= ra7;	// [0xA6 0xA7]
			dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);

		} while (ra6 < thr);
	}

	dev_info(pwr->dev, "[0xA6 0xA7] becomes [0x%.4X] . \n", thr);
	dev_info(pwr->dev, " Calibation finished ... \n\n");

	aft_ra2 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_11);	//
	aft_ra3 = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_12);	// I want to know initial A2 & A3.

	dev_info(pwr->dev, "TEST[A2,A3] = 0x%.2X%.2X\n", aft_ra2, aft_ra3);

	// bd71827_reg_write(mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_00);

	return 0;
}

static ssize_t bd71827_sysfs_set_calibrate(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	unsigned int val, mA;
	static u8 rA2;

	ret = sscanf(buf, "%d %d", &val, &mA);
	if (ret < 1) {
		dev_err(pwr->dev, "error: write a integer string");
		return count;
	}

	if (val == 1) {
		pwr->calib_current = CALIB_START;
		while (pwr->calib_current != CALIB_GO) {
			msleep(500);
		}
		rA2 = first_offset(pwr);
	}
	if (val == 2) {
		second_step(pwr, rA2);
	}
	if (val == 3) {
		if (ret <= 1) {
			dev_err(pwr->dev, "error: Fine adjust need a mA argument!");
		} else {
		unsigned int ra6_thr;

		ra6_thr = mA * 0xFFFF / 20000;
		dev_info(pwr->dev, "Fine adjust at %d mA, ra6 threshold %d(0x%X)\n", mA, ra6_thr, ra6_thr);
		third_step(pwr, ra6_thr);
		}
	}
	if (val == 4) {
		bd71827_reg_write(pwr->mfd, BD71827_REG_I2C_MAGIC, TEST_SEQ_00);
		pwr->calib_current = CALIB_NORM;
		pwr->gauge_delay = 0;
		schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(0));
	}

	return count;
}

static ssize_t bd71827_sysfs_show_calibrate(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	// struct power_supply *psy = dev_get_drvdata(dev);
	// struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;

	ret = 0;
	ret += sprintf(buf + ret, "write string value\n"
		"\t1      0 mA for step one\n"
		"\t2      1000 mA for rough adjust\n"
		"\t3 <mA> for fine adjust\n"
		"\t4      exit current calibration\n");
	return ret;
}

static DEVICE_ATTR(calibrate, S_IWUSR | S_IRUGO,
		bd71827_sysfs_show_calibrate, bd71827_sysfs_set_calibrate);


static ssize_t bd71827_sysfs_set_gauge(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	int delay = -1;

	ret = sscanf(buf, "%d", &delay);
	if (ret < 1) {
		dev_err(pwr->dev, "error: write a integer string");
		return count;
	}

	if (delay == -1) {
		dev_info(pwr->dev, "Gauge schedule cancelled\n");
		cancel_delayed_work(&pwr->bd_work);
		return count;
	}

	dev_info(pwr->dev, "Gauge schedule in %d\n", delay);
	pwr->gauge_delay = delay;
	schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(delay));

	return count;
}

static ssize_t bd71827_sysfs_show_gauge(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;

	ret = 0;
	ret += sprintf(buf + ret, "Gauge schedule in %d\n", pwr->gauge_delay);
	return ret;
}

static DEVICE_ATTR(gauge, S_IWUSR | S_IRUGO,
		bd71827_sysfs_show_gauge, bd71827_sysfs_set_gauge);

static struct attribute *bd71827_sysfs_attributes[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use pwr supply class props.
	 */
	&dev_attr_registers.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_charging.attr,
	&dev_attr_gauge.attr,
	NULL,
};

static const struct attribute_group bd71827_sysfs_attr_group = {
	.attrs = bd71827_sysfs_attributes,
};

/** @brief powers supplied by bd71827_ac */
static char *bd71827_ac_supplied_to[] = {
	BAT_NAME,
};

static const struct power_supply_desc bd71827_ac_desc = {
	.name		= AC_NAME,
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= bd71827_charger_props,
	.num_properties	= ARRAY_SIZE(bd71827_charger_props),
	.get_property	= bd71827_charger_get_property,
};

static const struct power_supply_desc bd71827_battery_desc = {
	.name		= BAT_NAME,
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties	= bd71827_battery_props,
	.num_properties	= ARRAY_SIZE(bd71827_battery_props),
	.get_property	= bd71827_battery_get_property,
};

/* called from pm inside machine_halt */
void bd71827_chip_hibernate(void)
{
	/* programming sequence in EANAB-151 */
	ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_NORMAL);
	ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_RESET);
	
}

/** @brief probe pwr device 
 * @param pdev platform deivce of bd71827_power
 * @retval 0 success
 * @retval negative fail
 */
static int __init bd71827_power_probe(struct platform_device *pdev)
{
	struct bd71827 *bd71827 = dev_get_drvdata(pdev->dev.parent);
	struct bd71827_power *pwr;
	struct power_supply_config ac_cfg = {};
	struct power_supply_config bat_cfg = {};
	int irq, ret;

	pwr = kzalloc(sizeof(*pwr), GFP_KERNEL);
	if (pwr == NULL)
		return -ENOMEM;

	pwr->dev = &pdev->dev;
	pwr->mfd = bd71827;

	platform_set_drvdata(pdev, pwr);

	if (battery_cycle <= 0) {
		battery_cycle = 0;
	}
	dev_info(pwr->dev, "battery_cycle = %d\n", battery_cycle);

	/* If the product often power up/down and the power down time is long, the Coulomb Counter may have a drift. */
	/* If so, it may be better accuracy to enable Coulomb Counter using following commented out code */
	/* for counting Coulomb when the product is power up(including sleep). */
	/* The condition  */
	/* (1) Product often power up and down, the power down time is long and there is no power consumed in power down time. */
	/* (2) Kernel must call this routin at power up time. */
	/* (3) Kernel must call this routin at charging time. */
	/* (4) Must use this code with "Stop Coulomb Counter" code in bd71827_power_remove() function */
	/* Start Coulomb Counter */
	/* bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB); */

	bd71827_set_battery_parameters(pwr);

	bd71827_init_hardware(pwr);

	bat_cfg.drv_data 			= pwr;
	pwr->bat = power_supply_register(&pdev->dev, &bd71827_battery_desc, &bat_cfg);
	if (IS_ERR(pwr->bat)) {
		ret = PTR_ERR(pwr->bat);
		dev_err(&pdev->dev, "failed to register bat: %d\n", ret);
		goto fail_register_bat;
	}

	ac_cfg.supplied_to			= bd71827_ac_supplied_to;
	ac_cfg.num_supplicants		= ARRAY_SIZE(bd71827_ac_supplied_to);
	ac_cfg.drv_data 			= pwr;
	pwr->ac = power_supply_register(&pdev->dev, &bd71827_ac_desc, &ac_cfg);
	if (IS_ERR(pwr->ac)) {
		ret = PTR_ERR(pwr->ac);
		dev_err(&pdev->dev, "failed to register ac: %d\n", ret);
		goto fail_register_ac;
	}

	/*Add DC_IN Inserted and Remove ISR */
	irq  = platform_get_irq(pdev, 0); // get irq number 
#ifdef __BD71827_REGMAP_H__
	irq += bd71827->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd71827_power_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	/* Configure wakeup capable */
	device_set_wakeup_capable(pwr->dev, 1);
	device_set_wakeup_enable(pwr->dev , 1);

	/*add VBAT Low Voltage detection, John Zhang*/
	irq  = platform_get_irq(pdev, 1);
#ifdef __BD71827_REGMAP_H__
	irq += bd71827->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd71827_vbat_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	/* add INT_STAT_11 */
	irq  = platform_get_irq(pdev, 2);
#ifdef __BD71827_REGMAP_H__
	irq += bd71827->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd71827_int_11_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	ret = sysfs_create_group(&pwr->bat->dev.kobj, &bd71827_sysfs_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register sysfs interface\n");
	}

	pwr->reg_index = -1;

	INIT_DELAYED_WORK(&pwr->bd_work, bd_work_callback);

	/* Schedule timer to check current status */
	pwr->calib_current = CALIB_NORM;
	pwr->gauge_delay = 0;
	schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(0));
	pmic_data = bd71827;
	//pm_power_hibernate = bd71827_chip_hibernate;

	return 0;

//error_exit:
	power_supply_unregister(pwr->ac);
fail_register_ac:
	power_supply_unregister(pwr->bat);
fail_register_bat:
	platform_set_drvdata(pdev, NULL);
	kfree(pwr);

	return ret;
}

/** @brief remove pwr device
 * @param pdev platform deivce of bd71827_power
 * @return 0
 */

static int __exit bd71827_power_remove(struct platform_device *pdev)
{
	struct bd71827_power *pwr = platform_get_drvdata(pdev);

	/* If the product often power up/down and the power down time is long, the Coulomb Counter may have a drift. */
	/* If so, it may be better accuracy to disable Coulomb Counter using following commented out code */
	/* for stopping counting Coulomb when the product is power down(without sleep). */
	/* The condition  */
	/* (1) Product often power up and down, the power down time is long and there is no power consumed in power down time. */
	/* (2) Kernel must call this routin at power down time. */
	/* (3) Must use this code with "Start Coulomb Counter" code in bd71827_power_probe() function */
	/* Stop Coulomb Counter */
	/* bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB); */

	sysfs_remove_group(&pwr->bat->dev.kobj, &bd71827_sysfs_attr_group);

	pwr->gauge_delay = -1;
	cancel_delayed_work(&pwr->bd_work);

	power_supply_unregister(pwr->bat);
	power_supply_unregister(pwr->ac);
	platform_set_drvdata(pdev, NULL);
	kfree(pwr);

	return 0;
}

static struct platform_driver bd71827_power_driver = {
	.driver = {
		.name = "bd71827-power",
		.owner = THIS_MODULE,
	},
	.remove = __exit_p(bd71827_power_remove),
};

/** @brief module initialize function */
static int __init bd71827_power_init(void)
{
	return platform_driver_probe(&bd71827_power_driver, bd71827_power_probe);
}

module_init(bd71827_power_init);

/** @brief module deinitialize function */
static void __exit bd71827_power_exit(void)
{
	platform_driver_unregister(&bd71827_power_driver);
}

/*-------------------------------------------------------*/


#define PROCFS_NAME 		"bd71827_rev"
#define BD71827_REV			"BD71827 Driver: Rev008\n"

#define BD71827_BUF_SIZE	1024
static char procfs_buffer[BD71827_BUF_SIZE];
/**
 * This function is called then the /proc file is read
 *
 */
static int onetime = 0;
static ssize_t bd71827_proc_read (struct file *file, char __user *buffer, size_t count, loff_t *data)
{
	int ret = 0, error = 0;
	if(onetime==0) {
		onetime = 1;
		memset( procfs_buffer, 0, BD71827_BUF_SIZE);
		sprintf(procfs_buffer, "%s", BD71827_REV);
		ret = strlen(procfs_buffer);
		error = copy_to_user(buffer, procfs_buffer, strlen(procfs_buffer));
	} else {
		//Clear for next time
		onetime = 0;
	}
	return (error!=0)?0:ret;
}

#if 0
int bd71827_debug_mask = 0;
static ssize_t bd71827_proc_write (struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	sscanf(buffer, "0x%x", &bd71827_debug_mask);
	printk("BD71827: bd71827_debug_mask=0x%08x\n", bd71827_debug_mask);
	return count;
}
#endif

static const struct file_operations bd71827_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= bd71827_proc_read,
	//.write		= bd71827_proc_write,
};

/**
 *This function is called when the module is loaded
 *
 */
int bd71827_revision_init(void)
{
	struct proc_dir_entry *bd71827_proc_entry;

	/* create the /proc/bd71827_rev */
	bd71827_proc_entry = proc_create(PROCFS_NAME, 0644, NULL, &bd71827_proc_fops);
	if (bd71827_proc_entry == NULL) {
		printk("Error: Could not initialize /proc/%s\n", PROCFS_NAME);
		return -ENOMEM;
	}

	return 0;
}
module_init(bd71827_revision_init);
/*-------------------------------------------------------*/

module_exit(bd71827_power_exit);

module_param(use_load_bat_params, int, S_IRUGO);
MODULE_PARM_DESC(use_load_bat_params, "use_load_bat_params:Use loading battery parameters");

module_param(battery_cap_mah, int, S_IRUGO);
MODULE_PARM_DESC(battery_cap_mah, "battery_cap_mah:Battery capacity (mAh)");

module_param(dgrd_cyc_cap, int, S_IRUGO);
MODULE_PARM_DESC(dgrd_cyc_cap, "dgrd_cyc_cap:Degraded capacity per cycle (uAh)");

module_param(soc_est_max_num, int, S_IRUGO);
MODULE_PARM_DESC(soc_est_max_num, "soc_est_max_num:SOC estimation max repeat number");

module_param(dgrd_temp_cap_h, int, S_IRUGO);
MODULE_PARM_DESC(dgrd_temp_cap_h, "dgrd_temp_cap_h:Degraded capacity at high temperature (uAh)");

module_param(dgrd_temp_cap_m, int, S_IRUGO);
MODULE_PARM_DESC(dgrd_temp_cap_m, "dgrd_temp_cap_m:Degraded capacity at middle temperature (uAh)");

module_param(dgrd_temp_cap_l, int, S_IRUGO);
MODULE_PARM_DESC(dgrd_temp_cap_l, "dgrd_temp_cap_l:Degraded capacity at low temperature (uAh)");

module_param(battery_cycle, uint, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(battery_parameters, "battery_cycle:battery charge/discharge cycles");

module_param_array(ocv_table, int, NULL, S_IRUGO);
MODULE_PARM_DESC(ocv_table, "ocv_table:Open Circuit Voltage table (uV)");

module_param_array(vdr_table_h, int, NULL, S_IRUGO);
MODULE_PARM_DESC(vdr_table_h, "vdr_table_h:Voltage Drop Ratio temperatyre high area table");

module_param_array(vdr_table_m, int, NULL, S_IRUGO);
MODULE_PARM_DESC(vdr_table_m, "vdr_table_m:Voltage Drop Ratio temperatyre middle area table");

module_param_array(vdr_table_l, int, NULL, S_IRUGO);
MODULE_PARM_DESC(vdr_table_l, "vdr_table_l:Voltage Drop Ratio temperatyre low area table");

module_param_array(vdr_table_vl, int, NULL, S_IRUGO);
MODULE_PARM_DESC(vdr_table_vl, "vdr_table_vl:Voltage Drop Ratio temperatyre very low area table");

MODULE_AUTHOR("Cong Pham <cpham2403@gmail.com>");
MODULE_DESCRIPTION("BD71827 Battery Charger Power driver");
MODULE_LICENSE("GPL");
