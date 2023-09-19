/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/mdio.h>
#include <board/ethphy.h>
#include <board/led.h>
#include "SORTE_g_master_PRU0.h"
#include "SORTE_g_device_PRU0.h"
#include "SORTE_g_device_PRU1.h"
#include "../include/sorte_g_device_host_interface.h"
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

char SORTE_VERSION[] = "2023-7-12-001";
PRUICSS_Handle gPruIcss0Handle;
volatile uint32_t        ModeStatus = 0;
static void *gPru_ctrl;
void *gPru_dramx;HwiP_Object         gPRUHwiObject;
#define CTR_EN (1 << 3)
uintptr_t icssgBaseAddr = 0;
LED_Handle ledHandle = 0;
volatile uint32_t UART_FLAG_CONTINUE = 0;

static void PRU_IsrFxn();
/* ========================================================================== */
/*                          SORTE_G Definitions                               */
/* ========================================================================== */
#define PD_INPUT_SIZE 4
#define PD_OUTPUT_SIZE 4

// Controller param frame configuration
#define PARAM_DATA_OFFSET 0x20
#define Cycle_time      0x00       // 32-bit
#define Sync_time       0x04       // 32-bit
#define Delay_period    0x08       // 32-bit
#define Delay_burst     0x0C       // 8-bit
#define Delay_gap       0x0D       // 8-bit
#define Delay_forward_time  0x0E   // 16-bit
#define Inter_packet_gap    0x10   // 16-bit
#define T_in            0x12       // 16-bit
#define T_out           0x14       // 16-bit
#define Topology        0x15       // 8-bit
#define Diagnostics     0x16       // 8-bit
#define Alarm           0x17       // 8-bit
#define Crc_mode        0x18       // 8-bit

#define OVERSAMPLING_1      (1+1)   // 4us
#define OVERSAMPLING_10     (10+1)  // 40us
#define OVERSAMPLING_100    (100+1) // 400us

#define NEW_SMA_SETTINGS    0x80
#define FILTER_CTRL_NO_COMP 0x40
#define FILTER_CTRL_NO_COMP_AT_STARTUP 0x20

// Simple Moving Average filter
#define SMA_WRAP_AROUND_1   0x03    // 1 SMA values
#define SMA_MULTIPLIER_1    0       // /1 SMA values
#define SMA_WRAP_AROUND_2   0x07    // 2 SMA values
#define SMA_MULTIPLIER_2    1       // /2 SMA values
#define SMA_WRAP_AROUND_4   0x0f    // 4 SMA values
#define SMA_MULTIPLIER_4    2       // /4 SMA values
#define SMA_WRAP_AROUND_8   0x1f    // 8 SMA values
#define SMA_MULTIPLIER_8    3       // /8 SMA values
#define SMA_WRAP_AROUND_16  0x3f    // 16 SMA values
#define SMA_MULTIPLIER_16   4       // /16 SMA values
#define SMA_WRAP_AROUND_32  0x7F    // 32 SMA values
#define SMA_MULTIPLIER_32   5       //  /32 SMA values
#define SMA_WRAP_AROUND_64  0xFF    // 64 SMA values
#define SMA_MULTIPLIER_64   6       // /64 SMA values

void generic_pruss_init()
{
    HwiP_Params     hwiPrms;
    uint32_t        intrNum, temp32;
    uint16_t        temp16=0;
    uint8_t enhancedlink_enable = 0;    // fast link down disabled

    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);
    uintptr_t icssgBaseAddr = (((PRUICSS_HwAttrs *)((gPruIcss0Handle)->hwAttrs))->baseAddr);

    // set C28 to PRU-ICSS shared memory 0x10000
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRU0, PRUICSS_CONST_TBL_ENTRY_C28, 0x0100);
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRU1, PRUICSS_CONST_TBL_ENTRY_C28, 0x0100);

    /* clear ICSS0 PRU data RAM */
    PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU0));
    PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU1));
    PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_SHARED_RAM);
    /* taken from sysconfig in file  ti_drivers_config.c */
    PRUICSS_intcInit(gPruIcss0Handle, &icss1_intc_initdata);

    // Use GPIO1_35 to select controller or device
    // remove jumper -> controller
    // set jumper J6-60 to J6-59 -> device
    ModeStatus = HW_RD_REG32(0x00601048);
    if((ModeStatus & (1<<(35-32))) == (1<<(35-32)))
        ModeStatus = 1; // controller
    else
        ModeStatus = 0; // device


    // enable/configure MDIO
    // ToDo - use SDK command for this
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE + 0x04, 0x400000ff);

    // User LED
    //TEST_LED3_RED: GPIO1_39
    //TEST_LED4_GREEN: GPIO1_38
    temp32 = HW_RD_REG32(0x00601038);
    temp32 = ~((1<<(38-32)) | (1<<(39-32)));
    HW_WR_REG32(0x00601038, temp32);
    HW_WR_REG32(0x00601040, 1<<(38-32));  // red
    HW_WR_REG32(0x00601044, 1<<(38-32));
    HW_WR_REG32(0x00601040, 1<<(39-32));
    HW_WR_REG32(0x00601044, 1<<(39-32));  // green
    HW_WR_REG32(0x00601040, 1<<(38-32));
    HW_WR_REG32(0x00601040, 1<<(39-32));
    HW_WR_REG32(0x00601044, 1<<(38-32));
    HW_WR_REG32(0x00601044, 1<<(39-32));


    // configure RGMII
    // enable Gbit interface RGMII0_GIG_IN
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE + + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG, 0 | \
                (1<< CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_GIG_IN_SHIFT) | \
                (1<< CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_GIG_IN_SHIFT));

    // Configure RGMII interface
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE + + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG, 0x0001002B);

    // set IPG to 96ns
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0, 0x18);
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1, 0x18);

    // configure IEP counter to 200 MHz, 32-bit mode
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG, 0x441);
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0, 0xFFFFFFFF);
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1, 0x0);
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG, 0x3);

    // reset and start IEP Timer, enable capture
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG0, 0);
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG1, 0);
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG, 0x441);
    HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_CAP_CFG_REG, 0x0001FC3F);


    // mapping of PRU INTC host interrupts to ARM R5F interrupt
    //   host0 = 120
    //   ...
    //   host7 = 127
    intrNum          = 120;
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU0);
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU1);
    /* Register PRU interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = (void *) &PRU_IsrFxn;
    HwiP_construct(&gPRUHwiObject, &hwiPrms);

    /* enable cycle counter */
    gPru_ctrl =  (void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + CSL_ICSS_G_PR1_PDSP1_IRAM_REGS_BASE);
    HW_WR_REG32(gPru_ctrl, CTR_EN);

    // PHYs are configured for RGMII TX clock delay (register 0x0032, bit 1)
    MDIO_phyRegRead(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, NULL, 3, 0x01, &temp16);
    MDIO_phyRegRead(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, NULL, 15, 0x01, &temp16);
    MDIO_phyExtRegWrite(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, NULL, 3, 0x32, 0xD2);
    MDIO_phyRegWrite(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, NULL, 3, 0x1F, 0x4000);
    MDIO_phyExtRegWrite(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, NULL, 15, 0x32, 0xD2);
    MDIO_phyRegWrite(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, NULL, 15, 0x1F, 0x4000);

    if(enhancedlink_enable == 0)
    {
        MDIO_enableLinkInterrupt(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, 0, 3, MDIO_LINKSEL_MDIO_MODE);
        MDIO_enableLinkInterrupt(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, 1, 15, MDIO_LINKSEL_MDIO_MODE);
    }
    else
    {
        MDIO_enableLinkInterrupt(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, 0, 3, MDIO_LINKSEL_MLINK_MODE);
        MDIO_enableLinkInterrupt(icssgBaseAddr + CSL_ICSS_G_PR1_MDIO_V1P7_MDIO_REGS_BASE, 1, 15, MDIO_LINKSEL_MLINK_MODE);
    }

    // Setup controller/device configuration data
    if(ModeStatus == 1) //controller
    {
        HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Cycle_time, 10000);
        HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Sync_time, 0);
        HW_WR_REG32(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Delay_period, 0);
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Delay_burst, 16);
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Delay_gap, 0);
        HW_WR_REG16(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Delay_forward_time, 0);
        HW_WR_REG16(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Inter_packet_gap, 96);
        HW_WR_REG16(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + T_in, 0);
        HW_WR_REG16(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + T_out, 0);
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Topology, 0);
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Diagnostics, 0);
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Alarm, 0);
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE + PARAM_DATA_OFFSET + Crc_mode, 1);
    }
    else // device
    {
#define MDIO_PHY_CONFIG_OFFSET (0x04U)
        // Configure share RAM
        HW_WR_REG16(icssgBaseAddr + CSL_ICSS_G_RAM_SLV_RAM_REGS_BASE + MDIO_PHY_CONFIG_OFFSET, (3<<0) | (15<<8)); // set MDIO address: 0x0000.8008
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_RAM_SLV_RAM_REGS_BASE + DEVICE_INDATA_FRAME_BUFFER_PTR, DEVICE_INDATA_FRAME_BUFFER_LOCATION);
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_RAM_SLV_RAM_REGS_BASE + DEVICE_INDATA_FRAME_BUFFER_SIZE, 12); // 12 Bytes payload
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_RAM_SLV_RAM_REGS_BASE + DEVICE_OUTDATA_FRAME_BUFFER_PTR, DEVICE_OUTDATA_FRAME_BUFFER_LOCATION);
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_RAM_SLV_RAM_REGS_BASE + DEVICE_OUTDATA_FRAME_BUFFER_SIZE, 12); // 12 Bytes payload
        HW_WR_REG8(icssgBaseAddr + CSL_ICSS_G_RAM_SLV_RAM_REGS_BASE + DEVICE_MASTER_FRAME_BUFFER_PTR, DEVICE_MASTER_FRAME_BUFFER_LOCATION);
    }
    if(ModeStatus == 1) //controller
    {
        /* load sorte_controller firmware */
        PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRU0),
                         0, (uint32_t *) sorte_g_master_0,
                         sizeof(sorte_g_master_0));

        PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRU0);

        /* wait 2 seconds */
        ClockP_usleep(2000000);

        /*Run firmware*/
        PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU0);
//        DebugP_log("PRESS 1 to start Controller\r\n");
    }
    else {
        /* load sorte_controller firmware */
         PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRU0),
                          0, (uint32_t *) sorte_g_device_PRU0_0,
                          sizeof(sorte_g_device_PRU0_0));
         PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRU1),
                          0, (uint32_t *) sorte_g_device_PRU1_0,
                          sizeof(sorte_g_device_PRU1_0));

         PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRU0);
         PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRU1);

         /*Run firmware*/
         PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU0);
         PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU1);
    }

}

static void PRU_IsrFxn()
{
    PRUICSS_clearEvent(gPruIcss0Handle, 16);  // 16 is PRU0 intr[0]
}


void sorte_g_main(void *args)
{
    uint32_t i;
    uint8_t cd, cd_old = -1;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    generic_pruss_init();

    icssgBaseAddr = (((PRUICSS_HwAttrs *)((gPruIcss0Handle)->hwAttrs))->baseAddr);

    ledHandle = gLedHandle[CONFIG_LED0];
    DebugP_assert(NULL != ledHandle);
    LED_setMask(ledHandle, 0xFFU);
    LED_on(ledHandle, 0xFF);
    for(i=0; i<4; i++)
        LED_off(ledHandle, i);

    // enable UART task
    UART_FLAG_CONTINUE = 1;

    while (1) {
        ClockP_usleep(5000);
        // Update process data LEDs
        /* todo: if mode 1 (controller) read from controller IO space */
        if (ModeStatus == 0)
          cd=HW_RD_REG8(icssgBaseAddr + 0x2000 + DEVICE_OUTDATA_FRAME_BUFFER_LOCATION); // IO data in PRU1 RAM
        else
          cd=HW_RD_REG8(icssgBaseAddr + 0x800 + 0x01);  // INDATA data from first device

        if(cd != cd_old) {
            cd_old = cd;
            if((cd & 0x01) || (cd & 0x80)) {
                LED_on(ledHandle, 0);
            }  else {
                LED_off(ledHandle, 0);
            }
            if((cd & 0x02) || (cd & 0x40)) {
                LED_on(ledHandle, 1);
            }  else {
                LED_off(ledHandle, 1);
            }
            if((cd & 0x04) || (cd & 0x20)) {
                LED_on(ledHandle, 2);
            }  else {
                LED_off(ledHandle, 2);
            }
            if((cd & 0x08) || (cd & 0x10)) {
                LED_on(ledHandle, 3);
            }  else {
                LED_off(ledHandle, 3);
            }
        }
    }

    Board_driversClose();
    Drivers_close();
}


void terminal(void *args)
{
    char cmd;

    while(UART_FLAG_CONTINUE == 0)
    {
        ClockP_usleep(2000);
    }

    DebugP_log("\r\nSORTE_G Demo application for AM243x Launchpad\r\n");
    DebugP_log("Version %s\r\n", SORTE_VERSION);
    DebugP_log("To configure device or controller\r\n\tSet jumper to J6 pin59/60(GND) for device\r\n\tremove jumper from J6 pin59/60(GND) for controller\r\n");
    DebugP_log("\tThis board works as\r\n");
    if(ModeStatus == 1)
        DebugP_log("\tCONTROLLER\r\n");
    else
        DebugP_log("\tDEVICE\r\n");

    DebugP_log("SORTE_G initialized\r\n");

    while (1) {
        DebugP_log("Enter command: \r\n");
        DebugP_scanf("%c", &cmd);
        switch(cmd) {
        case '0':
                // switch off SORTE_G state LEDs
                HW_WR_REG32(0x00601044, 1<<(38-32));
                HW_WR_REG32(0x00601044, 1<<(39-32));
                DebugP_log("Stop PRUs\r\n");
                if(ModeStatus==1) // controller
                    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU0);
                else //device
                {
                    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU0);
                    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU1);
                }
                break;
        case '1':
                // switch off SORTE_G state LEDs
                HW_WR_REG32(0x00601044, 1<<(38-32));
                HW_WR_REG32(0x00601044, 1<<(39-32));
                DebugP_log("Restart PRUs\r\n");
                if(ModeStatus==1)
                {
                    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU0);
                    PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRU0);
                    PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU0);

                }else
                {
                    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU0);
                    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU1);
                    PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRU0);
                    PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRU1);
                    PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU0);
                    PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU1);
                }
                break;
        case '?':
        default:
            DebugP_log("Commands\r\n0 - Stop PRUs\r\n1 - Restart PRUs\r\n");
            break;
        }

    }
}
