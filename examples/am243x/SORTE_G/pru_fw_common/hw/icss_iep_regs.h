;************************************************************************************
;**+------------------------------------------------------------------------------+**
;**|                              ******                                          |**
;**|                              ******     o                                    |**
;**|                              *******__////__****                             |**
;**|                              ***** /_ //___/ ***                             |**
;**|                           ********* ////__ ******                            |**
;**|                             *******(_____/ ******                            |**
;**|                                 **********                                   |**
;**|                                   ******                                     |**
;**|                                      ***                                     |**
;**|                                                                              |**
;**|            Copyright (c) 2017 Texas Instruments Incorporated                 |**
;**|                           ALL RIGHTS RESERVED                                |**
;**|                                                                              |**
;**|    Permission is hereby granted to licensees of Texas Instruments            |**
;**|    Incorporated (TI) products to use this computer program for the sole      |**
;**|    purpose of implementing a licensee product based on TI products.          |**
;**|    No other rights to reproduce, use, or disseminate this computer           |**
;**|    program, whether in part or in whole, are granted.                        |**
;**|                                                                              |**
;**|    TI makes no representation or warranties with respect to the              |**
;**|    performance of this computer program, and specifically disclaims          |**
;**|    any responsibility for any damages, special or consequential,             |**
;**|    connected with the use of this program.                                   |**
;**|                                                                              |**
;**+------------------------------------------------------------------------------+**
;************************************************************************************
; file:     icss_iep_regs.h
;
; brief:    ICSS Industrial Ethernet Peripheral Registers

    .if !$isdefed("____icss_iep_regs_h")
____icss_iep_regs_h     .set    1

ICSS_IEP_TIMER_BLK_INDEX        .set    0

ICSS_IEP_GLOBAL_CFG_REG         .set    0x0000
ICSS_IEP_GLOBAL_STATUS_REG      .set    0x0004
ICSS_IEP_COMPEN_REG             .set    0x0008
ICSS_IEP_COUNT_REG              .set    0x0010
ICSS_IEP_CAP_CFG_REG            .set    0x0018
ICSS_IEP_CAP_STATUS_REG         .set    0x001c
ICSS_IEP_CAPR0_REG              .set    0x0020
ICSS_IEP_CAPR1_REG              .set    0x0028
ICSS_IEP_CAPR2_REG              .set    0x0030
ICSS_IEP_CAPR3_REG              .set    0x0038
ICSS_IEP_CAPR4_REG              .set    0x0040
ICSS_IEP_CAPR5_REG              .set    0x0048
ICSS_IEP_CAPR6_REG              .set    0x0050
ICSS_IEP_CAPF6_REG              .set    0x0058
ICSS_IEP_CAPR7_REG              .set    0x0060
ICSS_IEP_CAPF7_REG              .set    0x0068
ICSS_IEP_CMP_CFG_REG            .set    0x0070
ICSS_IEP_CMP_STATUS_REG         .set    0x0074
ICSS_IEP_CMP0_REG               .set    0x0078
ICSS_IEP_CMP1_REG               .set    0x0080
ICSS_IEP_CMP2_REG               .set    0x0088
ICSS_IEP_CMP3_REG               .set    0x0090
ICSS_IEP_CMP4_REG               .set    0x0098
ICSS_IEP_CMP5_REG               .set    0x00a0
ICSS_IEP_CMP6_REG               .set    0x00a8
ICSS_IEP_CMP7_REG               .set    0x00b0
ICSS_IEP_RXIPG0_REG             .set    0x00b8
ICSS_IEP_RXIPG1_REG             .set    0x00bc

ICSS_IEP_SYNC_BLK_INDEX         .set    1
ICSS_IEP_SYNC_CTRL_REG          .set    0x0000
ICSS_IEP_SYNC_FIRST_STATUS_REG  .set    0x0004
ICSS_IEP_SYNC0_STATUS_REG       .set    0x0008
ICSS_IEP_SYNC1_STATUS_REG       .set    0x000C
ICSS_IEP_SYNC_PWIDTH_REG        .set    0x0010
ICSS_IEP_SYNC0_PERIOD_REG       .set    0x0014
ICSS_IEP_SYNC1_DELAY_REG        .set    0x0018
ICSS_IEP_SYNC_START_REG         .set    0x001C

ICSS_IEP_WD_BLK_INDEX           .set    2
ICSS_IEP_WD_PREDIV_REG          .set    0x0000
ICSS_IEP_PDI_WD_TIM_REG         .set    0x0004
ICSS_IEP_PD_WD_TIM_REG          .set    0x0008
ICSS_IEP_WD_STATUS_REG          .set    0x000C
ICSS_IEP_EXP_COUNTER_REG        .set    0x0010
ICSS_IEP_WD_CTRL_REG            .set    0x0014

ICSS_IEP_DIGIO_BLK_INDEX        .set    3
ICSS_IEP_DIGIO_CTRL_REG         .set    0x0000
ICSS_IEP_DIGIO_STATUS_REG       .set    0x0004
ICSS_IEP_DIGIO_DATA_IN_REG      .set    0x0008
ICSS_IEP_DIGIO_DATA_IN_RAW_REG  .set    0x000C
ICSS_IEP_DIGIO_DATA_OUT_REG     .set    0x0010
ICSS_IEP_DIGIO_DATA_OUT_EN_REG  .set    0x0014
ICSS_IEP_DIGIO_EXP_REG          .set    0x0018

ICSS_IEP_CMP_STATUS_CLR_0       .set    0x01
ICSS_IEP_CMP_STATUS_CLR_1       .set    0x02
ICSS_IEP_CMP_STATUS_CLR_2       .set    0x04
ICSS_IEP_CMP_STATUS_CLR_3       .set    0x08
ICSS_IEP_CMP_STATUS_CLR_4       .set    0x10
ICSS_IEP_CMP_STATUS_CLR_5       .set    0x20
ICSS_IEP_CMP_STATUS_CLR_6       .set    0x40
ICSS_IEP_CMP_STATUS_CLR_7       .set    0x80
ICSS_IEP_CMP_STATUS_CLR_ALL     .set    0xFF
ICSS_IEP_LOW_MAX_VALUE          .set    0xFFFFFFFF

    .endif ;____icss_iep_regs_h
