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
; file:     icss_xfer_defines.h
;
; brief:    XFER ID defines
;           Includes:
;           1. 
;

    .if !$isdefed("____icss_xfer_defines_h")
____icss_xfer_defines_h   .set    1


;--------------------------------------------------------------------------;
;   Constants: XFER IDS
;
;   MAC_XID  - XID for Multiplier (with optional accumulation)
;   RTU_SPAD_B0_XID - XID for RTU Scratch PAD Bank 0
;   RTU_SPAD_B1_XID - XID for RTU Scratch PAD Bank 1
;   PRU_SPAD_B0_XID - XID for PRU Scratch PAD Bank 0
;   PRU_SPAD_B1_XID - XID for PRU Scratch PAD Bank 1
;   PRU_SPAD_B2_XID - XID for PRU Scratch PAD Bank 2
;   PRU_SPAD_B3_XID - XID for PRU Scratch PAD Bank 3
;   PRU_RTU_SPAD_XID - XID for PRU-RTU 32 byte Scratch PAD
;   RXL2_SIDEA_XID  - XID for RXL2 Bank A
;   RXL2_SIDEB_XID  - XID for RXL2 Bank B
;   RXL2_FILTER_XID - XID for RXL2 Filter
;   RTU_BSRAM_XID   - XID for RTU Broadside RAM
;   FDB_DATA_XID - XID for FDB Data
;   FDB_ADDR_XID - XID for FDB Address & Config
;   FDB_GEN_PATTERN_XID   - XID for General Pattern Config
;   FDB_P0_RES_XID  - XID for FDB Physical Port1 results 
;   FDB_P1_RES_XID  - XID for FDB Physical Port2 results
;   FDB_HOST_RES_XID - XID for FDB Host Port results
;   FDB_GEN_RES_XID  -  XID for General compare Results
;   RTU_BSRAM_SUM32_XID - XID for RTU Broadside RAM + SUM32
;   TXL2_XID        - XID for TXL2 FIFO 
;   PRU_BSRAM_XID - XID for PRU Broadside RAM
;   PRU_BSRAM_SUM32_XID - XID for PRU Broadside RAM + SUM32
;   XFR2PSI_STAT_XID - XID for XFR2PSI Status 
;   XFR2PSI_RD1_XID - XID for XFR2PSI Read Widget1
;   XFR2PSI_WR1_XID - XID for XFR2PSI Write Widget1 
;   XFR2PSI_RD2_XID - XID for XFR2PSI Read Widget2
;   XFR2PSI_RD3_XID - XID for XFR2PSI Read Widget3
;   XFR2PSI_RD4_XID - XID for XFR2PSI Read Widget4 
;   XFR2VBUSP_RD0_XID - XID for XFR2VBUS Read Widget0 (for external memory access)
;   XFR2VBUSP_RD1_XID - XID for XFR2VBUS Read Widget1 (for external memory access)
;   XFR2VBUSP_WR0_XID - XID for XFR2VBUS Write Widget0 (for external memory access)
;   XFR2VBUSP_WR1_XID - XID for XFR2VBUS Write Widget1 (for external memory access)
;   XFR2TR_MASTER_XID - XID for Source TR pool
;   XFR2TR_RING_XID - XID for Destination TR ring
;   XFR2TR_SUBMIT_XID - XID for Trigger submit from Source TR pool to Destination TR ring 
;   INT_SPIN_XID - XID for Local spinlock
;   EXT0_SPIN_XID - XID for Spinlock in other ICSSG
;   EXT1_SPIN_XID - XID for Spinlock in other ICSSG 
;   BSWAP_XID       - XID for Endian swap
;   BSWAP_4_8_XID   - XID for Pivot swap (R2:R5 <=> R6:R9)
;   BSWAP_4_16_XID   - XID for Pivot swap (R2:R5 <=> R14:R17 && R6:R9 <=> R10:R13)
;   TM_YIELD_XID    - XID for Task manager yield xin instruction
;   TM_SAVE_XID - PRU_SPAD_B0_XID for PRU0 and PRU_SPAD_B1_XID for PRU1
;
;--------------------------------------------------------------------------;
MAC_XID .set 0
CRC_XID .set 1   
RTU_SPAD_B0_XID .set 10
RTU_SPAD_B1_XID .set 11
PRU_SPAD_B0_XID .set 10
PRU_SPAD_B1_XID .set 11
PRU_SPAD_B2_XID .set 12
PRU_SPAD_B3_XID .set 13
PRU_RTU_SPAD_XID .set 15
RXL2_SIDEA_XID .set 20
RXL2_SIDEB_XID .set 21
RXL2_FILTER_XID .set 22
RTU_BSRAM_XID .set 30
FDB_DATA_XID .set 30
FDB_ADDR_XID .set 30
FDB_GEN_PATTERN_XID .set 31
FDB_P0_RES_XID .set 32
FDB_P1_RES_XID .set 33
FDB_HOST_RES_XID .set 34
FDB_GEN_RES_XID .set 35
RTU_BSRAM_SUM32_XID .set 38
TXL2_XID .set 40
PRU_BSRAM_XID .set 48
PRU_BSRAM_SUM32_XID .set 49
XFR2PSI_STAT_XID .set 80
XFR2PSI_RD1_XID .set 81
XFR2PSI_WR1_XID .set 81
XFR2PSI_RD2_XID .set 82
XFR2PSI_RD3_XID .set 83
XFR2PSI_RD4_XID .set 84
XFR2VBUSP_RD0_XID .set 96
XFR2VBUSP_RD1_XID .set 97
XFR2VBUSP_WR0_XID .set 98
XFR2VBUSP_WR1_XID .set 99
XFR2TR_MASTER_XID .set 112
XFR2TR_RING_XID .set 113
XFR2TR_SUBMIT_XID .set 114  
INT_SPIN_XID .set 144
EXT0_SPIN_XID .set 145
EXT1_SPIN_XID .set 146
BSWAP_XID .set 160
BSWAP_4_8_XID .set 161
BSWAP_4_16_XID .set 162
TM_YIELD_XID .set 252
    .endif ; ____icss_xfer_defines_h
