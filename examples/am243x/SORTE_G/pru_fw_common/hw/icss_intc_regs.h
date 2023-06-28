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
; file:     icss_intc_regs.h
;
; brief:    ICSS Interrupt Controller Module Registers

    .if !$isdefed("____icss_intc_regs_h")
____icss_intc_regs_h    .set    1

ICSS_INTC_REVID     .set    0x0000
ICSS_INTC_CR        .set    0x0004
ICSS_INTC_HCR       .set    0x000C
ICSS_INTC_GER       .set    0x0010
ICSS_INTC_GNLR      .set    0x001C
ICSS_INTC_SISR      .set    0x0020
ICSS_INTC_SICR      .set    0x0024
ICSS_INTC_EISR      .set    0x0028
ICSS_INTC_EICR      .set    0x002C
ICSS_INTC_HIEISR    .set    0x0034
ICSS_INTC_HIDSR     .set    0x0038
ICSS_INTC_GPIR      .set    0x0080
ICSS_INTC_SRSR1     .set    0x0200
ICSS_INTC_SRSR2     .set    0x0204
ICSS_INTC_SECR1     .set    0x0280
ICSS_INTC_SECR2     .set    0x0284
ICSS_INTC_ESR1      .set    0x0300
ICSS_INTC_ESR2      .set    0x0304
ICSS_INTC_ECR1      .set    0x0380
ICSS_INTC_ECR2      .set    0x0384
ICSS_INTC_CMR1      .set    0x0400
ICSS_INTC_CMR2      .set    0x0404
ICSS_INTC_CMR3      .set    0x0408
ICSS_INTC_CMR4      .set    0x040C
ICSS_INTC_CMR5      .set    0x0410
ICSS_INTC_CMR6      .set    0x0414
ICSS_INTC_CMR7      .set    0x0418
ICSS_INTC_CMR8      .set    0x041C
ICSS_INTC_CMR9      .set    0x0420
ICSS_INTC_CMR10     .set    0x0424
ICSS_INTC_CMR11     .set    0x0428
ICSS_INTC_CMR12     .set    0x042C
ICSS_INTC_CMR13     .set    0x0430
ICSS_INTC_CMR14     .set    0x0434
ICSS_INTC_CMR15     .set    0x0438
ICSS_INTC_CMR16     .set    0x043C
ICSS_INTC_HMR1      .set    0x0800
ICSS_INTC_HMR2      .set    0x0804
ICSS_INTC_HMR3      .set    0x0808
ICSS_INTC_HIPIR1    .set    0x0900
ICSS_INTC_HIPIR2    .set    0x0904
ICSS_INTC_HIPIR3    .set    0x0908
ICSS_INTC_HIPIR4    .set    0x090C
ICSS_INTC_HIPIR5    .set    0x0910
ICSS_INTC_HIPIR6    .set    0x0914
ICSS_INTC_HIPIR7    .set    0x0918
ICSS_INTC_HIPIR8    .set    0x091C
ICSS_INTC_HIPIR9    .set    0x0920
ICSS_INTC_HIPIR10   .set    0x0924
ICSS_INTC_SIPR1     .set    0x0D00
ICSS_INTC_SIPR2     .set    0x0D04
ICSS_INTC_SITR1     .set    0x0D80
ICSS_INTC_SITR2     .set    0x0D84
ICSS_INTC_HINLR1    .set    0x1100
ICSS_INTC_HINLR2    .set    0x1104
ICSS_INTC_HINLR3    .set    0x1108
ICSS_INTC_HINLR4    .set    0x110C
ICSS_INTC_HINLR5    .set    0x1110
ICSS_INTC_HINLR6    .set    0x1114
ICSS_INTC_HINLR7    .set    0x1118
ICSS_INTC_HINLR8    .set    0x111C
ICSS_INTC_HINLR9    .set    0x1120
ICSS_INTC_HINLR10   .set    0x1124
ICSS_INTC_HIER      .set    0x1500

ICSS_INTC_eCAP_MASK     .set    0x8000

;----------------------- INTC MII_RT Mode Interrupts -----------------------;

    ;System Events 32 to 55.
ICSS_INTC_MIIRT_PRU0_RX_ERR         .set    0
ICSS_INTC_MIIRT_PRU0_RX_ERR32       .set    1
ICSS_INTC_MIIRT_PRU0_RX_SFD         .set    2
ICSS_INTC_MIIRT_PRU0_RX_SOF         .set    3
ICSS_INTC_MIIRT_PRU0_RX_CRC         .set    4
ICSS_INTC_MIIRT_PRU0_RX_NIBBLE_ODD  .set    5
ICSS_INTC_MIIRT_PRU0_RX_OVERFLOW    .set    6
ICSS_INTC_MIIRT_PORT0_TX_UNDERFLOW  .set    7
ICSS_INTC_MIIRT_PORT0_TX_OVERFLOW   .set    8
ICSS_INTC_MIIRT_MDIO_LINK0          .set    9
ICSS_INTC_MIIRT_PRU0_RX_EOF         .set    10
ICSS_INTC_MIIRT_RESERVED0           .set    11
ICSS_INTC_MIIRT_PRU1_RX_ERR         .set    12
ICSS_INTC_MIIRT_PRU1_RX_ERR32       .set    13
ICSS_INTC_MIIRT_PRU1_RX_SFD         .set    14
ICSS_INTC_MIIRT_PRU1_RX_SOF         .set    15
ICSS_INTC_MIIRT_PRU1_RX_CRC         .set    16
ICSS_INTC_MIIRT_PRU1_RX_NIBBLE_ODD  .set    17
ICSS_INTC_MIIRT_PRU1_RX_OVERFLOW    .set    18
ICSS_INTC_MIIRT_PORT1_TX_UNDERFLOW  .set    19
ICSS_INTC_MIIRT_PORT1_TX_OVERFLOW   .set    20
ICSS_INTC_MIIRT_MDIO_LINK1          .set    21
ICSS_INTC_MIIRT_PRU1_RX_EOF         .set    22

    ;Masks for System Events 32 to 47.
ICSS_INTC_MIIRT_PRU0_RX_ERR_MASK         .set    0x0001
ICSS_INTC_MIIRT_PRU0_RX_ERR32_MASK       .set    0x0002
ICSS_INTC_MIIRT_PRU0_RX_SFD_MASK         .set    0x0004
ICSS_INTC_MIIRT_PRU0_RX_SOF_MASK         .set    0x0008
ICSS_INTC_MIIRT_PRU0_RX_CRC_MASK         .set    0x0010
ICSS_INTC_MIIRT_PRU0_RX_NIBBLE_ODD_MASK  .set    0x0020
ICSS_INTC_MIIRT_PRU0_RX_OVERFLOW_MASK    .set    0x0040
ICSS_INTC_MIIRT_PORT0_TX_UNDERFLOW_MASK  .set    0x0080
ICSS_INTC_MIIRT_PORT0_TX_OVERFLOW_MASK   .set    0x0100
ICSS_INTC_MIIRT_MDIO_LINK0_MASK          .set    0x0200
ICSS_INTC_MIIRT_PRU0_RX_EOF_MASK         .set    0x0400
ICSS_INTC_MIIRT_RESERVED0_MASK           .set    0x0800
ICSS_INTC_MIIRT_PRU1_RX_ERR_MASK         .set    0x1000
ICSS_INTC_MIIRT_PRU1_RX_ERR32_MASK       .set    0x2000
ICSS_INTC_MIIRT_PRU1_RX_SFD_MASK         .set    0x4000
ICSS_INTC_MIIRT_PRU1_RX_SOF_MASK         .set    0x8000

    ;Masks for System Events 48 to 55.
ICSS_INTC_MIIRT_PRU1_RX_CRC_MASK         .set    0x0001
ICSS_INTC_MIIRT_PRU1_RX_NIBBLE_ODD_MASK  .set    0x0002
ICSS_INTC_MIIRT_PRU1_RX_OVERFLOW_MASK    .set    0x0004
ICSS_INTC_MIIRT_PORT1_TX_UNDERFLOW_MASK  .set    0x0008
ICSS_INTC_MIIRT_PORT1_TX_OVERFLOW_MASK   .set    0x0010
ICSS_INTC_MIIRT_MDIO_LINK1_MASK          .set    0x0020
ICSS_INTC_MIIRT_PRU1_RX_EOF_MASK         .set    0x0040

;------------------- End of INTC MII_RT Mode Interrupts --------------------;

;-------------------- INTC PRU System Event Interrupts ---------------------;

    ;System Events 16 to 31.
    ;Output channels 0 to 15 are mapped to system events 16 to 31.
    ;R31[3:0] must specify the channel number and R31[5] should be set to assert the SW interrupt.

ICSS_INTC_PRU_SYS_EVT_16    .set    0x20    ;16 - 16 = 0  -> 10 0000 = 0x20
ICSS_INTC_PRU_SYS_EVT_17    .set    0x21    ;17 - 16 = 1  -> 10 0001 = 0x21
ICSS_INTC_PRU_SYS_EVT_18    .set    0x22    ;18 - 16 = 2  -> 10 0010 = 0x22
ICSS_INTC_PRU_SYS_EVT_19    .set    0x23    ;19 - 16 = 3  -> 10 0011 = 0x23
ICSS_INTC_PRU_SYS_EVT_20    .set    0x24    ;20 - 16 = 4  -> 10 0100 = 0x24
ICSS_INTC_PRU_SYS_EVT_21    .set    0x25    ;21 - 16 = 5  -> 10 0101 = 0x25
ICSS_INTC_PRU_SYS_EVT_22    .set    0x26    ;22 - 16 = 6  -> 10 0110 = 0x26
ICSS_INTC_PRU_SYS_EVT_23    .set    0x27    ;23 - 16 = 7  -> 10 0111 = 0x27
ICSS_INTC_PRU_SYS_EVT_24    .set    0x28    ;24 - 16 = 8  -> 10 1000 = 0x28
ICSS_INTC_PRU_SYS_EVT_25    .set    0x29    ;25 - 16 = 9  -> 10 1001 = 0x29
ICSS_INTC_PRU_SYS_EVT_26    .set    0x2A    ;26 - 16 = 10 -> 10 1010 = 0x2A
ICSS_INTC_PRU_SYS_EVT_27    .set    0x2B    ;27 - 16 = 11 -> 10 1011 = 0x2B
ICSS_INTC_PRU_SYS_EVT_28    .set    0x2C    ;28 - 16 = 12 -> 10 1100 = 0x2C
ICSS_INTC_PRU_SYS_EVT_29    .set    0x2D    ;29 - 16 = 13 -> 10 1101 = 0x2D
ICSS_INTC_PRU_SYS_EVT_30    .set    0x2E    ;30 - 16 = 14 -> 10 1110 = 0x2E
ICSS_INTC_PRU_SYS_EVT_31    .set    0x2F    ;31 - 16 = 15 -> 10 1111 = 0x2F

;----------------- End of INTC PRU System Event Interrupts -----------------;

    .endif ;____icss_intc_regs_h
