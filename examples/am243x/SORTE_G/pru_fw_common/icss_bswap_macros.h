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
; file:     icss_bswap_macros.h
;
; brief:    PRU byteswap related Macro definitions
;           Includes:
;           1. 
;


    .if !$isdefed("____icss_bswap_macros_h")
____icss_bswap_macros_h   .set    1

    .include "pru_reg_structs.h"
    .include "icss_xfer_defines.h"

;******************************************************************************
;
;   Macro: m_byte_swap
;
;  Do little endian to big endian or vice versa swap
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       regstart: regstart+size
;
;   Pseudo code:
;       xin to accelerator
;
;   Parameters: 
;       regstart : Starting register
;       size :    Number of registers to do byte swap
;
;   Returns:
;       regstart: regstart+size
;
;   See Also:
;
;******************************************************************************
m_byte_swap .macro regstart, size
    xin BSWAP_XID, &regstart, size
    .endm

;******************************************************************************
;
;   Macro: m_pivot_4_8
;
;   Swap r2-r5 with r6-r9
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       r2:r9
;
;   Pseudo code:
;       xin to accelerator
;
;   Parameters: 
;       None
;
;   Returns:
;       r2:r9
;
;   See Also:
;
;******************************************************************************
m_pivot_4_8 .macro
    xin BSWAP_4_8_XID, &r2, 32
    .endm

;******************************************************************************
;
;   Macro: m_pivot_4_16
;
;   Swap r2-r5 with r14-r17 and swap r6-r9 with r10-r13
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       r2:r17
;
;   Pseudo code:
;       xin to accelerator
;
;   Parameters: 
;       None
;
;   Returns:
;       r2:r17
;
;   See Also:
;
;******************************************************************************
m_pivot_4_16 .macro
    xin BSWAP_4_16_XID, &r2, 64
    .endm
    .endif ;____icss_bswap_macros_h 