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
; file:     icss_filter_macros.h
;
; brief:    PRU RXL2 filter Related Macro definitions
;           Includes:
;           1. 
;


    .if !$isdefed("____icss_filter_macros_h")
____icss_filter_macros_h   .set    1

    .include "pru_reg_structs.h"
    .include "icss_xfer_defines.h"

filterResultStruct .struct
cur_byte .uint
fflags0 .uint
flags_f1 .uint
flags_f2 .uint
flags_f3 .uint
sa_hash .uint
conn_hash .uint
rate .uint
    .endstruct
filterResults .sassign r10, filterResultStruct

;******************************************************************************
;
;   Macro: m_filter_get_status 
;
;  Read filter status
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       r10:R17
;
;   Pseudo code:
;       xin filterResults
;
;   Parameters: 
;       None   
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_filter_get_status .macro
    xin RXL2_FILTER_XID, &r10, 32
    .endm

    .endif ;____icss_filter_macros_h 