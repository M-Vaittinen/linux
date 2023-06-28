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
; file:     icss_xfr2psi_macros.h
;
; brief:    PRU xfr2psi related macro definitions
;           Includes:
;           1. 
;


    .if !$isdefed("____icss_xfr2psi_macros_h")
____icss_xfr2psi_macros_h   .set    1

    .include "pru_reg_structs.h"
    .include "icss_xfer_defines.h"


;******************************************************************************
;
;   Macro: m_xfr2psi_read
;
;   Read via PSI widget based on PSI status
;
;   PEAK cycles: 
;       5 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       None
;
;   Pseudo code:
;       Issue xout to xfr2psi widget with programmed address, mode
;
;   Parameters: 
;       psi_stat : psi status read from <XFR2PSI_STAT_XID>
;       len :  Number of bytes or r0.b0/r0.b1/r0.b2/r0.b3 (1..16)
;
;   Returns:
;       r1:r9(max)
;
;   See Also:
;
;******************************************************************************
m_xfr2psi_read .macro psi_stat, len
    qbbs m_xfr2psi_read1, psi_stat, 0
    qbbs m_xfr2psi_read2, psi_stat, 1
    qbbs m_xfr2psi_read3, psi_stat, 2
    qbbs m_xfr2psi_read4, psi_stat, 3
    qba m_xfr2psi_read_done
m_xfr2psi_read1:
    xin XFR2PSI_RD1_XID, &r1, len
    qba m_xfr2psi_read_done
m_xfr2psi_read2:
    xin XFR2PSI_RD2_XID, &r1, len
    qba m_xfr2psi_read_done
m_xfr2psi_read3:
    xin XFR2PSI_RD3_XID, &r1, len
    qba m_xfr2psi_read_done
m_xfr2psi_read4:
    xin XFR2PSI_RD4_XID, &r1, len
m_xfr2psi_read_done:
    .endm

;******************************************************************************
;
;   Macro: m_xfr2psi_read_thread
;
;   Read len bytes data via selected xfr2psi widget
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       None
;
;   Pseudo code:
;       xin from accelerator
;
;   Parameters: 
;       xid : instance (<XFR2PSI_RD1_XID>, <XFR2PSI_RD2_XID>, <XFR2PSI_RD2_XID>, <XFR2PSI_RD3_XID>)
;       len :  Number of bytes or r0.b0/r0.b1/r0.b2/r0.b3 (1..16)
;
;   Returns:
;       r2:r9(max)
;
;   See Also:
;
;******************************************************************************
m_xfr2psi_read_thread .macro xid, len
    xin xid, &r1, len
    .endm

;******************************************************************************
;
;   Macro: m_xfr2psi_write
;
;   Write len bytes data via xfr2psi widget instance
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
;       xout from accelerator
;
;   Parameters: 
;       len :  Number of bytes or r0.b0/r0.b1/r0.b2/r0.b3 (1..16)
;       r1:r10(max)
;
;   Returns:
;        None
;
;   See Also:
;
;******************************************************************************
m_xfr2psi_write .macro len
    xin XFR2PSI_WR1_XID, &r1, len
    .endm

;******************************************************************************
;
;   Macro: m_xfr2psi_get_status
;
;   Read status from XFR2PSI widget
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       None
;
;   Pseudo code:
;       xin from accelerator
;
;   Parameters: 
;       None
;
;   Returns:
;       r1
;
;   See Also:
;
;******************************************************************************
m_xfr2psi_get_read_status .macro xid
    xin XFR2PSI_STAT_XID, &r1, 4
    .endm
    .endif ;____icss_xfr2psi_macros_h 