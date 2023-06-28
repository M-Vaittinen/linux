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
; file:     icss_xfr2tr_macros.h
;
; brief:    PRU xfr2tr related macro definitions
;           Includes:
;           1. 
;


    .if !$isdefed("____icss_xfr2tr_macros_h")
____icss_xfr2tr_macros_h   .set    1

    .include "pru_reg_structs.h"
    .include "icss_xfer_defines.h"


;******************************************************************************
;
;   Macro: m_xfr2tr_setup
;
;   Setup 8/64 bytes TR source pool and ring of ring_size 
;
;   PEAK cycles: 
;       8 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r6:r7
;
;   Pseudo code:
;       r6 = master_base & r7.b0 = tr_size
;       configure master source pool
;       r6 = ring_base & r7.b0 = 2 (ring reset) 
;       r7.w1 = ring_size
;       configure ring TR  
;
;   Parameters: 
;       master_base : Base address of master TR in ICSS shared memory (18-bit)
;       ring_base : Base address of ring TR in ICSS shared memory (18-bit)
;       ring_size : Number of TRs in the ring (12-bit)
;       tr_size : (0 : 32 bytes 1 : 64 bytes)
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_xfr2tr_setup .macro master_base, ring_base, ring_size, tr_size
    ldi32 r6, master_base
    ldi r7.b0, (2 | tr_size) ; Reset Ring (bit1), TR size (bit0)  
    xout XFR2TR_MASTER_XID, &r6, 5 ; Reset Ring is don't care here
    ldi32 r6, ring_base
    ldi r7.w1, ring_size
    xout XFR2TR_RING_XID, &r6, 7 ; TR size is don't care
    .endm

;******************************************************************************
;
;   Macro: m_xfr2tr_submit
;
;   Submit/copy num_trs TRs from master source pool to ring TR
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
;       xout TRs to the widget
;
;   Parameters: 
;       num_trs : Number of TRs (1..8) or Number of bytes in r0.b0/r0.b1/r0.b2/r0.b2
;       TRs : r2.w0, r2.w2, r3.w0, r3.w2, ..., r5.w0, r5.w2
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_xfr2tr_submit .macro num_trs
    .var tmp
    .asg :num_trs(1):, tmp ; extract first char to tmp
    .if $symcmp(tmp,"b") = 0 ;variable length : .b0, .b1, .b2, .b3
    xout XFR2TR_SUBMIT_XID, &r2, num_trs
    .else
    xout XFR2TR_SUBMIT_XID, &r2, (num_trs << 1)
    .endif
    .endm

;******************************************************************************
;
;   Macro: m_xfr2tr_get_ring_status
;
;   Read status of ring submit FIFO
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       r18.b0
;
;   Pseudo code:
;       xin from XFR2TR_RING_XID
;
;   Parameters: 
;       None
;
;   Returns:
;       r7.w0 (r7.b1 : occupancy of FIFO, r7.bit0 FIFO busy : 1, FIFO free : 0)
;
;   See Also:
;
;******************************************************************************
m_xfr2tr_get_ring_status .macro
    xin XFR2TR_RING_XID, &r7, 2
    .endm

    .endif ;____icss_xfr2tr_macros_h 