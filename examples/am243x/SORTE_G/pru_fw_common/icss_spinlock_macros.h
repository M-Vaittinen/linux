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
; file:     icss_spinlock_macros.h
;
; brief:    PRU spinlock related macro definitions
;           Includes:
;           1. 
;


    .if !$isdefed("____icss_spinlock_macros_h")
____icss_spinlock_macros_h   .set    1

    .include "pru_reg_structs.h"
    .include "icss_xfer_defines.h"

;******************************************************************************
;
;   Macro: m_spinlock_acquire
;
;   Acquire spinlock
;
;   PEAK cycles: 
;       2 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r1
;
;   Pseudo code:
;       r1.b0 = spinlock_number
;       xin from accelerator
;
;   Parameters: 
;       spinlock : spinlock instance (<INT_SPIN_XID>, <EXT0_SPIN_XID>, <EXT1_SPIN_XID>)
;       number :   which one out of 64 shared resources (0..63)
;
;   Returns:
;       r1.b3
;
;   See Also:
;
;******************************************************************************
m_spinlock_acquire .macro spinlock, number
    ldi r1.b0, number
    xin spinlock, &r1.b3, 1
    .endm
;******************************************************************************
;
;   Macro: m_spinlock_acquire_branch_on_fail
;
;   Acquire spinlock and branch to supplied label on failure
;
;   PEAK cycles: 
;       3 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r1
;
;   Pseudo code:
;       r1.b0 = spinlock_number
;       xin from accelerator
;       check acquired status in r1.b3 bit0, if not branch to label   
;
;   Parameters: 
;       spinlock : spinlock instance (<INT_SPIN_XID>, <EXT0_SPIN_XID>, <EXT1_SPIN_XID>)
;       number :  which one out of 64 shared resources (0..63)
;       label :  label of fail handler
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_spinlock_acquire_branch_on_fail .macro spinlock, number, label
    ldi r1.b0, number
    xin spinlock, &r1.b3, 1
    qbbc label, r1, 24 ;TODO : quick branch won't work for large offsets !
    .endm

;******************************************************************************
;
;   Macro: m_spinlock_release
;
;   Release spinlock, assumption is that r1.b0 already has spinlock_number initialized
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
;       xout r1.b1 to spinlock to release
;
;   Parameters: 
;       spinlock : spinlock instance (<INT_SPIN_XID>, <EXT0_SPIN_XID>, <EXT1_SPIN_XID>)
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_spinlock_release .macro spinlock
    xout spinlock, &r1.b1, 1
    .endm
    .endif ;____icss_spinlock_macros_h 