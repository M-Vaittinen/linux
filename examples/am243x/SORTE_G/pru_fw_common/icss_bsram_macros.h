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
; file:     icss_bsram_macros.h
;
; brief:    PRU broadside RAM related macro definitions
;           Includes:
;           1. 
;


    .if !$isdefed("____icss_bsram_macros_h")
____icss_bsram_macros_h   .set    1

    .include "pru_reg_structs.h"
    .include "icss_xfer_defines.h"

;******************************************************************************
;
;   Macro: m_bsram_set_address
;
;   Configure address to read/write and autoincrement mode in BS RAM 
;
;   PEAK cycles: 
;       2 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r10.w0
;
;   Pseudo code:
;       r10.w0 = (blk idx | autoincrement - optional)
;       xout 10.w0
;
;   Parameters: 
;       xid      :   <RTU_BSRAM_XID>/<RTU_BSRAM_SUM32_XID>/<PRU_BSRAM_XID>/<PRU_BSRAM_SUM32_XID>
;       blk_idx  :   Block index 9-bit. Set Bit15 for autoincrement
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************    
m_bsram_set_address .macro xid, blk_idx    
    ldi r10.w0, blk_idx
    xout xid, &r10, 2
    .endm        

;******************************************************************************
;
;   Macro: m_bsram_read
;
;   Read len bytes from index (32-bytes block) in BS RAM 
;
;   PEAK cycles: 
;       3 cycles
;
;   Invokes: 
;       <m_bsram_set_address>
;
;   Registers:
;       r10.w0
;
;   Pseudo code:
;       r10.w0 = (blk idx | autoincrement - optional)
;       xout 10.w0
;       xin len bytes from BS RAM
;
;   Parameters: 
;       xid      :   <RTU_BSRAM_XID>/<RTU_BSRAM_SUM32_XID>/<PRU_BSRAM_XID>/<PRU_BSRAM_SUM32_XID>
;       blk_idx  :   Block index 9-bit. Set Bit15 for autoincrement
;       len      :   Number of bytes or r0.b0/r0.b1/r0.b2/r0.b3 (1..32)
;
;   Returns:
;       r2:r9 (max)
;
;   See Also:
;
;******************************************************************************
m_bsram_read .macro xid, blk_idx, len
    m_bsram_set_address xid, blk_idx
    xin xid, &r2, len
    .endm

;******************************************************************************
;
;   Macro: m_bsram_write32
;
;   Write 32 bytes to index (32-bytes block) in BS RAM 
;
;   PEAK cycles: 
;       2 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r2:r9, r10.w0
;
;   Pseudo code:
;       r10.w0 = (blk idx | autoincrement - optional)
;       xout r2:r10 to accelerator
;
;   Parameters: 
;       xid      :   <RTU_BSRAM_XID>/<RTU_BSRAM_SUM32_XID>/<PRU_BSRAM_XID>/<PRU_BSRAM_SUM32_XID>
;       blk_idx  :   Block index 9-bit. Set Bit15 for autoincrement
;
;   Returns:
;       r2:r9
;
;   See Also:
;
;******************************************************************************
m_bsram_write32 .macro xid, blk_idx
    ldi r10.w0, blk_idx
    xout xid, &r2, 34
    .endm

;******************************************************************************
;
;   Macro: m_bsram_write
;
;   Write len bytes to index (32-bytes block) in BS RAM assuming address is set 
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       r2:r9(max)
;
;   Pseudo code:
;       xout r2:rN to accelerator
;
;   Parameters: 
;       xid      :   <RTU_BSRAM_XID>/<RTU_BSRAM_SUM32_XID>/<PRU_BSRAM_XID>/<PRU_BSRAM_SUM32_XID>
;       len      :   Number of bytes or r0.b0/r0.b1/r0.b2/r0.b3 (1..32)
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_bsram_write .macro xid, len
    xout xid, &r2, len
    .endm

    .endif ;____icss_bsram_macros_h 