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
; file:     icss_fdb_macros.h
;
; brief:    PRU FDB Related Macro definitions
;           Includes:
;           1. 
;


    .if !$isdefed("____icss_fdb_macros_h")
____icss_fdb_macros_h   .set    1

    .include "pru_reg_structs.h"
    .include "icss_xfer_defines.h"

;fdb entry:: 8 bytes  (4 of these per fdb)
fdbEntryStruct .struct
addr03 .uint
addr45 .ushort  
vlan .ubyte ;mapped from 12 to 8
flags .ubyte ;see below
    .endstruct
;host/port1/port2:: 
;for unicast, only one of host,port1,port2 can be set (indicates which port this address is located
;for multicast, can have multiple bits set == flood mask
fdb_flags_host .set 0
fdb_flags_port1 .set 1
fdb_flags_port2 .set 2
fdb_flags_ageable .set 3 ; Set : ageable Clear : static/permanent
fdb_flags_block .set 4
fdb_flags_secure .set 5
fdb_flags_touched .set 6 ; Set : recently seen
fdb_flags_valid .set 7 ; Set : valid

fdbResultsStruct .struct ; FDB port lookup result
fid .ubyte
fid_c1 .ubyte
fid_da_c2 .ubyte
fid_sa_c2 .ubyte
da_fdb_address_info .ushort ; 8-0:  address for xin  11-10:  which of the 4 slots
sa_fdb_address_info .ushort ; 8-0:  address for xin  11-10:  which of the 4 slots
res_flags .ubyte
fdb_c1  .set    0
fdb_da_c2 .set    1
fdb_da_match .set 2
fdb_sa_c2 .set    3
fdb_sa_match .set 4
spare0 .ubyte
spare1 .ubyte
spare2 .ubyte
    .endstruct
 
fdbResults .sassign r10, fdbResultsStruct
fdbEntry .sassign r14, fdbEntryStruct
fdbEntryBin1 .sassign r2, fdbEntryStruct
fdbEntryBin2 .sassign r4, fdbEntryStruct
fdbEntryBin3 .sassign r6, fdbEntryStruct
fdbEntryBin4 .sassign r8, fdbEntryStruct

;******************************************************************************
;
;   Macro: m_fdb_set_addr_n_config 
;
;  FDB set address and config
;
;   PEAK cycles: 
;       3 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r10.w2 : For address configuration
;
;   Pseudo code:
;       r10.w0 = addr | (mode << 8)
;       xout r10.w0 to FDB addr config XFRID  
;
;   Parameters: 
;       addr -> 9-bit FDB index (32-byte entries) 
;       mode -> 0x80 : Auto increment FDB index 0x00 : Normal 
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_fdb_set_addr_n_config .macro addr, mode
    mov  r10.w0, addr
    or   r10.b2, r10.b2, mode
    xout FDB_DATA_XID, &r10, 2
    .endm

;******************************************************************************
;
;   Macro: m_fdb_write_data_32b
;
;  Write 32 bytes of data to FDB
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
;       xout r2:r9 to FDB XFRID  
;
;   Parameters: 
;       r2:r9 contains 32 bytes to write
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_fdb_write_data_32b .macro
    xout FDB_DATA_XID, &r2, 32
.endm

;******************************************************************************
;
;   Macro: m_fdb_read_data_32b
;
;  Read 32 bytes of data from FDB
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
;       xin r2:r9 from FDB XFRID  
;
;   Parameters: 
;       None
;
;   Returns:
;       r2:r9 contains 32 bytes read
;
;   See Also:
;
;******************************************************************************
m_fdb_read_data_32b .macro
    xin FDB_DATA_XID, &r2, 32
.endm

;******************************************************************************
;
;   Macro: m_fdb_poll_lookup_complete
;
;  Read FDB port lookup result. Block till specified flag is set
;
;   PEAK cycles: 
;       1 + HW DA lookup time cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r10:r12
;
;   Pseudo code:
;       xin fdb results
;       block till DA lookup is completed
;
;   Parameters: 
;       port : XFRID of results (<FDB_P0_RES_XID>, <FDB_P1_RES_XID>, <FDB_HOST_RES_XID> )
;       flags : <fdb_c1>, <fdb_da_c2>, <fdb_da_match>, <fdb_sa_c2>, <fdb_sa_match>
;
;   Returns:
;       r10:r12 with results
;
;   See Also:
;
;******************************************************************************
m_fdb_poll_lookup_complete .macro port, mode
wait_till_da_ready?:
    xin port, &fdbResults, 12
    qbbc wait_till_da_ready?, fdbResults.res_flags, fdb_look_da_c2 
.endm

;******************************************************************************
;
;   Macro: m_fdb_read_port_result
;
;  Read FDB port result
;
;   PEAK cycles: 
;       1 cycle 
;
;   Invokes: 
;       None
;
;   Registers:
;       r10:r12
;
;   Pseudo code:
;       xin fdb results
;
;   Parameters: 
;       port : XFRID of results (<FDB_P0_RES_XID>, <FDB_P1_RES_XID>, <FDB_HOST_RES_XID> )
;
;   Returns:
;       r10:r12 with results
;
;   See Also:
;
;******************************************************************************
m_fdb_read_port_result .macro port
    xin port, &fdbResults, 12
.endm

;******************************************************************************
;
;   Macro: m_fdb_clear
;
;  Clear FDB RAM for given bank
;
;   PEAK cycles: 
;       260 cycles 
;
;   Invokes: 
;       None
;
;   Registers:
;       r2:r9, r10
;
;   Pseudo code:
;       xin fdb results
;
;   Parameters: 
;       addr : Set 0 for bank0 and 0x100 for bank1
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_fdb_clear .macro addr
    ldi  r10.w0, (0x8000 | addr )
    xout FDB_DATA_XID, &r10, 2 
    zero &r2, 32
    loop m_fdb_clear_endloop?, 256
    xout FDB_DATA_XID, &r2, 32
m_fdb_clear_endloop?:         
    .endm
 
;******************************************************************************
;
;   Macro: m_fdb_bin4_update
;
;  Find free entry (with bin size of 4) and update
;  fid0,fid1 are 8 bytes of fid (properly formatted) to be saved
;  Assumption: index set already and not autoicrement
;  fid entry assumed to be FDB_HBIN_REG0, FDB_HBIN_REG1  (r14:r15)
;  entry can't be permanent (use FDB_FIND_FREE_AND_UPDATE4_PERM for this case)
;  Aging is not supported
;
;   PEAK cycles: 
;       7 cycles 
;
;   Invokes: 
;       None
;
;   Registers:
;       r2:r9
;
;   Pseudo code:
;      xin FDB entries (4 bins - 32 bytes) - address is already set
;      skip to next entry if fdb_flags_valid is set
;      Update first entry which is not valid
;      if all three entries are valid overwrite last entry (assumption last entry is not static)
;
;   Parameters: 
;       fdbEntry (r14:r15)
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_fdb_bin4_update .macro fdbEntry
    xin FDB_DATA_XID, &r2, 32
    qbbs m_fdb_update_skip1?, fdbEntryBin1.flags, fdb_flags_valid
    mov fdbEntryBin1.addr03, fdbEntry.addr03
    mov r3, r15
    xout FDB_DATA_XID, &r2, 32
    qba m_fdb_update_done?
m_fdb_update_skip1?:
    qbbs m_fdb_update_skip2?, fdbEntryBin2.flags, fdb_flags_valid
    mov fdbEntryBin2.addr03, fdbEntry.addr03
    mov r5, r15
    xout FDB_DATA_XID, &r2, 32
    qba m_fdb_update_done?
m_fdb_update_skip2?:
    qbbs m_fdb_update_skip3?, fdbEntryBin3.flags, fdb_flags_valid
    mov fdbEntryBin3.addr03, fdbEntry.addr03
    mov r7, r15
    xout FDB_DATA_XID, &r2, 32
    qba m_fdb_update_done?
m_fdb_update_skip3?:
;(note: need to guarantee that entry3 can never be a permanent entry!)
    mov fdbEntryBin4.addr03, fdbEntry.addr03
    mov r9, r15
    xout FDB_DATA_XID, &r2, 32
m_fdb_update_done?:
    .endm
    .endif ;____icss_fdb_macros_h 