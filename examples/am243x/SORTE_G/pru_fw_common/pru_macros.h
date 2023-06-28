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
; file:     pru_macros.h
;
; brief:    Macro definitions
;           Includes:
;           1.
;

    .if !$isdefed("____pru_macros_h")
____pru_macros_h   .set    1

    .include "pru_reg_structs.h"
    .include "pru_constant_defines.h"
    .include "icss_xfer_defines.h"


;************************************************************************************
;
;   Macro: m_reset_rx_fifo
;
;  Reset RX FIFOs using R31 command interface
;
;   PEAK cycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R31
;
;   Pseudo code:
;      > R31 = (1 << R31_CMD_RX_RESET_bit)
;
;   Parameters:
;      None
;
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_reset_rx_fifo .macro
    set r31, r31, R31_CMD_RX_RESET_bit
    .endm

;************************************************************************************
;
;   Macro: m_rx_l2_done_n_eof
;
;  Issue RX L2 done event to release back pressure from RX L1 and clear RX EOF.
;
;   PEAK cycles:
;       3 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       R11
;
;   Pseudo code:
;      > Clear RX EOF using R31 command.
;      > Set R11.t31
;      > xout R11.b3 to RXL2_FILTER_XID.
;
;   Parameters:
;      None
;
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_rx_l2_done_n_eof .macro
    set r31, r31, R31_CMD_RX_EOF_CLR_bit
    set r11, r11, 31
    xout RXL2_FILTER_XID, &R11.b3, 1
    .endm

;************************************************************************************
;
;   Macro: m_rx_l2_done
;
;  Issue RX L2 done event to release back pressure from RX L1.
;
;   PEAK cycles:
;       2 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       R11
;
;   Pseudo code:
;      > Set R11.t31
;      > xout R11.b3 to RXL2_FILTER_XID.
;
;   Parameters:
;      None
;
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_rx_l2_done .macro
    set r11, r11, 31
    xout RXL2_FILTER_XID, &R11.b3, 1
    .endm



;************************************************************************************
;
;   Macro: m_reset_tx_fifo
;
;  Reset TX FIFOs using R31 command interface
;
;   PEAK cycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R31
;
;   Pseudo code:
;       > R31 = (1 << R31_CMD_TX_RESET_bit)
;
;   Parameters:
;      None
;
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_reset_tx_fifo .macro
    set r31, r31, R31_CMD_TX_RESET_bit
    .endm


;************************************************************************************
;
;   Macro: m_reset_rx_fifo_n_eof
;
;  Reset RX FIFO and clear EOF using R31 command interface
;
;   PEAK cycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R31
;
;   Pseudo code:
;      > R31 = (1 << R31_CMD_RX_RESET_bit) |  (1 << R31_CMD_RX_EOF_CLR_bit)
;
;   Parameters:
;      None
;
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_reset_rx_fifo_n_eof .macro
    ;reset RX (Bit 18) and clear EOF(Bit 22)
    or r31.b2, r31.b2, 0x44
    .endm


;************************************************************************************
;
;   Macro: m_store_bytes_n_advance
;
;  Store num_bytes to (*c28+write_ptr) offset from register file (reg_src)
;
;   PEAK cycles:
;      2+(num_bytes/4)
;
;   Invokes:
;       None
;
;   Registers:
;
;
;   Pseudo code:
;      (start code)
;       for (i=0; i < num_bytes; i++) {
;           *(uint8_t*)(c28+write_ptr+i) = *(uint8_t*)(&reg_src);
;       }
;       write_ptr += num_bytes;
;      (end code)
;
;   Parameters:
;      reg_src : First register in the register file
;      write_ptr : Write pointer register (Offset in shared memory)
;      num_bytes : Number of bytes to write (32 or b0)
;
;   Returns:
;      None
;
;   See Also:
;
;************************************************************************************
m_store_bytes_n_advance .macro reg_src, write_ptr, num_bytes
    .var tmp
    .asg :num_bytes(1):, tmp ; extract first char to tmp
    .if $symcmp(tmp,"b") = 0 ;variable length : .b0, .b1, .b2, .b3
    sbco &reg_src, c28, write_ptr, num_bytes; r20.w0 is read pointer
    add write_ptr, write_ptr, r0.:num_bytes: ; Increment bytes read
    .else ; fixed length : 1..255
    sbco &reg_src, c28, write_ptr, num_bytes; r20.w0 is read pointer
    add write_ptr, write_ptr, num_bytes; Increment bytes read
    .endif
    .endm

    .endif ; ____pru_macros_h
