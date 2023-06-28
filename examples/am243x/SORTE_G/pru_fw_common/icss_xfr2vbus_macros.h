


    .if !$isdefed("____icss_xfr2vbus_macros_h")
____icss_xfr2vbus_macros_h   .set    1

   .include "pru_reg_structs.h"
   .include "icss_xfer_defines.h"


;******************************************************************************
;
;   Macro: m_xfr2vbus_issue_read
;
;   Issue read command via XFR2VBUS widget
;
;   PEAK cycles: 
;       5 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r18:r20
;
;   Pseudo code:
;       Issue xout to xfr2vbus widget with programmed address, mode
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_RD0_XID>, <XFR2VBUS_RD1_XID>)
;       addr_low : lower 32-bit of 48-bit external address
;       addr_high : upper 16-bit of 48-bit external address;
;       mode : (0 - 4bytes read, 1 - 4bytes read autoincrement, 4 - 32bytes read,  
;               5 - 32bytes read autoincrement 6 - 64bytes read, 7 - 64bytes autoincrement)
;   Returns:
;       r2:r9
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_issue_read .macro xid, addr_low, addr_high, addr_mode
wait_till_read_busy?:
    m_xfr2vbus_get_read_status xid
    qbbs wait_till_read_busy?,  r18, 0
    ldi r18.w0, addr_mode
    ldi32 r19, addr_low
    ldi r20.w0, addr_high
    xout xid, &r18, 12
    .endm

;******************************************************************************
;
;   Macro: m_xfr2vbus_read4
;
;   Read 4 bytes data via XFR2VBUS widget instance
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       r2:r5
;
;   Pseudo code:
;       xin from accelerator
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_RD0_XID>, <XFR2VBUS_RD1_XID>)
;
;   Returns:
;       r2:r5
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_read32 .macro xid
    xin xid, &r2, 4
    .endm

;******************************************************************************
;
;   Macro: m_xfr2vbus_read32
;
;   Read 32 bytes data via XFR2VBUS widget instance
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
;       xin from accelerator
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_RD0_XID>, <XFR2VBUS_RD1_XID>)
;
;   Returns:
;       r2:r9
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_read32 .macro xid
    xin xid, &r2, 32
    .endm

;******************************************************************************
;
;   Macro: m_xfr2vbus_read64
;
;   Read 64 bytes data via XFR2VBUS widget instance
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
;       xin from accelerator
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_RD0_XID>, <XFR2VBUS_RD1_XID>)
;
;   Returns:
;       r2:r17
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_read64 .macro xid
    xin xid, &r2, 64
    .endm

;******************************************************************************
;
;   Macro: m_xfr2vbus_get_read_status
;
;   Read status of last read transaction
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
;       xin from accelerator
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_RD0_XID>, <XFR2VBUS_RD1_XID>)
;
;   Returns:
;       r18.b0
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_get_read_status .macro xid
    xin xid, &r18, 1
    .endm

;******************************************************************************
;
;   Macro: m_xfr2vbus_poll_read_ready
;
;   Poll till read data is available in FIFO
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
;       xin from accelerator
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_RD0_XID>, <XFR2VBUS_RD1_XID>)
;
;   Returns:
;       r18.b0
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_poll_read_ready .macro xid
wait_till_read_done?:
    xin xid, &r18, 1
    qbbc wait_till_read_done?, r18.b0, 2
    .endm

;******************************************************************************
;
;   Macro: m_xfr2vbus_write32
;
;   Issue write command via XFR2VBUS widget for 32 bytes
;
;   PEAK cycles: 
;       4 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r2..r9, r10:r11
;
;   Pseudo code:
;       Issue xout to xfr2vbus widget with programmed address
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_WR0_XID>, <XFR2VBUS_WR1_XID>)
;       addr_low : lower 32-bit of 48-bit external address
;       addr_high : upper 16-bit of 48-bit external address;
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_write32 .macro xid, addr_low, addr_high
    ldi32 r10, addr_low
    ldi r11.w0, addr_high
    xout xid, &r2, 40
    .endm

;******************************************************************************
;
;   Macro: m_xfr2vbus_write64
;
;   Issue write command via XFR2VBUS widget for 64 bytes
;
;   PEAK cycles: 
;       5 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r2..r17, r18:r19
;
;   Pseudo code:
;       Issue xout to xfr2vbus widget with programmed address
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_WR0_XID>, <XFR2VBUS_WR1_XID>)
;       addr_low : lower 32-bit of 48-bit external address
;       addr_high : upper 16-bit of 48-bit external address;
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_write64 .macro xid, addr_low, addr_high
    ldi32 r18, addr_low
    ldi r19.w0, addr_high
    xout xid, &r2, 72
    .endm

;******************************************************************************
;
;   Macro: m_xfr2vbus_set_address
;
;   Set address for writing 1, 4, 8 or 32 bytes via XFR2VBUS widget instance
;
;   PEAK cycles: 
;       4 cycles
;
;   Invokes: 
;       None
;
;   Registers:
;       r10:r11
;
;   Pseudo code:
;       Issue xout to xfr2vbus widget with programmed address
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_WR0_XID>, <XFR2VBUS_WR1_XID>)
;       addr_low : lower 32-bit of 48-bit external address
;       addr_high : upper 16-bit of 48-bit external address;
;
;   Returns:
;       r2:r5
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_set_address .macro xid, addr_low, addr_high
wait_till_write_busy?:
    m_xfr2vbus_get_write_status xid
    qbbs wait_till_write_busy?,  r20, 0
    ldi32 r10, addr_low
    ldi r11.w0, addr_high
    xout xid, &r10, 6
    .endm

;******************************************************************************
;
;   Macro: m_xfr2vbus_writeN
;
;   Issue write command via XFR2VBUS widget for 1, 4, 8 or 32 bytes
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
;       Issue xout to xfr2vbus widget to execute write <= 32 bytes
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_WR0_XID>, <XFR2VBUS_WR1_XID>)
;       length : 1 or 4 or 8 or 32
;       r2:rx : Contains the data to write (depending on size)
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_writeN .macro xid, length
    xout xid, &r2, length
    .endm



;******************************************************************************
;
;   Macro: m_xfr2vbus_writeNAlign 
;
;   Issue write command via XFR2VBUS widget for 1, 4, 8 or 32 bytes
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
;       Issue xout to xfr2vbus widget to execute write <= 32 bytes
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_WR0_XID>, <XFR2VBUS_WR1_XID>)
;       x : start register
;       length : 1 or 4 or 8 or 32
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_writeNAlign .macro xid, x, length
    xout xid, &x, length
    .endm


;******************************************************************************
;
;   Macro: m_xfr2vbus_get_write_status
;
;   Read status of last write transaction
;
;   PEAK cycles: 
;       1 cycle
;
;   Invokes: 
;       None
;
;   Registers:
;       r20.b0
;
;   Pseudo code:
;       xin from accelerator
;
;   Parameters: 
;       xid : instance (<XFR2VBUS_WR0_XID>, <XFR2VBUS_WR1_XID>)
;
;   Returns:
;       r20.b0
;
;   See Also:
;
;******************************************************************************
m_xfr2vbus_get_write_status .macro xid
    xin xid, &r20, 1
    .endm    
    .endif ;____icss_xfr2vbus_macros_h 
