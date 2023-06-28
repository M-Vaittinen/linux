
    .cdecls C,LIST,"xfr2vbus_widget.h"
;read in read result (32)
XFR2VBUS_READ32_RESULT .macro rnum
    xin rnum,&r2,32
    .endm

;issue a oneshot read of 32 bytes from addrss _lo|_hi
; steps on R18,R19,R20
XFR2VBUS_ISSUE_READ_ONESHOT_32 .macro rnum,addr_lo,addr_hi
    ldi r20,addr_hi
    ldi r19,addr_lo
    ldi r18,0x4  ; [1.2] = 2 => 32 bytes
    add r0,r0,0 ;nop - see instability in r18 ..
    xout rnum,&r18,12
    .endm

;issue a oneshot read for size 1
;issue a oneshot read for size 4
;issue a oneshot read of 64 bytes from addrss _lo|_hi
; steps on R18,R19,R20
XFR2VBUS_ISSUE_READ_ONESHOT_64 .macro rnum,addr_lo,addr_hi
    ldi r20,addr_hi
    ldi r19,addr_lo
    ldi r18,0x6     ; [1.2] = 3 => 64 bytes
    add r0,r0,0     ;   nop - see instability in r18 ..
    xout rnum,&r18,12
    .endm



;poll for read complete
;returns in r18.tb2 the result
XFR2VBUS_POLL_READ .macro rnum
     xin rnum,&r18,4
     .endm


;issue a read of 64 bytes from addrss _lo|_hi ,auto mode
;clobbers r18-r20
XFR2VBUS_ISSUE_READ_AUTO_64_CMD .macro rnum,addr_lo,addr_hi
     mov r20,addr_hi
     mov r19,addr_lo
     mov r18, 0x7  ;(0x1 | 0x3>>1)
     xout rnum,12
     .endm

;issue cmd to stop read auto mode
XFR2VBUS_CANCEL_READ_AUTO_64_CMD .macro  rnum
     mov r18,0
     xout rnum,&r18,4
     .endm


;read in data64
; fills in R2-R17
XFR2VBUS_READ64_RESULT .macro rnum
     xin rnum,&r2,64
     .endm

;read in data8
; fills in R2-R3
XFR2VBUS_READ8_RESULT .macro  rnum
     xin rnum,&r2,8
     .endm

;read in data4
; fills in R2
XFR2VBUS_READ4_RESULT .macro rnum
     xin rnum,&r2,4
     .endm

;set auto mode for writes
XFR2VBUS_SET_WRITE_AUTO .macro
    ldi r20,1
    xout XFR2VBUS_XID_WRITE,&r20,2
    .endm

;clear auto mode for writes
XFR2VBUS_CLEAR_WRITE_AUTO .macro
     ldi R20,0
     xout XFR2VBUS_XID_WRITE,&r20,2
     .endm

;issue a write of 32 bytes from address _lo|_hi, one shot
; data in r2-r9
; clobbers r10,r11
XFR2VBUS_WRITE32 .macro  xid,addr_lo,addr_hi
    mov r10,addr_lo
    ldi r11,addr_hi
    xout xid,&r2,40
    .endm

;issue a write of 64 bytes from address _lo|_hi, one shot
; data in r2-r17, r18+r19 = addr
; widget detects 64B from length
XFR2VBUS_WRITE64 .macro  xid,addr_lo,addr_hi
     mov r18,addr_lo
     ldi r19,addr_hi
     xout xid,&r2,72
     .endm

;.if $defined(OLD) ;not needed after nov tweak
;XFR2VBUS_SET_WRITE64 .macro
;    ldi r18,2  ;64B mode
;    xout XFR2VBUS_XID_WRITE,&r18,4
;.endif



;poll for write complete
;returns in r20.tb2 the result
XFR2VBUS_POLL_WRITE .macro xid
     xin xid,&r20,4
     .endm

WRI_CHK_XFR2VBUS  .macro xid
rtu_chk_write?:
     XFR2VBUS_POLL_WRITE xid
     qbne    rtu_chk_write?, r20.b0, 0
     .endm

