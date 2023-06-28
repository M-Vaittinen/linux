; Copyright (C) 2018 Texas Instruments Incorporated - http:;www.ti.com/
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; Redistributions of source code must retain the above copyright
; notice, this list of conditions and the following disclaimer.
;
; Redistributions in binary form must reproduce the above copyright
; notice, this list of conditions and the following disclaimer in the
; documentation and/or other materials provided with the
; distribution.
;
; Neither the name of Texas Instruments Incorporated nor the names of
; its contributors may be used to endorse or promote products derived
; from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;     file:  basicio.inc
;
;     brief:  basic rx macros for reception


;************************************* Includes *************************************
    .cdecls C,LIST,"basicio.h"

;---------------------------------------------------------------
;--------------------------RX BN COMBOS----------------------------
;---------------------------------------------------------------
;   COMBOS:  (18)
;   DROP_SIDEA
;   DROP_SIDEB
;   CUTTHRU SIDEA
;   CUTTHRU SIDEB
;   S&F SIDEA
;   S&F SIDEB
;   CUT + HOST SIDEA
;   CUT + HOST SIDEB
;   S&F + HOST SIDEA
;   S&F + HOST SIDEB
;   HOST SIDE A
;   HOST SIDE B
;   HOST+UDP SIDE A
;   HOST+UDP SIDE B
;   CUT + HOST + UDP SIDEA
;   CUT + HOST + UDP SIDEB
;   S&F + HOST + UDP SIDEA
;   S&F + HOST + UDP SIDEB


;BN RX + DROP +SIDEA
;clobbers: R18, R2-R9,
BN_RX_DROP  .macro sidereg
     xin    RXL2_SIDEA,&r18,2   ;read len
     QBGT   not_enuf?,R18,32    ;if 32 > write ptr branch(keep waiting until bank0 full or eof))
     xin    RXL2_SIDEA,&r2,32
     SET    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm

;BN RX + DROP + SIDEB
;clobbers: R18, R2-R9,
BN_RX_DROP_SIDEB    .macro  sidereg
     xin    RXL2_SIDEB,&r18,2   ;read len
     QBLE   not_enuf?,R18,32    ;if 32 <= write ptr  then branch(keep waiting until bank1 full or eof)
     xin    RXL2_SIDEB,&r2,32
     CLR    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm


;BN RX + CUTTHRU + SIDEA
;clobbers: R18, R2-R9,
BN_RX_CUT_SIDEA     .macro sidereg
     ;xin RXL2_SIDEA,&r18,2     ;read len
    ; QBGT not_enuf?,R18,32     ;if 32 > write ptr branch(keep waiting until bank0 full or eof))
     xin    RXL2_SIDEA,&r2,32
     xout   TXL2,&r2,32
     SET    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm

;BN RX +CUTTHRU + SIDEB
;clobbers: R18, R2-R9,
BN_RX_CUT_SIDEB     .macro sidereg
     ;xin RXL2_SIDEB,r18,2      ;read len
     ;QBLE not_enuf?,R18,32     ;if 32 <= write ptr  then branch(keep waiting until bank1 full or eof)
     xin    RXL2_SIDEB,&r2,32
     xout   TXL2,&r2,32
     CLR    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm

;BN RX + S&F + SIDEA
;clobbers: R18, R2-R9,
; R10:R11 = address
BN_RX_SF_SIDEA     .macro sidereg
     xin    RXL2_SIDEA,&r18,2    ;read len
     QBGT   not_enuf?,R18,32     ;if 32 > write ptr branch(keep waiting until bank0 full or eof))
     xin    RXL2_SIDEA,&r2,32
     ;OUT TO port queue
     xout   XFR2VBUS_XID_WRITE,&r2,40
     ADD    R10,R10,32
     SET    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm

;BN RX +S&F + SIDEB
; R10:R11 = address
;clobbers: R18, R2-R9,
BN_RX_SF_SIDEB      .macro
     xin    RXL2_SIDEB,&r18,2    ;read len
     QBLE   not_enuf?,R18,32     ;if 32 <= write ptr  then branch(keep waiting until bank1 full or eof)
     xin    RXL2_SIDEB,&r2,32
     ;OUT TO port queue
     xout   XFR2VBUS_XID_WRITE,&r2,40
     ADD    R10,R10,32
     CLR    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm


;BN RX + TOHOST + SIDEA
;clobbers: R1 R18, R2-R9,
BN_RX_HOST_SIDEA    .macro sidereg
     xin    RXL2_SIDEA,&r18,2     ;read len
     QBGT   not_enuf?,R18,32      ;if 32 > write ptr branch(keep waiting until bank0 full or eof))
     xin    RXL2_SIDEA,&r2,32
     ;out to host
     LDI    R1,PSI_PAT
     xout   XFR2PSI_XID,&R1,36
     SET    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm

;BN RX + TOHOST  + SIDEB
;clobbers: R1 R18, R2-R9,
BN_RX_HOST_SIDEB    .macro
     xin    RXL2_SIDEB,&r18,2     ;read len
     QBLE   not_enuf?,R18,32      ;if 32 <= write ptr  then branch(keep waiting until bank1 full or eof)
     xin    RXL2_SIDEB,&r2,32
     ;out to host
     LDI    R1,PSI_PAT
     xout   XFR2PSI_XID,&R1,36
     CLR    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm

;BN RX + S&F + HOST + UDP SIDEA
; R10:R11 = address
;clobbers:  R1 R18, R2-R9,
BN_RX_SF_HOST_CS_SIDEA  .macro  sidereg
     xin    RXL2_SIDEA,&r18,2     ;read len
     QBGT   not_enuf?,R18,32      ;if 32 > write ptr branch(keep waiting until bank0 full or eof))
     xin    RXL2_SIDEA,&r2,32
     ;out to host
     LDI    R1,PSI_PAT
     xout   XFR2PSI_XID,&R1,36
     ;CSum
     xout   CS,&r2,32
     ;OUT TO port queue
     xout   XFR2VBUS_XID_WRITE,&r2,40
     ADD    R10,R10,32
     SET    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm


;BN RX + S&F + HOST + UDP SIDEB
; R10:R11 = address
;clobbers:  R1 R18, R2-R9,
BN_RX_SF_HOST_SIDEB     .macro sidereg
     xin    RXL2_SIDEA,&r18,2   ;read len
     QBLE   not_enuf?,R18,32    ;if 32 <= write ptr  then branch(keep waiting until bank1 full or eof)
     xin    RXL2_SIDEA,&r2,32
     ;out to host
     LDI    R1,PSI_PAT
     xout   XFR2PSI_XID,&R1,36
     ;CSum
     xout   CS,&r2,32
     ;OUT TO port queue
     xout   XFR2VBUS_XID_WRITE,&r2,40
     ADD    R10,R10,32
     CLR    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm


;BN RX + CUT + HOST + UDP SIDEA
; R10:R11 = address
;clobbers:  R1 R18, R2-R9,
BN_RX_CUT_HOST_CS_SIDEA     .macro sidereg
     xin    RXL2_SIDEA,&r18,2   ;read len
     QBGT   not_enuf?,R18,32    ;if 32 > write ptr branch(keep waiting until bank0 full or eof))
     xin    RXL2_SIDEA,&r2,32
     ;out to host
     LDI    R1,PSI_PAT
     xout   XFR2PSI_XID,&R1,36
     ;CSum
     xout   CS,&r2,32
     ;CUT through
     xout   TXL2,&r2,32
     SET    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm


;BN RX + CUT + HOST + UDP SIDEB
;clobbers: R1 R18, R2-R9,
BN_RX_CUT_HOST_CS_SIDEB     .macro sidereg
     xin    RXL2_SIDEA,&r18,2   ;read len
     QBLE   not_enuf?,R18,32    ;if 32 <= write ptr  then branch(keep waiting until bank1 full or eof)
     xin    RXL2_SIDEA,&r2,32
     ;out to host
     LDI    R1,PSI_PAT
     xout   XFR2PSI_XID,&R1,36
     ;CSum
     xout   CS,&r2,32
     ;CUT through
     xout   TXL2,&r2,32
     CLR    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm

;---------------------------------------------------------------
;--------------------------B_RX_EOF COMBOS----------------------------
;---------------------------------------------------------------
;COMBOS:  (18)
; DROP_SIDEA
; DROP_SIDEB
;  CUTTHRU SIDEA
;  CUTTHRU SIDEB
;  S&F SIDEA
;  S&F SIDEB
;  CUT + HOST SIDEA
;  CUT + HOST SIDEB
;  S&F + HOST SIDEA
;  S&F + HOST SIDEB
;  HOST SIDE A
;  HOST SIDE B
;  HOST+UDP SIDE A
;  HOST+UDP SIDE B
;  CUT + HOST + UDP SIDEA
;  CUT + HOST + UDP SIDEB
;  S&F + HOST + UDP SIDEA
;  S&F + HOST + UDP SIDEB



;EOF RX + DROP +SIDEA
;clobbers: R18, R2-R9,
RX_EOF_DROP     .macro sidereg
     xin    RXL2_SIDEA,&r18,2   ;read len
     QBGT   not_enuf?,R18,32  ;if 32 > write ptr branch(keep waiting until bank0 full or eof))
     xin    RXL2_SIDEA,&r2,32
     SET    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm

;EOF RX + DROP + SIDEB
;clobbers: R18, R2-R9,
RX_EOF_DROP_SIDEB     .macro sidereg
     xin    RXL2_SIDEB,&r18,2   ;read len
     QBLE   not_enuf?,R18,32 ;if 32 <= write ptr  then branch(keep waiting until bank1 full or eof)
     xin    RXL2_SIDEB,&r2,32
     CLR    sidereg.b0, sidereg.b0, 0
not_enuf?:
     .endm

;B_EOF RX + CUTTHRU + SIDEA
;clobbers: R18, R2-R9,
RX_EOF_CUT_SIDEA    .macro
     xin RXL2_SIDEA,&r18,2   ;read len
     mvib r0.b0,r18.b0
     xin RXL2_SIDEA,&r2,b0
     xout  TXL2,&r2,b0
     .endm

;B_EOF RX +CUTTHRU + SIDEB
;clobbers: R18, R2-R9,
RX_EOF_CUT_SIDEB    .macro
     xin    RXL2_SIDEB,&r18,2   ;read len
     SUB    r18,r18,32
     mvib   r0.b0,r18.b0
     xin    RXL2_SIDEB,&r2,b0
     xout   TXL2,&r2,b0
     .endm


;------------------------------------------------------------------------
;-------------------------- (S&F) B_TX COMBOS----------------------------
;------------------------------------------------------------------------

;move 32 bytes from XFR2VBUS Read fifo to TXL2 fifo
BN_TX_32    .macro rnum,cnt_reg
    xin   rnum,&r2,32
    xout  TXL2,&r2,32
    add   cnt_reg,cnt_reg,32
    .endm

;move n  bytes from XFR2VBUS Read fifo to TXL2 fifo (n in .b0
BN_TX_n     .macro rnum,cnt_reg
    xin   rnum,&r2,32
    xout  TXL2,&r2,b0
    add   cnt_reg,cnt_reg,r0.b0
    .endm


