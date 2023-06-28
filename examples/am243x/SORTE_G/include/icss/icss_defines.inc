; Copyright (C) 2016 Texas Instruments Incorporated - http:;www.ti.com/
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; Redistributions of source code must retain the above copyright
; notice, this list of conditions and the following disclaimer.
;
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
; file:   icss_defines.inc
;
; brief:  {ICSS Specific Defines}
;
;
;  (C) Copyright 2016, Texas Instruments, Inc
;
;  author     {Thomas Leyrer, Thomas Mauer}
;
;  version    0.1     Created

	.if ! $defined( __icss_defines_inc)
	.define "1", __icss_defines_inc

; Bank ids for Xfer instructions
BANK0_ID        .set		10		; scratch pad R2-R5 is permanent for cut-through
BANK1_ID        .set		11		; R20-R23 is permanent for rcv context
BANK2_ID        .set		12		; XFR with other PRU
BANK3_ID        .set        13      ;
RXL2_BANK0_ID	.set		20      ; RX_L2 Fifo bank 0
RXL2_BANK1_ID	.set		21      ; RX_L2 Fifo bank 1
TXL2_ID         .set        40      ; TX L2 XFER
TM_YIELD_ID     .set 252 ; task manager end task
RANGE_R18       .set    1       ; R18.b0
RANGE_R2_R18 	.set   (4*16+1)	; only up to R18.b0
RANGE_R2_R13	.set   (4*12)
RANGE_R2_R9		.set   (4*8)
RANGE_R10_R13	.set   (4*4)
RANGE_R0_R19 .set           (4*20)
RXL2_SIDEA          .set	20
RXL2_SIDEB          .set	21
TXL2_XID            .set	40
CS                  .set	38
XFR2VBUS_XID_READ0  .set	0x60
xfr2vbus_rd0        .set	0x60
xfr2vbus_wr0        .set 	0x62
XFR2VBUS_XID_READ1  .set 	0x61
XFR2VBUS_XID_WRITE  .set 	0x62
xfr2vbus_wr0        .set 	0x62

PSI_PAT             .set	0x10
RFC_ID				.set 	15
BANK0_ID			.set	10
BANK1_ID			.set 	11
FILTER_ID           .set 	22 
TM_YIELD_XID        .set    252

	.if	$isdefed("SWITCH_CFG")
	.if	$isdefed("PRU0")
MII_CARRIER_SENSE_REG	.set  ICSS_MIIRT_PRS1
	.else
MII_CARRIER_SENSE_REG	.set  ICSS_MIIRT_PRS0
	.endif
	.else
	.if	$isdefed("PRU0")
MII_CARRIER_SENSE_REG	.set  ICSS_MIIRT_PRS0
	.else
MII_CARRIER_SENSE_REG	.set  ICSS_MIIRT_PRS1
	.endif
	.endif

	.endif
