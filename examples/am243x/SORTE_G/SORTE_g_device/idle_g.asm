; Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
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
;     file:   discovery.asm
;
;     brief:  AM654x Register Test Source
;
;
;    Version        Description                                Author
;     0.1         Created                                      Thomas Mauer

;************************************* Includes *************************************

; includes C header file to be shared with PRU assembler
   .cdecls C,NOLIST
%{
	#include "cslr_icss_g.h"
%}

; includes from \icss-g-industrial-fw\pru_fw_common
	;.include "icss_bsram_macros.h"
    ;.include "pru_constant_defines.h"
    ;.include "icss_xfer_defines.h"
    ;.include "icss_tm_macros.h"
    ;.include "pru_macros.h"

    .include "icss_regs.inc"
    .include "sorte_g_pru_device_register.inc"
    .include "sorte_g_state_define.inc"
    .include "macros.inc"
    .include "sorte_g_packet_definition.inc"

  	.global     main

	.global 	TSK_IDLE_RX_BK1
	.global 	TSK_IDLE_RX_EOF
	.global		TSK_IDLE_RX_BKN

TSK_IDLE_RX_BK1:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	; check if discovery frame
	qbne	TSK_IDLE_RX_BK1_CHECK_LAST_SLAVE, R2.b0, DA_MC		;broadcast
	qbne	TSK_IDLE_RX_BK1_CHECK_LAST_SLAVE, R2.b2, T_DISCOV_MC;
	; initiate task switch with EOF as this is a discovery frame
	set		DEVICE_STATUS, DEVICE_STATUS, EOF_TRIGGER_FLAG
	set		DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	qbbc	TSK_IDLE_RX_BK1_CHECK_LAST_SLAVE, L2_DISC_STATUS_REG, ST_CONTLAST_BIT		; is bit 0 (ST_CONTLAST_BIT) ST_LAST set?
	clr		DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
TSK_IDLE_RX_BK1_CHECK_LAST_SLAVE:
	; the last slave has to actively transmit the frame back because it sets the last device bit
	qbbc	TSK_IDLE_RX_BK1_DONE, DEVICE_STATUS, LAST_DEVICE_FLAG
	; setting the last bit in register only affects the last device
	set		L2_DISC_STATUS_REG, L2_DISC_STATUS_REG, ST_CONTLAST_BIT		; set bit 0 (ST_CONTLAST_BIT) to ST_LAST
	; push frame into L2 TX FIFO
	xout    TXL2_ID, &r2, 32
	set		DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	set		DEVICE_STATUS, DEVICE_STATUS, RXL2_CURRENT_BANK
TSK_IDLE_RX_BK1_DONE:
	.if $defined(PRU0)
    xin    BANK0_ID, &r0, RANGE_R0_R19     ; restore r0 - r19
	.else
    xin    BANK1_ID, &r0, RANGE_R0_R19     ; restore r0 - r19
	.endif
	; task exit code
	xin     TM_YIELD_ID, &R0.b3, 1
	nop
	nop
	halt


TSK_IDLE_RX_BKN:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	qbbc	TSK_IDLE_RX_BKN_DONE, DEVICE_STATUS, LAST_DEVICE_FLAG
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	qbbc	IDLE_RX_BKN_FWD_FRAME, DEVICE_STATUS, RXL2_CURRENT_BANK
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18
IDLE_RX_BKN_FWD_FRAME:
	; push frame into L2 TX FIFO
	xout    TXL2_ID, &r2, 32
	; toggle RXL2 bank for next processed incoming block
	xor		DEVICE_STATUS, DEVICE_STATUS, 1<<RXL2_CURRENT_BANK

TSK_IDLE_RX_BKN_DONE:
	.if $defined(PRU0)
    xin    BANK0_ID, &r0, RANGE_R0_R19     ; restore r0 - r19
	.else
    xin    BANK1_ID, &r0, RANGE_R0_R19     ; restore r0 - r19
	.endif
	; task exit code
	xin     TM_YIELD_ID, &R0.b3, 1
	nop
	nop
	halt

;********************
; TSK_IDLE_RX_EOF
; *******************
TSK_IDLE_RX_EOF:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	qbbc	TSK_IDLE_RX_EOF_CONT, DEVICE_STATUS, LAST_DEVICE_FLAG
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	qbbc	IDLE_RX_EOF_HANDLE_BANK0, DEVICE_STATUS, RXL2_CURRENT_BANK
;IDLE_RX_EOF_HANDLE_BANK1:
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18
	qblt	IDLE_RX_EOF_CHCK_MORE_BYTES_TO_SEND, R18.b0, 32			; if R18.b0 <= 32 && current bankID = 1 --> transmit 32 bytes from bank1
	xout    TXL2_ID, &r2, 32
	; check if more bytes must be send
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	qba		IDLE_RX_EOF_CHCK_MORE_BYTES_TO_SEND
IDLE_RX_EOF_HANDLE_BANK0:
	qbgt	IDLE_RX_EOF_CHCK_MORE_BYTES_TO_SEND, R18.b0, 32
	xout    TXL2_ID, &r2, 32
	; check if more bytes must be send
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18
	;qba		IDLE_RX_EOF_CHCK_MORE_BYTES_TO_SEND
IDLE_RX_EOF_CHCK_MORE_BYTES_TO_SEND:
	and		R0.b0, R18.b0, 0x1F
	qbeq	TSK_IDLE_RX_EOF_PUSH_CRC, R0.b0, 0	; don't push anything in the FIFO if we are at a 32 byte boundary
	; DevNote: Do not send 4 bytes old CRC, but append new CRC.
	sub 	R0.b0, R0.b0, 4
	xout    TXL2_ID, &r2, b0
	; DevNote: Do we need wait
	NOP
	NOP

TSK_IDLE_RX_EOF_PUSH_CRC:
	M_SET_CMD	D_TX_EOF | D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD

TSK_IDLE_RX_EOF_CONT:
	; only process frame if this is a discovery frame
	qbbc	TSK_IDLE_RX_EOF_DONE, DEVICE_STATUS, EOF_TRIGGER_FLAG
	clr		DEVICE_STATUS, DEVICE_STATUS, EOF_TRIGGER_FLAG
	; check frame CRC
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	sub		R18.b0, R18.b0, 1
	and		R18.b0, R18.b0, 0x3F
	qbgt	TSK_IDLE_RX_EOF_CHECK_CRC, R18.b0, 32
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R13
	lsr		R18.b0, R18.b0, 1
	; DevNote: Need to add CRC checking
TSK_IDLE_RX_EOF_CHECK_CRC:
	lsr		R18.b0, R18.b0, 1
	add		R1.b0, R18.b0, 8+32	; add &R10 offset
	mvib	R18.b1, *R1.b0
	qbbs	TSK_IDLE_RX_EOF_DONE, R18.b1, 7	; check if RX_ERR bit is set
	; initiate state switch
	ldi		PROTOCOL_STATE_CNT, DISC_STATE
	set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	qbbs	TSK_IDLE_RX_EOF_DONE, DEVICE_STATUS, LAST_DEVICE_FLAG
	; the master port PRU will be discoverd in the discovery state, based on ST_LAST bit in telegram
	;clr		DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG

TSK_IDLE_RX_EOF_DONE:
	.if $defined(PRU0)
    xin    BANK0_ID, &r0, RANGE_R0_R19     ; restore r0 - r19
	.else
    xin    BANK1_ID, &r0, RANGE_R0_R19     ; restore r0 - r19
	.endif
	; clear RX FIFO after reception
	M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR )
	xin     TM_YIELD_ID, &R0.b3,1
	nop
	nop
	halt

