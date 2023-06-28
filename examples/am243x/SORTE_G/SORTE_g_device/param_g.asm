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
    .include "sorte_g_device_cfg.inc"
    .include "sorte_g_packet_definition.inc"
    .include "sorte_g_host_interface.inc"

	.global 	TSK_PARAM_BK1
	.global		TSK_PARAM_BK2
	.global		TSK_PARAM_EOF


; **********************
;
; **********************
TSK_PARAM_BK1:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	; load complete 32 byte of header
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	mov		FRAME_STATUS, R2.b3
	qbbc	PARAM_CHECK_STATE_SWITCH, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
;PARAM_PROCESS_DATA:
	ldi		TEMP_REG_6.w0, PARAM_OFFSET
	sbco	&R3.b0, ICSS_SHARED_RAM_CONST, TEMP_REG_6.w0, 7*4; param without 4 byte header
	;add		TEMP_REG_6.w0, TEMP_REG_6.w0, 7*4
	;xin		RXL2_BANK1, &R2, RANGE_R2_R13
	;sbco	&R2.b0, ICSS_SHARED_RAM_CONST, TEMP_REG_6.w0, $sizeof(param_s) - 4 - 7*4; param without 4 byte header
	;set		DEVICE_STATUS, DEVICE_STATUS, RX_IGNORE_TO_EOF
PARAM_CHECK_STATE_SWITCH:
	qbbc	PARAM_DONE, FRAME_STATUS, 3		; ST_STATE_CHANGE
	; initiate state change switch
	set		DEVICE_STATUS, DEVICE_STATUS, EOF_TRIGGER_FLAG

PARAM_DONE:
	.if $defined(PRU0)
    xin    BANK0_ID, &r0, RANGE_R0_R19     ; restore r0 - r19
	.else
    xin    BANK1_ID, &r0, RANGE_R0_R19     ; restore r0 - r19
	.endif
	; exit task
	xin     TM_YIELD_ID, &R0.b3,1
	nop
	nop
	halt

; **********************
;
; **********************
TSK_PARAM_BK2:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	qbbc	PARAM_DONE, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	; load complete 32 byte of header
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18
	ldi		TEMP_REG_6.w0, PARAM_OFFSET
	add		TEMP_REG_6.w0, TEMP_REG_6.w0, 7*4
	sbco	&R2.b0, ICSS_SHARED_RAM_CONST, TEMP_REG_6.w0, $sizeof(param_s) - 4 - 7*4; param without 4 byte header
	qba		PARAM_DONE

; **********************
;
; **********************
TSK_PARAM_EOF:
	;.if $defined(PRU0)
    ;xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	;.else
    ;xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	;.endif
	M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR )
	qbbc	PARAM_DONE, DEVICE_STATUS, EOF_TRIGGER_FLAG
	; DevNote: check if CRC is valid
	clr		DEVICE_STATUS, DEVICE_STATUS, EOF_TRIGGER_FLAG
	; initiate state switch
	ldi		PROTOCOL_STATE_CNT, LINED_STATE
	set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	qba		PARAM_DONE
