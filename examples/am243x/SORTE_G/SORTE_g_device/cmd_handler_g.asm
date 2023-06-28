; Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
;     file:   cmd_handler.asm
;
;     brief:  
;
;
;    Version        Description                                Author
;     0.1         	Ported to AM65xx                           Thomas Mauer
; includes C header file to be shared with PRU assembler
   .cdecls C,NOLIST
%{
	#include "cslr_icss_g.h"
	#include "sorte_g_device_host_interface.h"
	#include "sorte_g_cmd_def.h"
%}

    .include "icss_regs.inc"
    .include "sorte_g_pru_device_register.inc"
    .include "sorte_g_state_define.inc"
    .include "sorte_g_device_cfg.inc"
    .include "sorte_g_host_interface.inc"
    .include "macros.inc"

;CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

	.global		FN_CMD_HANDLER
    .sect    ".text"

FN_CMD_HANDLER:
	qbgt	CMD_DONE, PROTOCOL_STATE_CNT, IOEX_STATE

	ldi		TEMP_REG_1.w0, PARAM_OFFSET
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, BROADCAST_LENGTH
	lbco	&R0.b0,	ICSS_SHARED_RAM_CONST, TEMP_REG_1.w0, 1

	lbco	&R1.w0,	ICSS_SHARED_RAM_CONST, DEVICE_MASTER_FRAME_BUFFER_PTR, 2
	add		R1.w0, R1.w0, 2

	lbco	&R2, PRU1_DMEM_CONST, R1.w0, b0
	lbco	&R1.b0,	ICSS_SHARED_RAM_CONST, DEVICE_OUTDATA_FRAME_BUFFER_PTR, 2
	lbco	&TEMP_REG_1, PRU1_DMEM_CONST, R1.w0, b0

	qbeq	CMD_DONE, TEMP_REG_1, 0
	qbbs	RESET_CMD_HANDLER, 	TEMP_REG_1.b0, CMD_RESET_FSM_SHIFT

	qba		CMD_DONE

; **************************************************
;
; **************************************************

RESET_CMD_HANDLER:
	ldi		PROTOCOL_STATE_CNT, RESET_STATE
	set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	qba		CMD_DONE

CMD_DONE:
	RET
