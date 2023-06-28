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
	#include "sorte_g_device_host_interface.h"
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

	.global 	TSK_DISCOVERY_BK1
	.global		TSK_DISCOVERY_BKN
	.global		TSK_DISCOVERY_BKN2
	.global		TSK_DISCOVERY_EOF


; **********************
; TSK_DISCOVERY_BK1
; **********************
TSK_DISCOVERY_BK1:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif

	; debug: time stamping
	;ldi		TEMP_REG_1, 0x02
	;lbco	&TEMP_REG_2, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	;sbco	&TEMP_REG_1, PRU_LOCAL_DMEM_CONST, TS_ADDR, 8
	;add		TS_ADDR, TS_ADDR, 8


	; load complete 32 byte of header
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	; check if discovery frame
	qbne	DISC_BK1_NO_VALID_FRAME, R2.b0, DA_MC		;broadcast
	qbne	DISC_BK1_NO_VALID_FRAME, R2.b2, T_DISCOV_MC;

	; check for state switching
	qbbc	TSK_DISC_NO_STATE_SWITCH, L2_DISC_STATUS_REG, ST_STATECHANGE_BIT
	; initiate task switch with EOF as this is a statechange bit is set
	set		DEVICE_STATUS, DEVICE_STATUS, EOF_TRIGGER_FLAG

TSK_DISC_NO_STATE_SWITCH:
	; init RX parameters
	LDI		RX_BYTE_CNT, 0
	clr		DEVICE_STATUS, DEVICE_STATUS, RXL2_CURRENT_BANK

	; check if this is the reverse port
	set		DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	qbbc	DISC_MASTER_PORT_FLAG, L2_DISC_STATUS_REG, ST_CONTLAST_BIT	; if ST_CONTLAST_BIT is set, this frams is received on the reverser path
	clr		DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
DISC_MASTER_PORT_FLAG:
	; reverse port should not touch the frame but just forward
	qbbc	DISC_SEND32, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
;DISC_STATUS: ; status (4)
	; update status byte
	qbbc	DISC_STATUS_2, DEVICE_STATUS, LAST_DEVICE_FLAG
	set		L2_DISC_STATUS_REG, L2_DISC_STATUS_REG, ST_CONTLAST_BIT		; set bit 0 (ST_CONTLAST_BIT) to ST_LAST
	set		DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
DISC_STATUS_2:
	; last RX frame had an error -> inform the master
	qbbc	DISC_STATUS_3, DEVICE_STATUS, RX_FRAME_CRC_ERROR
	set		L2_DISC_STATUS_REG, L2_DISC_STATUS_REG, 2 	; bit 2 ST_ERROR_BIT
DISC_STATUS_3:
	mov		FRAME_STATUS, L2_DISC_STATUS_REG	; store current DISK_STATUS value

; Device Count (5, 6)
DISC_DEV_CNT:
	add		L2_DISC_DEVICE_COUNT, L2_DISC_DEVICE_COUNT, 1	; increment device count
	mov		DEVICE_ADDR, L2_DISC_DEVICE_COUNT	; up to 254 slaves are supported by the slave implementation
	sbco	&DEVICE_ADDR, ICSS_SHARED_RAM_CONST, DEVICE_ADDR_OFFSET, 1

; Read INPUT_BYTES_COUNT and OUTPUT_BYTES_COUNT from the SharedMem
	lbco	&TEMP_REG_2.w0, ICSS_SHARED_RAM_CONST, DEVICE_INDATA_FRAME_BUFFER_SIZE, 2
	lbco	&TEMP_REG_2.w2, ICSS_SHARED_RAM_CONST, DEVICE_OUTDATA_FRAME_BUFFER_SIZE, 2

; OFFSET_IN_PTR (7,8)
	;add	L2_OFFSET_IN_PTR, L2_OFFSET_IN_PTR, INPUT_BYTES_COUNT
	add		L2_OFFSET_IN_PTR, L2_OFFSET_IN_PTR, TEMP_REG_2.w0
	mov		OFFSET_IN_PTR, L2_OFFSET_IN_PTR

; OFFSET_OUT_PTR (9,10)
DISC_OFFSET_OUT_PTR:
	mov		OFFSET_OUT_PTR, L2_OFFSET_OUT_PTR
	;add		L2_OFFSET_OUT_PTR, L2_OFFSET_OUT_PTR, OUTPUT_BYTES_COUNT
	add		L2_OFFSET_OUT_PTR, L2_OFFSET_OUT_PTR, TEMP_REG_2.w2

; calculate input/ouput insertion pointer
	sub		TEMP_REG_1.w0, DEVICE_ADDR, 1
	lsl		TEMP_REG_1.w0, TEMP_REG_1.w0, 2
	add		DISC_INPUT_OFFSET, TEMP_REG_1.w0, DISC_IO_OFFSET
	add		DISC_OUTPUT_OFFSET, DISC_INPUT_OFFSET, 2
	; check if data can be added
	qble	DISC_SEND32, DISC_INPUT_OFFSET, 32
	; insert input data now
	add		R1.b0, DISC_INPUT_OFFSET, 8	;R2 offset
	;ldi	TEMP_REG_1.w0, INPUT_BYTES_COUNT
	mov		TEMP_REG_1.w0, TEMP_REG_2.w0
	mviw	*R1.b0++, TEMP_REG_1.w0
	; check if output data can be added
	qble	DISC_SEND32, DISC_OUTPUT_OFFSET, 32
	; insert output data now
	;add		R1.b0, DISC_OUTPUT_OFFSET, 8	;R2 offset
	;ldi	TEMP_REG_1.w0, OUTPUT_BYTES_COUNT
	mov		TEMP_REG_1.w0, TEMP_REG_2.w2
	mviw	*R1.b0++, TEMP_REG_1.w0
	qba		DISC_SEND32

DISC_BK1_NO_VALID_FRAME:
	; initiate state switch
	;ldi		PROTOCOL_STATE_CNT, IDLE_STATE
	;set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR )
	qba			DISC_DONE


DISC_SEND32:
	add		RX_BYTE_CNT, RX_BYTE_CNT, 32
	ldi		R0.b0, 32
	; check if max frame length was reached.
	ldi		TEMP_REG_1.w0, DISC_LENGTH - 4	; discovery frame length without CRC
	qbge	DISC_SEND32_XOUT, RX_BYTE_CNT, TEMP_REG_1.w0 ; check if #_of_bytes_transmitted <= frame_length
	; send remaining bytes: frame length - #_of_bytes_transmitted
	sub		RX_BYTE_CNT, RX_BYTE_CNT, 32
	sub		R0.b0, TEMP_REG_1.w0, RX_BYTE_CNT
	and		R0.b0, R0.b0, 0x1f	; don't send more than 32 bytes in case the packet has not the expected length
	add		RX_BYTE_CNT, RX_BYTE_CNT, R0.b0
	qbeq	DISC_SEND32_XOUT_DONE, R0.b0, 0
DISC_SEND32_XOUT:
	xout    TXL2_ID, &r2, b0
DISC_SEND32_XOUT_DONE:
	xor		DEVICE_STATUS, DEVICE_STATUS, 1<<RXL2_CURRENT_BANK	; toggle bank
	;qba		DISC_DONE

DISC_DONE:

	; debug: time stamping
	;ldi		TEMP_REG_1, 0x01
	;lbco	&TEMP_REG_2, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	;sbco	&TEMP_REG_1, PRU_LOCAL_DMEM_CONST, TS_ADDR, 8
	;add		TS_ADDR, TS_ADDR, 8

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
; TSK_DISCOVERY_BKN
; **********************
TSK_DISCOVERY_BKN2
	nop
TSK_DISCOVERY_BKN:
	; push next 32 bytes of data into TX FIFO
	; check if input and output byte count has to be inserted into this frame
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif

	; debug: time stamping
	;ldi		TEMP_REG_1, 0x04
	;lbco	&TEMP_REG_2, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	;sbco	&TEMP_REG_1, PRU_LOCAL_DMEM_CONST, TS_ADDR, 8
	;add		TS_ADDR, TS_ADDR, 8

	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	qbbc	DISC_BKN_CHECK_FOR_MOD, DEVICE_STATUS, RXL2_CURRENT_BANK
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18
DISC_BKN_CHECK_FOR_MOD:
	; reverse port should not touch the frame because
	qbbc	DISC_SEND32, DEVICE_STATUS, PRU_MASTER_PORT_FLAG

	; check if data can be added
	add		R1.w2, RX_BYTE_CNT, 32
	; jump if (input data offset < RX_BYTE_CNT) || (input data offset >=  (RX_BYTE_CNT+32))
	qbgt	DISC_BKN_OUTPUT_DATA_CHECK, DISC_INPUT_OFFSET, RX_BYTE_CNT
	qble	DISC_BKN_OUTPUT_DATA_CHECK, DISC_INPUT_OFFSET, R1.w2
	; insert input data now
	add		R1.w0, DISC_INPUT_OFFSET, 8	;R2 offset
	sub		R1.w0, R1.w0, RX_BYTE_CNT
	;ldi	TEMP_REG_1.w0, INPUT_BYTES_COUNT
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEVICE_INDATA_FRAME_BUFFER_SIZE, 2
	mviw	*R1.b0, TEMP_REG_1.w0
DISC_BKN_OUTPUT_DATA_CHECK:
	; check if output data can be added
	qbgt	DISC_SEND32, DISC_OUTPUT_OFFSET, RX_BYTE_CNT
	qble	DISC_SEND32, DISC_OUTPUT_OFFSET, R1.w2
	; insert output data now
	add		R1.w0, DISC_OUTPUT_OFFSET, 8	;R2 offset
	sub		R1.w0, R1.w0, RX_BYTE_CNT
	;ldi	TEMP_REG_1.w0, OUTPUT_BYTES_COUNT
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEVICE_OUTDATA_FRAME_BUFFER_SIZE, 2
	mviw	*R1.b0, TEMP_REG_1.w0

	; transmit 32 bytes
	qba		DISC_SEND32


; **********************
; TSK_DISCOVERY_EOF
; **********************
TSK_DISCOVERY_EOF:
	; check if more bytes have to be pushed into the TX FIFO
	; compute and append CRC
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif

	; debug: time stamping
	;ldi		TEMP_REG_1, 0x08
	;lbco	&TEMP_REG_2, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	;sbco	&TEMP_REG_1, PRU_LOCAL_DMEM_CONST, TS_ADDR, 8
	;add		TS_ADDR, TS_ADDR, 8

	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	qbbc	DISC_EOF_CHECK_FOR_MOD, DEVICE_STATUS, RXL2_CURRENT_BANK
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18
DISC_EOF_CHECK_FOR_MOD:
	; reverse port should not touch the frame because as it is AF
	qbbc	DISK_EOF_TRANSMIT_REST, DEVICE_STATUS, PRU_MASTER_PORT_FLAG

	; check if data can be added
	add		R1.w2, RX_BYTE_CNT, 32
	; jump if (input data offset < RX_BYTE_CNT) || (input data offset >=  (RX_BYTE_CNT+32))
	qbgt	DISC_EOF_OUTPUT_DATA_CHECK, DISC_INPUT_OFFSET, RX_BYTE_CNT
	qble	DISC_EOF_OUTPUT_DATA_CHECK, DISC_INPUT_OFFSET, R1.w2
	; insert input data now
	add		R1.w0, DISC_INPUT_OFFSET, 8	;R2 offset
	sub		R1.w0, R1.w2, R1.w0
	;ldi	TEMP_REG_1.w0, INPUT_BYTES_COUNT
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEVICE_INDATA_FRAME_BUFFER_SIZE, 2
	mviw	*R1.b0, TEMP_REG_1.w0
DISC_EOF_OUTPUT_DATA_CHECK:
	; check if output data can be added
	qbgt	DISK_EOF_TRANSMIT_REST, DISC_OUTPUT_OFFSET, RX_BYTE_CNT
	qble	DISK_EOF_TRANSMIT_REST, DISC_OUTPUT_OFFSET, R1.w2
	; insert output data now
	add		R1.w0, DISC_OUTPUT_OFFSET, 8	;R2 offset
	sub		R1.w0, R1.w2, R1.w0
	;ldi	TEMP_REG_1.w0, OUTPUT_BYTES_COUNT
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEVICE_OUTDATA_FRAME_BUFFER_SIZE, 2
	mviw	*R1.b0, TEMP_REG_1.w0

DISK_EOF_TRANSMIT_REST:
	ldi		TEMP_REG_1.w0, DISC_LENGTH - 4	; discovery frame length without CRC
	qble	TSK_DISCOVERY_EOF_DONE, RX_BYTE_CNT, TEMP_REG_1.w0
	; transmit remaining bytes
	and		R18.b0, R18.b0, 0x1F
	sub		R0.b0, R18.b0, 4
	qble	TSK_DISCOVERY_EOF_DONE, R0.b0, 32
	qbeq	TSK_DISCOVERY_EOF_DONE, R18.b0, 0	; don't push anything in the FIFO if we are at a 32 byte boundary
	add		RX_BYTE_CNT, RX_BYTE_CNT, R0.b0
	xout    TXL2_ID, &r2, b0

TSK_DISCOVERY_EOF_DONE:
	M_SET_CMD	D_TX_EOF | D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD
	M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR )

	; check if CRC is valid
	sub		R18.b1, R18.b0, 1
	and		R18.b1, R18.b1, 0x1F
	lsr		R18.b0, R18.b0, 1
	add		R1.b0, R18.b1, 8+32	; add &R10 offset
	mvib	R18.b1, *R1.b0
	qbbc	TSK_DISCOVERY_EOF_DONE2, R18.b1, 7	;ERROR_CRC bit 7
	; CRC error
	.if $defined(PRU0)
	lbco	&R0.w0,ICSS_SHARED_RAM_CONST, STATS_CRC_ERROR_PRU0, 2
	add		R0.w0, R0.w0, 1
	sbco	&R0.w0,ICSS_SHARED_RAM_CONST, STATS_CRC_ERROR_PRU0, 2
	.else
	lbco	&R0.w0,ICSS_SHARED_RAM_CONST, STATS_CRC_ERROR_PRU1, 2
	add		R0.w0, R0.w0, 1
	sbco	&R0.w0,ICSS_SHARED_RAM_CONST, STATS_CRC_ERROR_PRU1, 2
	.endif

TSK_DISCOVERY_EOF_DONE2:
	qbbc	DISC_DONE, DEVICE_STATUS, EOF_TRIGGER_FLAG
	clr		DEVICE_STATUS, DEVICE_STATUS, EOF_TRIGGER_FLAG
	; initiate state switch
	ldi		PROTOCOL_STATE_CNT, PARAM_STATE
	set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	qba		DISC_DONE


