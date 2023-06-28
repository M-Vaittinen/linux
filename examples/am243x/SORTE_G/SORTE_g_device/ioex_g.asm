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
;     file:   ioex.asm
;
;     brief:
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
    .include "icss_regs.inc"
    .include "sorte_g_pru_device_register.inc"
    .include "sorte_g_state_define.inc"
    .include "macros.inc"
    .include "utils.inc"
    .include "sorte_g_device_cfg.inc"
    .include "sorte_g_packet_definition.inc"
    .include "sorte_g_host_interface.inc"
    .include "irq_num.inc"

	.global 	TSK_IOEX_STATE_FWD_BKN
	.global		TSK_IOEX_STATE_CMP0_EVENT
	.global		TSK_IOEX_STATE_TTS_COMPLETE_EVENT
	.global		TSK_IOEX_STATE_FWD_SOF
	.global		TSK_IOEX_STATE_FWD_EOF

;****************************
; TSK_IOEX_STATE_FWD_SOF
;****************************
TSK_IOEX_STATE_FWD_SOF:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	; load complete 32 byte of header
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18

	; IO exchange init receive parameters for this packet
	LDI		RX_BYTE_CNT, 0

	; store RX timestamp
	.if $defined(PRU0)
	lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 4
	.else
	lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CAPR2_REG, 4
	.endif
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, SYNC_TS_RX, 4
	set		DEVICE_STATUS, DEVICE_STATUS, NEW_SYNC_FRAME_FLAG

	qba		IOEX_STATE_DONE


;****************************
; TSK_IOEX_STATE_FWD_BKN
;****************************
TSK_IOEX_STATE_FWD_BKN:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	; load complete 32 byte of header
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	qbbs	IOEX_FWD_BKN_PROCESS_DATA, R18.b0, 5			; bit 5 indicates the bank ID (>=32 bytes and <64 bytes); when R18.b0 >=32 && <63, then load the lower bank.
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18
	lbco	&r0, ICSS_SHARED_RAM_CONST, DEBUG_RXL2, 4
	add		r0, r0, 1
	sbco	&r0, ICSS_SHARED_RAM_CONST, DEBUG_RXL2, 4
IOEX_FWD_BKN_PROCESS_DATA:
	; calculate write pointer
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEVICE_MASTER_FRAME_BUFFER_PTR, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, RX_BYTE_CNT
	SBCO	&R2, PRU1_DMEM_CONST, TEMP_REG_1.w0, 32
	ADD		RX_BYTE_CNT, RX_BYTE_CNT, 32
	;check for jumbo frame - prevent that PRU1_DMEM_CONST runs full
	; master frame is < 250 bytes
	ldi		TEMP_REG_1.w2, 250
	qbge	IOEX_STATE_DONE, RX_BYTE_CNT, TEMP_REG_1.w2
	; Master Frame too long, reset RX and TX FIFO
	M_CMD16 D_RESET_TXFIFO | D_TX_CRC_ERR | D_RESET_RXFIFO | D_RX_ERROR_CLEAR
	;qba		IOEX_STATE_DONE

IOEX_STATE_DONE:
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


;****************************
; TSK_IOEX_STATE_FWD_EOF
;****************************
TSK_IOEX_STATE_FWD_EOF:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	; load complete 32 byte of header
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	qbbc	IOEX_FWD_EOF_RESET_FIFO, R18.b0, 5	; load in remaining bytes
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18
IOEX_FWD_EOF_RESET_FIFO:
	; reset RX and TX FIFOs and errors
	M_CMD16 D_RESET_TXFIFO | D_TX_CRC_ERR | D_RESET_RXFIFO | D_RX_ERROR_CLEAR

	; check if R18.b0 write pointer is on a 32-byte boundary -> if yes, the data was stored via NB task
	qbeq	IOEX_FWD_EOF_DONE_STORING, R18.b0, 0
	qbeq	IOEX_FWD_EOF_DONE_STORING, R18.b0, 32
	; calculate write pointer
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEVICE_MASTER_FRAME_BUFFER_PTR, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, RX_BYTE_CNT
	; store remaining bytes
	AND		R0.b0, R18.b0, 0x1F
	qbeq	IOEX_FWD_EOF_DONE_STORING, R0.b0, 0
	SBCO	&R2, PRU1_DMEM_CONST, TEMP_REG_1.w0, b0	;
	ADD		RX_BYTE_CNT, RX_BYTE_CNT, R0.b0

IOEX_FWD_EOF_DONE_STORING:
	; Check CRC
	; 2 bytes of data are merged into one byte of status
	lsr		R1.b0, R1.b0, 1
	; add offset to R10
	add		R1.b0, R1.b0, (10*4)
	mvib	TEMP_REG_1.b0, *R1.b0
	qbbs	IOEX_FWD_EOF_CRC_ERROR, TEMP_REG_1.b0, L2_ERROR_CRC_FLAG_BITNUM

	;--------------------------------------------
	; extract the broadcast data from the received frame
	;--------------------------------------------

	ldi		TEMP_REG_1.w0, PARAM_OFFSET
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, BROADCAST_LENGTH
	lbco	&R0.b0,	ICSS_SHARED_RAM_CONST, TEMP_REG_1.w0, 1

	lbco	&R1.w0,	ICSS_SHARED_RAM_CONST, DEVICE_MASTER_FRAME_BUFFER_PTR, 2
	add		R1.w0, R1.w0, 2

	lbco	&R2, PRU1_DMEM_CONST, R1.w0, b0
	lbco	&R1.b0,	ICSS_SHARED_RAM_CONST, DEVICE_OUTDATA_FRAME_BUFFER_PTR, 2
	sbco	&R2, PRU1_DMEM_CONST, R1.w0, b0

	; save number of broadcast bytes as offset for next step.
	mov		R1.b2, R0.b0

	;--------------------------------------------
	; extract this slaves data from the received frame
	;--------------------------------------------

	;lbco	&R0.w0,	ICSS_SHARED_RAM_CONST, DEVICE_MASTER_FRAME_BUFFER_PTR, 2
	;add		R0.w0, R0.w0, OFFSET_OUT_PTR
	;add		R0.w0, R0.w0, 2	; two bytes header
	;lbco	&R1, PRU1_DMEM_CONST, R0.w0, OUTPUT_BYTES_COUNT
	;lbco	&R0.b0,	ICSS_SHARED_RAM_CONST, IOEX_OUTDATA_FRAME_BUFFER_PTR, 2
	;sbco	&R1, PRU1_DMEM_CONST, R0.w0, OUTPUT_BYTES_COUNT
	;set		DEVICE_STATUS, DEVICE_STATUS, NEW_IOEX_DATA_FLAG

	; ICSS_SHARED_RAM_CONST = 0x10000 + DEVICE_OUTDATA_FRAME_BUFFER_SIZE = 0x66 = 0x10066 in the PRU_Device_Memory
	lbco	&R0.b0,	ICSS_SHARED_RAM_CONST, DEVICE_OUTDATA_FRAME_BUFFER_SIZE, 1

	; 0x00010068 (ICSS_SHARED_RAM_CONST = c28 = 0x00010000) + (DEVICE_MASTER_FRAME_BUFFER_PTR = 0x0068) in the PRU_Device_Memory
	lbco	&R1.w0,	ICSS_SHARED_RAM_CONST, DEVICE_MASTER_FRAME_BUFFER_PTR, 2
	add		R1.w0, R1.w0, OFFSET_OUT_PTR
	add		R1.w0, R1.w0, 2	; two bytes header

	add		R1.w0, R1.w0, R1.b2	; broadcast field length

	lbco	&R2, PRU1_DMEM_CONST, R1.w0, b0

	; 0x00010064 (ICSS_SHARED_RAM_CONST = c28 = 0x00010000) + (DEVICE_OUTDATA_FRAME_BUFFER_PTR = 0x0064) in the PRU_Device_Memory
	lbco	&R1.w0,	ICSS_SHARED_RAM_CONST, DEVICE_OUTDATA_FRAME_BUFFER_PTR, 2
	add		R1.w0, R1.w0, R1.w2	; broadcast field length

	; For (PRU_0: PRU1_DMEM_CONST = c25 = 0x00002000)
	sbco	&R2, PRU1_DMEM_CONST, R1.w0, b0

	set		DEVICE_STATUS, DEVICE_STATUS, NEW_IOEX_DATA_FLAG
	;--------------------------------------------
	; Test only: copy output data to input data
	lbco	&R1.w0,	ICSS_SHARED_RAM_CONST, DEVICE_INDATA_FRAME_BUFFER_PTR, 2
	sbco	&R2, PRU1_DMEM_CONST, R1.w0, b0
	;--------------------------------------------


	; IOEX frame monitor flag
	ldi		TEMP_REG_1.b1, 1
	sbco	&TEMP_REG_1.b1, ICSS_SHARED_RAM_CONST, IOEX_FRAME_RECEIVED, 1

	; generate interrupt to notify APPL
	ldi		R31, IRQ_OUT_DATA_READY		; generate int

	qba		IOEX_STATE_DONE

IOEX_FWD_EOF_CRC_ERROR:
	; update statistics
	.if $defined(PRU0)
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, STATS_CRC_NO_ERROR_PRU0, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, 1
	sbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, STATS_CRC_NO_ERROR_PRU0, 2
	.else
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, STATS_CRC_NO_ERROR_PRU1, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, 1
	sbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, STATS_CRC_NO_ERROR_PRU1, 2
	.endif


	; CRC error, update statistics
	.if $defined(PRU0)
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, STATS_CRC_ERROR_PRU0, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, 1
	sbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, STATS_CRC_ERROR_PRU0, 2
	.else
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, STATS_CRC_ERROR_PRU1, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, 1
	sbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, STATS_CRC_ERROR_PRU1, 2
	.endif

	qba		IOEX_STATE_DONE


;****************************
; TSK_IOEX_STATE_CMP0_EVENT
;****************************
; setup TTS and place frame into buffer
TSK_IOEX_STATE_CMP0_EVENT:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif

	; generate int to trigger IN_DATA_READY_EVENT
	ldi		R31, IRQ_IN_DATA_READY	; generate int

	; re-enable SYNC0?
	ldi		TEMP_REG_1.b0, 1<<1		; CMP[1] / ICSS_IEP_CMP1_REG
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

    ldi    TEMP_REG_1, 0x06    ; clear sync_en (sync0_en, sync1_en)
    ldi    TEMP_REG_2.w0, ICSS_IEP_SYNC_CTRL_REG
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4
    ldi    TEMP_REG_1, 0x07    ;  set sync_en (sync0_en, sync1_en)
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4

    ; configure local port to TTS
	M_SET_MIIRT_TTS_LOCAL

	; increment SORTE cycle count
	lbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, SORTE_CYCLE_CNT, 4
	add		TEMP_REG_1, TEMP_REG_1, 1
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, SORTE_CYCLE_CNT, 4

IOEX_STATE_CLR_CMP_EVENT:
; clear cmp0, cmp1 and cmp3 events
	;ldi		TEMP_REG_1.b0, 0x1f
	ldi		TEMP_REG_1.b0, 0x3b		;	0x3b - 0x0011 1011

	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
; re-enable SYNC0
; clear and set sync_en bit in IEP timer to re-enable trigger
    ldi    TEMP_REG_1, 0x06    ; clear sync_en (sync0_en, sync1_en)
    ldi    TEMP_REG_2.w0, ICSS_IEP_SYNC_CTRL_REG
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 2
    ldi    TEMP_REG_1, 0x07    ;  set sync_en (sync0_en, sync1_en)
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 2

	; reset RX- and TX-FIFO and clear TX errors
	ldi		R31.w2, (D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RESET_TXFIFO)
	; fill FIFO
	.if $defined(FULL_PREAMBLE)
	ldi		R30.w0, 0x5555
	; check if NOPs are needed
	ldi		R30.w0, 0x5555
	.endif
	; send pre-amble + slave address
	ldi		R30.w0, 0x5555
	ldi		R30.w0, 0xd555
	mov		R30.b0, DEVICE_ADDR		; This adds 1 additional byte to INPUT_BYTES_COUNT
; Load the device input byte from memory
; Check if length is >26 byte
	;ldi		R0.w0, INPUT_BYTES_COUNT
	ldi		R0.w0, 0
	lbco	&R0.b0,	ICSS_SHARED_RAM_CONST, DEVICE_INDATA_FRAME_BUFFER_SIZE, 1
	qbgt	IOEX_CMP0_LOAD_DATA, R0.w0, 26
;IOEX_CMP0_MULTI_PAKET:
	ldi		R0.w0, 26		; if INPUT_BYTES_COUNT length >26, we push data to TX FIFO multiple times
IOEX_CMP0_LOAD_DATA:
; load payload from PRU data memory
    ldi		TEMP_REG_1.w0, DEVICE_INDATA_FRAME_BUFFER_PTR
    lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, TEMP_REG_1.w0, 2
    lbco	&R2.b0, PRU1_DMEM_CONST, TEMP_REG_1.w0, b0
	ldi		R1.b0, &R2.b0
	loop	IOEX_CMP0_SEND, R0.b0
	mvib	TX_DATA_BYTE, *R1.b0++
	nop		; DevNote: NOP?
IOEX_CMP0_SEND:
	; generate CRC and TX_EOF if  INPUT_BYTES_COUNT <=26
	qblt	IOEX_CMP0_NO_CRC, R0.w0, 26
	; push CRC32
	M_CMD16 (D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD | D_TX_EOF)
IOEX_CMP0_NO_CRC:

	.if $defined (pru_data_generation_test)
	; DevNote: The next lines are for validation only !!! Need to be removed for final application
	; increment cycle counter in process data
	add		R2, R2, 1
	sbco	&R2, PRU1_DMEM_CONST, TEMP_REG_1.w0, 4
	; e/o DevNote
	.endif

	qba		IOEX_STATE_DONE

;****************************
; TSK_IOEX_STATE_TTS_COMPLETE_EVENT
;****************************
; triggered by CMP3/4 (TTS) event
TSK_IOEX_STATE_TTS_COMPLETE_EVENT:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	; init previous FIFO level to -1
	ldi		TEMP_REG_1.b1, 0xff
	; init TX byte counter
	;ldi		R1.w0, INPUT_BYTES_COUNT
	ldi		R1.w0, 0
	lbco	&R0.b0,	ICSS_SHARED_RAM_CONST, DEVICE_INDATA_FRAME_BUFFER_SIZE, 1
	ldi		R1.w2, 26	; 26 bytes are already in the TX FIFO
IOEX_TTS_COMPLETE_CHECK_TXFIFOLEVEL:
	.if $defined(PRU0)
	lbco	&TEMP_REG_1.b0, ICSS_MII_RT_CONST, ICSS_MIIRT_TX_FIFO_LEVEL1, 1
	.else
	lbco	&TEMP_REG_1.b0, ICSS_MII_RT_CONST, ICSS_MIIRT_TX_FIFO_LEVEL0, 1
	.endif
	qbeq	IOEX_TTS_TIMEOUT, TEMP_REG_1.b0, TEMP_REG_1.b1	; when new FIFO level does not increase, we detected a stuck -> update statistics and exit this function
	mov		TEMP_REG_1.b1, TEMP_REG_1.b0

	qble	IOEX_TTS_MULTI_FIFO_FILL, R1.w0, 26
	; byte to transmit <=26 bytes: single FIFO fill in CMP0
	qbne	IOEX_TTS_COMPLETE_CHECK_TXFIFOLEVEL, TEMP_REG_1.b0, 0
	qba		IOEX_TTS_COMPLETE

; multi FIFO fill
IOEX_TTS_MULTI_FIFO_FILL:
	qble	IOEX_TTS_COMPLETE_CHECK_TXFIFOLEVEL, TEMP_REG_1.b0, 40-16-4	; 40 bytes FIFO, 16 bytes data, 4 bytes CRC
	; load next 16 bytes
	sub		R0.b0, R1.w0, R1.w2
	qble	IOEX_TTS_MULTI_CHECK_ALL_BYTES_SENT, R0.b0, 16
	ldi		R0.w0, 16	; more than 16 bytes to sent, limit next send to 16 bytes
IOEX_TTS_MULTI_CHECK_ALL_BYTES_SENT:
	qbne	IOEX_TTS_MULTI_LOAD_DATA, R1.w0, R1.w2
	; all bytes sent, wait for TXFIFO empty
	qbne	IOEX_TTS_COMPLETE_CHECK_TXFIFOLEVEL, TEMP_REG_1.b0, 0
	qba		IOEX_TTS_COMPLETE

IOEX_TTS_MULTI_LOAD_DATA:
; load payload from PRU data memory
    ldi		TEMP_REG_1.w0, DEVICE_INDATA_FRAME_BUFFER_PTR
    lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, TEMP_REG_1.w0, 2
    lbco	&R2.b0, PRU1_DMEM_CONST, TEMP_REG_1.w0, b0
	ldi		R1.b0, &R2.b0
	loop	IOEX_TTS_MULTI_SEND, R0.b0
	mvib	TX_DATA_BYTE, *R1.b0++
	nop		; DevNote: NOP needed?
IOEX_TTS_MULTI_SEND:
	add		R1.w0, R1.w0, R0.b0
	; check if this is the last part of data: if yes, CRC/TX_EOF has to be generated.
	qbne	IOEX_TTS_COMPLETE_CHECK_TXFIFOLEVEL, R1.w0, R1.w2

IOEX_TTS_MULTI_CRC:
	M_CMD16 (D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD | D_TX_EOF)
	qba		IOEX_TTS_COMPLETE_CHECK_TXFIFOLEVEL

IOEX_TTS_TIMEOUT:
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, TTS_TIMEOUT_COUNT, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, 1
	sbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, TTS_TIMEOUT_COUNT, 2
IOEX_TTS_COMPLETE:
	; configure local port to AF
	M_SET_MIIRT_AF_LOCAL
	; reset RX- and TX-FIFO and clear TX errors
	ldi		R31.w2, (D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RESET_TXFIFO)
	qba		IOEX_STATE_DONE

