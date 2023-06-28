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
;     file:   sync.asm
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
%}
    .include "icss_regs.inc"
    .include "sorte_g_pru_device_register.inc"
    .include "sorte_g_state_define.inc"
    .include "macros.inc"
    .include "sorte_g_device_cfg.inc"
    .include "sorte_g_packet_definition.inc"
    .include "sorte_g_host_interface.inc"

	.global 	TSK_SYNC_STATE_FWD_SOF
	.global		TSK_SYNC_STATE_REV_EOF
	.global 	FN_SYNC_HANDLER


;****************************
; TSK_SYNC_STATE_FWD_SOF
;****************************
TSK_SYNC_STATE_FWD_SOF:
	; push 1st byte into TX FIFO
	ldi		R30.b0, 0xFF	; broadcast

	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif

	; load delay values
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, LINE_DELAY_NEXT, 2		;
	;ldi		TEMP_REG_1.w2, BRIDGE_DELAY_CONST	; PRU bridge delay	;DevNote: use measured value located at SYNC_BRIDGE_DELAY if applicable also in IO State
	;add		TEMP_REG_2, TEMP_REG_1.w0, TEMP_REG_1.w2

SYNC_WAIT_FOR_4_BYTES:
	xin		RXL2_BANK0_ID, &R18.b0, RANGE_R18
	; load complete 32 byte of header
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R9
	qbge	SYNC_WAIT_FOR_4_BYTES, R18.b0, 3		; wait for 4 bytes
	mov		R30.b0, R2.b1
	;nop
	ldi		TEMP_REG_1.w2, BRIDGE_DELAY_SYNC_IOEX - 25	; PRU bridge delay	;DevNote: use measured value located at SYNC_BRIDGE_DELAY if applicable also in IO State
	mov		R30.w0, R2.w2

	; SYNC header parsing
	qbeq	SYNC_TYPE_OKAY, R2.b2, T_SYNC_MC
	; DevNote: Error condition, frame is not a sync frame

SYNC_TYPE_OKAY:
	;qbbc	SYNC_STATUS_2, DEVICE_STATUS, LAST_DEVICE_FLAG
	;set		L2_DISC_STATUS_REG, L2_DISC_STATUS_REG, ST_CONTLAST_BIT		; set bit 0 (ST_CONTLAST_BIT) to ST_LAST
SYNC_STATUS_2:
	;qbbc	SYNC_STATUS_3, DEVICE_STATUS, RX_FRAME_CRC_ERROR
	;set		L2_DISC_STATUS_REG, L2_DISC_STATUS_REG, 2 	; bit 2 ST_ERROR_BIT
	;clr		DEVICE_STATUS, DEVICE_STATUS, RX_FRAME_CRC_ERROR
SYNC_STATUS_3:

SYNC_WAIT_FOR_10_BYTES:
	;xin		RXL2_BANK0_ID, &R18.b0, RANGE_R18
	; load complete 32 byte of header
	;nop
	add		TEMP_REG_2, TEMP_REG_1.w0, TEMP_REG_1.w2	; instead of nop
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R9
	;qbge	SYNC_WAIT_FOR_4_BYTES, R18.b0, 9		; wait for 10 bytes
	; store previous slaves line delay
	;sbco	&R3, ICSS_SHARED_RAM_CONST, SYNC_DELAY, 4
	mov		TEMP_REG_3, R3
	; add this slaves bridge delay and next line delay
	add		R3, R3, TEMP_REG_2
	; push to TX FIFO
	mov		R30.w0, R3.w0
	nop
	mov		R30.w0, R3.w2
	;nop
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R9
	;mov		R30.w0, R4.w0
	mov		R30.w0, TEMP_REG_1.w0		; LINE_DELAY_NEXT	;DevNote: Clarify this value
	nop
	; push CRC32
	M_CMD16 (D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD | D_TX_EOF)
	; DevNote: for debug purpose removed CRC
	;M_CMD16 (D_TX_EOF)
	QBBC	SYNC_RESET_RX_FIFO, DEVICE_STATUS, LAST_DEVICE_FLAG
	M_CMD16 (D_RESET_TXFIFO | D_TX_CRC_ERR)
SYNC_RESET_RX_FIFO:
	; clear RX FIFO + Errors
	M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR)

	sbco	&TEMP_REG_3, ICSS_SHARED_RAM_CONST, SYNC_DELAY, 4

	; store frame status in registers
	mov		FRAME_STATUS, L2_DISC_STATUS_REG
	; State change flag set?
	qbbc	SYNC_STATUS_4, L2_DISC_STATUS_REG, ST_STATECHANGE_BIT; ST_STATE_CHANGE
	; initiate state change switch
	ldi		PROTOCOL_STATE_CNT, IOEX_STATE
	set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	; dummy read for the time stamp
	.if $defined(PRU0)
	lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 4	; PRU0_RX_SOF
	.else
	lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CAPR2_REG, 4	; PRU1_RX_SOF
	.endif
	; for the last SYNC frame, do not execute SYNC Delay measurment as we have already the state changed!
	qba		SYNC_STATE_DONE	; no timestamp processing for last SYNC frame
SYNC_STATUS_4:
SYNC_GET_TIMESTAMP:
	; get RX_SOF and TX_SOF timestamps
	.if $defined(PRU0)
	lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 4	; PRU0_RX_SOF
	lbco	&TEMP_REG_2, ICSS_IEP_CONST, ICSS_IEP_CAPR5_REG, 4	; PORT1_TX_SOF
	.else
	lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CAPR2_REG, 4	; PRU1_RX_SOF
	lbco	&TEMP_REG_2, ICSS_IEP_CONST, ICSS_IEP_CAPR4_REG, 4	; PORT0_TX_SOF
	.endif
	sub		TEMP_REG_3, TEMP_REG_2, TEMP_REG_1					; cacluate SYNC_BRIDGE_DELAY
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, SYNC_TS_RX, 12		; SYNC_TS + SYNC_TS_TX + SYNC_BRIDGE_DELAY
	set		DEVICE_STATUS, DEVICE_STATUS, NEW_SYNC_FRAME_FLAG

SYNC_STATE_DONE:
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
; TSK_SYNC_STATE_REV_EOF
;****************************
; reverse port only need to monitor state change bit
TSK_SYNC_STATE_REV_EOF:
	.if $defined(PRU0)
    xout    BANK0_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.else
    xout    BANK1_ID, &r0, RANGE_R0_R19     ; save r0 - r19
	.endif
	; load complete 32 byte of header
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR)
	; State change flag set?
	qbbc	SYNC_STATE_DONE, L2_DISC_STATUS_REG, ST_STATECHANGE_BIT; ST_STATE_CHANGE
	; initiate state change switch
	ldi		PROTOCOL_STATE_CNT, IOEX_STATE
	set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	qba		SYNC_STATE_DONE

;**************************
; SYNC Handler Function
;**************************
; called from state.asm when NEW_SYNC_FRAME_FLAG is set.
FN_SYNC_HANDLER:
	qbbc	SYNC_HANDLER_AVERAGE_DONE, DEVICE_STATUS, NEW_SYNC_FRAME_FLAG
	clr		DEVICE_STATUS, DEVICE_STATUS, NEW_SYNC_FRAME_FLAG
	qbne	SYNC_HANDLER_AVERAGE, SYNC_STATE_REG, SYNC_STATE_NOT_INITIALIZED

; called once when the SYNC state starts. The code synchronizes the devices IEP counter with the master; IEP timer runs linear
SYNC_HANDLER_INIT:
	lbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, SYNC_DELAY, 4	; SYNC_DELAY
	lbco	&TEMP_REG_2, ICSS_SHARED_RAM_CONST, SYNC_TS_RX, 4	; RX timestamp
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, SYNC_DELAY_1ST, 2 ; store the first SYNC_DELAY from SYNC STATE
	sub		TEMP_REG_1, TEMP_REG_2, TEMP_REG_1	;delta TS = SYNC_TS - SYNC_DELAY
	sub		TEMP_REG_1, TEMP_REG_3, TEMP_REG_1  ; IEP CNT - delta TS
	add		TEMP_REG_1, TEMP_REG_1, (3+5+3) * 4 ; IEP access and PRU processing cycles ;DevNote: Adopt for AM65xx
	; 3 and 3 cycles for accessing IEP timer, 5 cycles arithmetics
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	; set count to 1
	ldi		SYNC_STATE_REG, SYNC_STATE_OPERATIONAL

	.if $defined(AMA)	; approximate moving average filter
	; init AMA filter
	zero	&TEMP_REG_1, 4		; ToDo: Initialize the first delta_t instead of 0
	ldi		TEMP_REG_2.b0, AMA_N_DEPTH
	ldi		TEMP_REG_2.w1, 1
	lsl		TEMP_REG_2.w1, TEMP_REG_2.w1, AMA_N_DEPTH - 1	; rounding summand (0.5 * N)
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, AMA_Xm_update, 7		; AMA_Xm_update (4), AMA_N (1), AMA_ROUND_SUMMAND (2)
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, AMA_IEP_CORR, 4
	.endif
	; SMA_WRITE_OFFSET is used to monitor deltat TS values.
	zero	&R2, 4
	;clear SMA_WRITE_OFFSET
	sbco	&R2, ICSS_SHARED_RAM_CONST, SMA_WRITE_OFFSET, 2

	;qbne	debug_skip_toggle1, DEVICE_ADDR, 1
	;.if $defined(PRU0)
	;EDIO toggle
	;ldi32	TEMP_REG_1, 0x0B22E313
	;ldi		TEMP_REG_2.b0, 0x80
	;sbbo	&TEMP_REG_2.b0, TEMP_REG_1, 0, 1
	;M_SET_G2_EDIO31 TEMP_REG_1, TEMP_REG_2.b0
	;.endif
;debug_skip_toggle1:

	; debug
	; set SYNC0 pulse width
	ldi		TEMP_REG_1.w2, 125
	ldi     TEMP_REG_2.w0, ICSS_IEP_SYNC_PWIDTH_REG
	sbco    &TEMP_REG_1.w2, ICSS_IEP_CONST, TEMP_REG_2.w0, 2

	ldi		TEMP_REG_4, 10500
	sbco	&TEMP_REG_4, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG, 4

	ldi		TEMP_REG_1.b0, 0x07	;cmp1
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	ldi		TEMP_REG_4.b0, 0x1f
	sbco	&TEMP_REG_4.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
	ldi		TEMP_REG_4.b0, 0x04

    ldi    TEMP_REG_1, 0x06    ; clear sync_en (sync0_en, sync1_en)
    ldi    TEMP_REG_2.w0, ICSS_IEP_SYNC_CTRL_REG
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4
    ldi    TEMP_REG_1, 0x07    ;  set sync_en (sync0_en, sync1_en)
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4

	;EDIO toggle
	;ldi32	TEMP_REG_1, 0x0B22E313
	;ldi		TEMP_REG_2.b0, 0x00
	;sbbo	&TEMP_REG_2.b0, TEMP_REG_1, 0, 1
	M_CLR_G2_EDIO31 TEMP_REG_1, TEMP_REG_2.b0
	; e/o debug

	;clears reset bit in control byte
	;lbco	&TEMP_REG_1.b0, ICSS_SHARED_RAM_CONST, FILTER_CTRL, 1
	;clr		TEMP_REG_1.b0, TEMP_REG_1.b0, FILTER_CTRL_RESET_BIT
	;sbco	&TEMP_REG_1.b0, ICSS_SHARED_RAM_CONST, FILTER_CTRL, 1
	; clear the SMA buffer in memory ; 365ns to clear; Todo: timing
	;zero	&R1, 68	;clear R1..R17
	;loop	SYNC_CLR_PRU0_DMEM, 4	;clear 64 32-bit values
	;sbco	&R2, PRU1_DMEM_CONST, R1, 64
	;add		R1, R1, 64
SYNC_CLR_PRU0_DMEM:
	; clear SMA_SUM
	;sbco	&R2, ICSS_SHARED_RAM_CONST, SMA_SUM, 4
	; clear DEBUG_BUFFER_OFFSET
	;sbco	&R2, ICSS_SHARED_RAM_CONST, DEBUG_BUFFER_OFFSET, 2
	; restart SMA oversampling value
	;lbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, FILTER_OVERSAMPLING_CONFIG, 1
	;sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, FILTER_OVERSAMPLING_VALUE, 1
	qba		SYNC_HANDLER_AVERAGE_DONE

SYNC_HANDLER_AVERAGE:
	qbeq	SYNC_HANDLER_DELTA_T, PROTOCOL_STATE, IOEX_STATE

	; debug code to generate sync0 signal during SYNC state
	ldi		TEMP_REG_3, 10000
	lbco	&TEMP_REG_4, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG, 4
	add		TEMP_REG_4, TEMP_REG_4, TEMP_REG_3
	sbco	&TEMP_REG_4, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG, 4
	ldi		TEMP_REG_4.b0, 0x1f
	sbco	&TEMP_REG_4.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
	ldi		TEMP_REG_4.b0, 0x04
    ldi    TEMP_REG_1, 0x06    ; clear sync_en (sync0_en, sync1_en)
    ldi    TEMP_REG_2.w0, ICSS_IEP_SYNC_CTRL_REG
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4
    ldi    TEMP_REG_1, 0x07    ;  set sync_en (sync0_en, sync1_en)
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4

	; DevNote: Jump over averaging
	;qba		SYNC_HANDLER_DELTA_T

	;add		SYNC_STATE_REG, SYNC_STATE_REG, 1				; count used for averaging
	;lbco	&R1.w0, ICSS_SHARED_RAM_CONST, FILTER_CTRL, 2		; load FILTER_CTRL (.b0) and FILTER_OVERSAMPLING_CONFIG (.b1)
	;qbbs	SYNC_HANDLER_INIT, R1.b0, FILTER_CTRL_RESET_BIT
	;qbne	SYNC_HANDLER_AVERAGE_DONE, SYNC_STATE_REG, R1.b1 ; FILTER_OVERSAMPLING_CONFIG
	;qbbs	SYNC_HANDLER_SKIP_UPDATE, R1.b0, FILTER_CTRL_NO_COMP
	;qbbc	SYNC_HANDLER_DELTA_T, R1.b0, FILTER_CTRL_NO_COMP_AT_STARTUP
	;qbeq	SYNC_HANDLER_DELTA_T, PROTOCOL_STATE, IOEX_STATE
;SYNC_HANDLER_SKIP_UPDATE:
	;ldi		SYNC_STATE_REG, SYNC_STATE_OPERATIONAL
	;qba		SYNC_HANDLER_AVERAGE_DONE

	.if $defined(AMA)	; AMA approximate moving average filter
SYNC_HANDLER_DELTA_T:
	;PRU_SET_GPIO0_28	R0, R1
	lbco	&R2, ICSS_SHARED_RAM_CONST, SYNC_DELAY, 4	; value sent by master that has the expected receive time (IEP timer configured linear)
	qbne	SYNC_HANDLER_GET_TS, PROTOCOL_STATE, IOEX_STATE
	ldi		R2.w2, 0
	lbco	&R2.w0, ICSS_SHARED_RAM_CONST, SYNC_DELAY_1ST, 2	; value sent by master that has the expected receive time (IEP timer configured linear)
SYNC_HANDLER_GET_TS:
	lbco	&R3, ICSS_SHARED_RAM_CONST, SYNC_TS_RX, 4
	; calculate delta_t
	sub		TEMP_REG_2, R2, R3
	; ToDo debug: create copy of delta_t into R4
	mov		R4, TEMP_REG_2
	; approximate moving average filter implementation
	; TEMP_REG_2 = delta_t_new
	lbco	&TEMP_REG_3, ICSS_SHARED_RAM_CONST, AMA_Xm_update, 4	; TEMP_REG_3 = AMA_Xm_update (Xm_update_N) = xma(i-1) * N
	lbco	&TEMP_REG_6.b0, ICSS_SHARED_RAM_CONST, AMA_N, 3			; TEMP_REG_6.b0 AMA_N
	; Approximate moving average equation
	; xma(i) = xma(i-1) + ((delta_t(i)/N) - xma(i-1)/N; This is the AMA formula
	; xma(i) * N  = xma(i-1) * N + ((delta_t(i)) - xma(i-1); This formula is used to prevent rounding errors introduced by divide N
	; TEMP_REG_3 = xma(i-1) * N
	; TEMP_REG_6.b0 = N
	; TEMP_REG_2 = delta_t(i)
	DIV_N	TEMP_REG_4, TEMP_REG_3, TEMP_REG_6.b0	; TEMP_REG_4 = xma(i-1) = TEMP_REG_3 / N
	sub		TEMP_REG_2, TEMP_REG_2, TEMP_REG_4		; TEMP_REG_2 = delta_t(i) - xma(i-1)
	add		TEMP_REG_3, TEMP_REG_3, TEMP_REG_2		; TEMP_REG_3 = xma(i) * N = xma(i-1) * N + [delta_t(i) - xma(i-1)]

	DIV_N	TEMP_REG_4, TEMP_REG_3, TEMP_REG_6.b0 	; TEMP_REG_4 = xma(i)
	ABS		TEMP_REG_5, TEMP_REG_4					; TEMP_REG_5 = |xma(i)|
	lsl		TEMP_REG_2, TEMP_REG_4, TEMP_REG_6.b0	; TEMP_REG_2 = integer(xma(i)) * N ; this is the integer IEP value getting applied
	sub		TEMP_REG_3, TEMP_REG_3, TEMP_REG_2		; TEMP_REG_3 = xma(i) * N - integer(xma(i)) * N
	sbco	&TEMP_REG_3, ICSS_SHARED_RAM_CONST, AMA_Xm_update, 4	; store 'xma(i-1) * N' in AMA_Xm_update_N

	ldi		TEMP_REG_1.b0, IEP_FAST_INCREMENT		; xma(i) > 0
	qbbc	SYNC_HANDLER_XMA_POSITIVE, TEMP_REG_4, 31
	ldi		TEMP_REG_1.b0, IEP_SLOW_INCREMENT		; xma(i) < 0
SYNC_HANDLER_XMA_POSITIVE:
SYNC_HANDLER_UPDATE:
	; set compensation count value
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG + 1, 1		; write 3 or 5 to cmp_inc
	sbco	&TEMP_REG_5.w0, ICSS_IEP_CONST, ICSS_IEP_COMPEN_REG, 2				; set compen_cnt

	; Debug: store the sum of total IEP correction value
	lbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, AMA_IEP_CORR, 4
	add		TEMP_REG_1, TEMP_REG_1, TEMP_REG_5.w0
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, AMA_IEP_CORR, 4

	;PRU_CLR_GPIO0_28	R0, R1

	; store timestamp history for debug purpose
	; debug store TEMP_REG_5 (delta_t_new)
	mov		TEMP_REG_2, R4
	lbco	&R9.w0, ICSS_SHARED_RAM_CONST, SMA_WRITE_OFFSET, 2
	ldi		R9.w2, TS_BUFFER		; icss_g1 0x0b11 0400
	add		R10.w0, R9.w0, R9.w2	; pointer to IEP_Update
	ldi		R11.w0, 1024			; pointer to dT
	add		R11.w0, R10.w0, R11.w0
	;lbco	&TEMP_REG_2.w2, ICSS_SHARED_RAM_CONST, SORTE_CYCLE_CNT, 2	; load cycle counter
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, R10.w0, 1*4	; iep_update value
	sbco	&TEMP_REG_2, ICSS_SHARED_RAM_CONST, R11.w0, 1*4	; delta_t
	; buffer boundaries
	ldi		R9.w2, 1024 * 1*4; 1000 entries in buffer
	add		R9.w0, R9.w0, 1*4
	; buffer full?
	qblt	sync_debug_buffer_full, R9.w0, R9.w2
	sbco	&R9.w0, ICSS_SHARED_RAM_CONST, SMA_WRITE_OFFSET, 2
	qba		SYNC_HANDLER_AVERAGE_DONE
sync_debug_buffer_full:
	.if $defined(remove)
	; stop PRU1
	ldi32	TEMP_REG_2, 0x0b124000
	lbbo	&TEMP_REG_1, TEMP_REG_2, 0, 4
	clr		TEMP_REG_1, TEMP_REG_1, 1	; clear ENABLE
	sbbo	&TEMP_REG_1, TEMP_REG_2, 0, 4
	; stop PRU0
	ldi32	TEMP_REG_2, 0x0b122000
	lbbo	&TEMP_REG_1, TEMP_REG_2, 0, 4
	clr		TEMP_REG_1, TEMP_REG_1, 1	; clear ENABLE
	sbbo	&TEMP_REG_1, TEMP_REG_2, 0, 4
	; e/o debug
	.endif
	nop		; used for breakpoint

	.if $defined (remove)
	; debug
	; debug store TEMP_REG_1.b0 (CMPEN value), TEMP_REG_2 (AMA_Xm_update), TEMP_REG_3 (AMA_Xm_update_N), TEMP_REG_5 (delta_t_new)
	mov		TEMP_REG_4, TEMP_REG_5
	lbco	&R9.w0, ICSS_SHARED_RAM_CONST, SMA_WRITE_OFFSET, 2
	ldi		R9.w2, TS_BUFFER
	add		R10.w0, R9.w0, R9.w2
	lbco	&TEMP_REG_2.w2, ICSS_SHARED_RAM_CONST, SORTE_CYCLE_CNT, 2	; load cycle counter
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, R10.w0, 4*4
	ldi		R9.w2, 200 * 4*4	; 200 entries in buffer
	add		R9.w0, R9.w0, 4*4
	; buffer full?
	qble	sync_debug_buffer_full, R9.w0, R9.w2
	sbco	&R9.w0, ICSS_SHARED_RAM_CONST, SMA_WRITE_OFFSET, 2
sync_debug_buffer_full:
	; e/o debug
	.endif

SYNC_HANDLER_AVERAGE_DONE:
	RET

	.else ; below is the code for no AMA filter
SYNC_HANDLER_DELTA_T:
	lbco	&R2, ICSS_SHARED_RAM_CONST, SYNC_DELAY, 4	; value sent by master that has the expected receive time (IEP timer configured linear)
	qbne	SYNC_HANDLER_GET_TS, PROTOCOL_STATE, IOEX_STATE
	ldi		R2.w2, 0
	lbco	&R2.w0, ICSS_SHARED_RAM_CONST, SYNC_DELAY_1ST, 2	; value sent by master that has the expected receive time (IEP timer configured linear)
SYNC_HANDLER_GET_TS:
	lbco	&R3, ICSS_SHARED_RAM_CONST, SYNC_TS_RX, 4
	; calculate delta_t
	sub		TEMP_REG_3, R2, R3
	mov		TEMP_REG_2, TEMP_REG_3

	; init fast compensation count
	ldi		TEMP_REG_1.b0, IEP_FAST_INCREMENT
	; check if positive or negative
	qbbc	SYNC_HANDLER_WRITE_VALUES, TEMP_REG_2, 31
;SYNC_HANDLER_AV_NEGATIVE:
	not		TEMP_REG_2, TEMP_REG_2
	add		TEMP_REG_2, TEMP_REG_2, 1
	; set slow compensation count
	ldi		TEMP_REG_1.b0, IEP_SLOW_INCREMENT
SYNC_HANDLER_WRITE_VALUES:
	; threshold value if >= xx
	;qbge	SYNC_HANDLER_AVERAGE_DONE, TEMP_REG_2, 11
	mov		TEMP_REG_4, TEMP_REG_2
	qbge	sync_debug_update_buffer, TEMP_REG_2, 11
	; limit compensation value to 4; When compensation <30, then use delta time
	;qble	SYNC_HANDLER_SET_COMP, TEMP_REG_2, 30
	ldi		TEMP_REG_2.w0, 3

	; filter
	;lsr		TEMP_REG_2, TEMP_REG_2, 2
	; set compensation count value
SYNC_HANDLER_SET_COMP:
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG + 1, 1		; write 3 or 5 to cmp_inc
	sbco	&TEMP_REG_2.w0, ICSS_IEP_CONST, ICSS_IEP_COMPEN_REG, 2				; set compen_cnt
sync_debug_update_buffer:
	; debug
	; debug store TEMP_REG_1.b0 (CMPEN value), TEMP_REG_2 (cmpen_cnt), TEMP_REG_3 (delta_t_new), TEMP_REG_4 (not used)
	lbco	&R9.w0, ICSS_SHARED_RAM_CONST, SMA_WRITE_OFFSET, 2
	ldi		R9.w2, TS_BUFFER
	add		R10.w0, R9.w0, R9.w2
	lbco	&TEMP_REG_2.w2, ICSS_SHARED_RAM_CONST, SORTE_CYCLE_CNT, 2	; load cycle counter
	sbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, R10.w0, 4*4
	ldi		R9.w2, 200 * 4*4	; 200 entries in buffer
	add		R9.w0, R9.w0, 4*4
	; buffer full?
	qble	sync_debug_buffer_full, R9.w0, R9.w2
	sbco	&R9.w0, ICSS_SHARED_RAM_CONST, SMA_WRITE_OFFSET, 2
sync_debug_buffer_full:
	; e/o debug

SYNC_HANDLER_AVERAGE_DONE:
	RET
	.endif ;e/o no AMA filter
