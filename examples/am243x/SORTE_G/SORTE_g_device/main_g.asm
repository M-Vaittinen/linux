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
;     file:   main.asm
;
;     brief:  SORTE_g implementation
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
    .include "firmware_version.inc"


;CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

  	.global     main
  	.global		FN_TASK_MANAGER_SETUP
  	.global		FN_EVENT_HANDLER
  	.global		FN_LINE_DELAY_MASTER
  	.global 	FN_SYNC_HANDLER

	.sect	".text:main"

;PRU0 .set 0  moved to build options


;********
;* MAIN *
;********

main:
	JMP		header	; jump over the firmware version information
	.word   ICSS_FIRMWARE_RELEASE_1
	.word   ICSS_FIRMWARE_RELEASE_2
header:
; disable task manager
    tsen 0


	;.if $defined(remove)
; debug code start
; disable TX_FIFO, TX_L2 and TX_L1 to recover from previous debug stall checking TX_L2 fifo depth
; Set TXCFG0 - Port 1
	ldi		r0.w0, 0x0800
	ldi		r0.w2, 0x0000
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0, 4
	nop
	nop
	nop
	ldi		r0.w0, 0x0228
	ldi		r0.w2, 0x0001
	sbco    &r0, ICSS_MII_G_RT_CFG_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG, 4

; Set ICSS_G_CFG - MII0/1 - RGMII Mode, RX_L2_G_EN - Enabled, TX_L2_EN - Enabled, TX_L1_EN- Enabled, Fiber Mode
	ldi		r0.w0, 0x022b
	ldi		r0.w2, 0x0001
	sbco    &r0, ICSS_MII_G_RT_CFG_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG, 4

; enable and configure pa_stat, all 32 bit counters
    ldi32   r0,0x0003C000
    ldi32   r1,0x80000000
	sbbo    &r1,r0,8,4
; clear counters which are located at 0x27000 - 0x273ff
	sbbo    &r1,r0,4,4


; set the MII_RT Configuration Registers for Transmit -  enable, auto preamble, tx port 0, 32 bit mode
; Set TXCFG0 - Port 1
	ldi		r0.w0, 0x0803		; 0x0903
	ldi		r0.w2, 0x0000
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0, 4
; Set TXCFG1 - Port 2
	ldi		r0.w0, 0x0903		; 0x0803
	ldi		r0.w2, 0x0000
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1, 4
; Set RXCFG0 - RX_EOF self clear, SFD, no auto fwd, no swap, rxl2 enable, port 0, cut-preamble, data rdy mode, rx enable
	ldi		r0.w0, 0x0015		; 0x0055
	ldi		r0.w2, 0x0000
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0, 4
; Set RXCFG1 RX_EOF self clear, SFD, no auto fwd, no swap, rxl2 enable, port 1, cut-preamble, data rdy mode, rx enable
	ldi		r0.w0, 0x001f       ; 0x001d
	ldi		r0.w2, 0x0000
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1, 4
; Set GPCFG0 - ICSS_CFG_CONST, GPI mode = MII_RT, GP_MUX_SEL = MII mode
	ldi		r0.w0, 0x0003
	ldi		r0.w2, 0x0800
	sbco	&r0, ICSS_CFG_CONST, 0x08, 4
; Set TX_IPG0/ TX_IPG1
	ldi		r0.w2, 0x0000
	ldi		r0.w0, 0x018
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0, 4
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1, 4
; Set ICSS_G_CFG - MII0/1 - RGMII Mode, RX_L2_G_EN - Enabled, TX_L2_EN - Enabled, TX_L1_EN- Enabled, Fiber Mode
;	ldi		r0.w0, 0x022b
;	ldi		r0.w2, 0x0001
;	sbco    &r0, ICSS_MII_G_RT_CFG_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG, 4
; Set RGMII_CFG - Fullduplex, 1000 Mbits, Link Up
	ldi		r0.w0, 0x00DD
	ldi		r0.w2, 0x0066
	sbco    &r0, ICSS_MII_G_RT_CFG_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG, 4
; Disable Global Count
	lbco	&r0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG , 4
	clr		r0, r0, 0
	sbco	&r0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG , 4
; Reset the Counters
	ldi		r0.w0, 0xffff
	ldi 	r0.w2, 0xffff
	sbco	&r0, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
    sbco	&r0, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG+4, 4

; disable cmp0 resset
	ldi		TEMP_REG_1.b0, 0x10
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

; set IEP clock to ocp clock
    ldi32   r0, 0x26030
	ldi     r1, 1
    sbbo    &r1, r0, 0, 2

; set VBUSP sync mode to reduce latency from MSMC
    ldi32   r0, 0x2603C
	ldi     r1, 1
    sbbo    &r1, r0, 0, 2

; Enable Global Count
	ldi		r0.w0, 0x0041
	sbco	&r0.w0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG , 2


;reset RX (Bit 18) and clear EOF(Bit 22), rx_error_clr (23)
    ldi      r31.b2, 0xF4

;reset TX Fifo (bit 30)
    ldi      r31.b3, 0x40

	;.endif ;e/o remove







; e/o debug code

    ;EDIO toggle
	ldi32	TEMP_REG_1, 0x0B22E313
	ldi		TEMP_REG_2.b0, 0x00
	sbbo	&TEMP_REG_2.b0, TEMP_REG_1, 0, 1

	fill	&TEMP_REG_1, 4
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP0_REG, 4
	ldi		TEMP_REG_1.b0, 0x3	;cmp0 + CMP_RESET_EN
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

;reset task manager
	.if $defined(PRU0)
    ldi32   TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_BASE	; TM base
	.else
    ldi32   TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_BASE	; TM base
	.endif
    ldi		TEMP_REG_3, 0xfff
    sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4
    xin     TM_YIELD_ID, &R0.b3, 1
    ldi		TEMP_REG_3, 0
    sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4

	zero	&r0, 120

	LDI		R30.w2, 0xFFFF	; set forward mask to accept data from R30.w0

	ldi		PROTOCOL_STATE_CNT, IDLE_STATE
	call	FN_TASK_MANAGER_SETUP
	M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RESET_TXFIFO)

	; trigger MDIO event in the PRU to read port information
	.if $defined(PRU0)
	ldi32	TEMP_REG_1, 0x00200000	; Set MDIO_MII_LINK[1]
	ldi		TEMP_REG_2.w0, ICSS_INTC_SRSR1
	sbco	&TEMP_REG_1, ICSS_INTC_CONST, TEMP_REG_2.w0, 4
	.else
	ldi32	TEMP_REG_1, 0x00000200	; Set MDIO_MII_LINK[0] bit
	ldi		TEMP_REG_2.w0, ICSS_INTC_SRSR1
	sbco	&TEMP_REG_1, ICSS_INTC_CONST, TEMP_REG_2.w0, 4
	.endif

IDLE_LOOP:
	call	FN_EVENT_HANDLER
	nop
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18

;LINED_STATE_LOOP:
	qbbs	SYNC_STATE_LOOP, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	qbne	SYNC_STATE_LOOP, PROTOCOL_STATE, LINED_STATE
	call	FN_LINE_DELAY_MASTER

SYNC_STATE_LOOP:
	qbbc	IOEX_STATE_LOOP, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	qbne	IOEX_STATE_LOOP, PROTOCOL_STATE, SYNC_STATE
	call	FN_SYNC_HANDLER

IOEX_STATE_LOOP:
	qbbc	CHECK_FOR_SWITCH_STATE, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	qbne	CHECK_FOR_SWITCH_STATE, PROTOCOL_STATE, IOEX_STATE
	call	FN_SYNC_HANDLER

CHECK_FOR_SWITCH_STATE:
	qbbc	IDLE_LOOP, DEVICE_STATUS, SWITCH_STATE
	call	FN_TASK_MANAGER_SETUP
	qbeq	IDLE_LOOP, R2, 0

; *********************************************
; test code below
; *********************************************
	;M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR )
	qbbc	IDLE_LOOP, DEVICE_STATUS, SWITCH_STATE
	call	FN_TASK_MANAGER_SETUP
	qba		IDLE_LOOP
	M_SET_CMD	D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RX_SOF_CLEAR | D_RX_SFD_CLEAR | D_RX_EOF_CLEAR | D_RESET_TXFIFO
	;M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR )
	qba		IDLE_LOOP

iep_debug:
	ldi		R0.w0, 0x0403
	ldi		R0.w2, 0x0005
	ldi		R1.w0, 0x0001
	nop
	nop
	nop
	nop
	nop
	; comp = 3
	lbco	&R2, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	sbco	&R0.b0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG+1, 1
	sbco	&R1.w0, ICSS_IEP_CONST, ICSS_IEP_COMPEN_REG, 2
	nop
	nop
	nop
	nop
	lbco	&R3, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	sub		R2, R3, R2
	; comp = 4
	lbco	&R3, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	sbco	&R0.b0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG+1, 1
	sbco	&R1.w0, ICSS_IEP_CONST, ICSS_IEP_COMPEN_REG, 2
	nop
	nop
	nop
	nop
	lbco	&R4, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	sub		R3, R4, R3
	; comp = 5
	lbco	&R4, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	sbco	&R0.b0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG+1, 1
	sbco	&R1.w0, ICSS_IEP_CONST, ICSS_IEP_COMPEN_REG, 2
	nop
	nop
	nop
	nop
	lbco	&R5, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	sub		R4, R5, R4
	qba		iep_debug

idle_test:
	LDI		R31.b0, PRU2PRU_INTR1
	xin		RXL2_BANK0_ID, &r2, RANGE_R2_R18
	xin		RXL2_BANK1_ID, &r2, RANGE_R2_R18
	qba		idle_test
	M_SET_CMD	D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RX_SOF_CLEAR | D_RX_SFD_CLEAR | D_RX_EOF_CLEAR | D_RESET_TXFIFO


	tsen 0
	M_ENABLE_TXL2
	M_SET_MIIRT_HR_HSBS ; forward port must be HR_HS

send_arp_frame:
	; ARP frame
	ldi32	R2, 0xFFFFFFFF
	ldi32	R3, 0xADDEFFFF
	ldi32	R4, 0xAA55EFBE
	ldi32	R5, 0x01000608
	ldi32	R6, 0x04060008
	ldi32	R7, 0x04030201
	ldi32	R8, 0x08070605
	ldi32	R9, 0x0c0b0a09
	ldi32	R10, 0x87654321
	ldi32	R11, 0x44332211
	ldi32	R12, 0x88776655
	ldi32	R13, 0xdeadbeef
	ldi32	R14, 0x12345678
	ldi32	R15, 0xaabbccdd
	ldi32	R16, 0xeeff1234
	ldi32	R17, 0xaa554321

	xout	TXL2_ID, &r2, 60
	nop
	nop

check_tx_l2_fifo_64b_1:
	xin          TXL2_ID, &r19, 4
	qbne   check_tx_l2_fifo_64b_1, r19.b2, 0


;	loop	txdelay, 0x33 + 32*8/4	; 0x33 is start delay; 32 bytes * 8ns per byte / 4 ns PRU cycle
;	nop
;txdelay:
	M_SET_CMD	D_TX_EOF | D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD

	ldi32	R0, 10000/4	; wait 10us
	loop	packetdelay, R0
	nop
packetdelay:
	qba		send_arp_frame


	.if $defined(remove)
; C28 for shared memory needs initialization through offset register
 .if $defined(PRU0)
    ldi     r2, 0x0100
 .else
    ldi     r2, 0x0110
 .endif
    sbco    &r2, c11, 0x28, 2

; set receive tasks address
    ldi     r3.w0, $CODE(rx_fb)
    sbbo    &r3.w0, r2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S1, 2

    ldi     r3.w0, $CODE(rx_b2)
    sbbo    &r3.w0, r2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S2, 2

    ldi     r3.w0, $CODE(rx_nb)
    sbbo    &r3.w0, r2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S3, 2

    ldi     r3.w0, $CODE(rx_eof)
    sbbo    &r3.w0, r2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S4, 2

    ldi     r3, 0x7ffa
;    ldi     r3, 0x7fff
    sbbo    &r3,r2, 0x30, 2


; set the MII_RT Configuration Registers for Transmit -  enable, auto preamble, tx port 0, 32 bit mode
; Set TXCFG0 - Port 1
	ldi		r0.w0, 0x0903		; 0x0903
	ldi		r0.w2, 0x0000
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0, 4
; Set TXCFG1 - Port 2
	ldi		r0.w0, 0x0803		; 0x0803
	ldi		r0.w2, 0x0000
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1, 4
; set the MII_RT Configuration Registers for Transmit -  Auto preamble
; Set RXCFG0 - RX_EOF self clear, SFD, no auto fwd, no swap, rxl2 enable, port 0, cut-preamble, data rdy mode, rx enable
	ldi		r0.w0, 0x0015		; 0x0015
	ldi		r0.w2, 0x0000
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0, 4
; Set RXCFG1 RX_EOF self clear, SFD, no auto fwd, no swap, rxl2 enable, port 1, cut-preamble, data rdy mode, rx enable
	ldi		r0.w0, 0x001d       ; 0x001d
	ldi		r0.w2, 0x0000
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1, 4
; Set GPCFG0 - ICSS_CFG_CONST, GPI mode = MII_RT, GP_MUX_SEL = MII mode
	ldi		r0.w0, 0x0003
	ldi		r0.w2, 0x0800
	sbco	&r0, ICSS_CFG_CONST, 0x08, 4
; Set TX_IPG0/ TX_IPG1
	ldi		r0.w2, 0x0000
	ldi		r0.w0, 0x28
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0, 4
	sbco    &r0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1, 4
; Set ICSS_G_CFG - MII0/1 - RGMII Mode, RX_L2_G_EN - Enabled, TX_L2_EN - Enabled, TX_L1_EN- Enabled, Fiber Mode
	ldi		r0.w0, 0x022b
	ldi		r0.w2, 0x0001
	sbco    &r0, ICSS_G_CFG_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG, 4
; Set RGMII_CFG - Fullduplex, 1000 Mbits, Link Up
	ldi		r0.w0, 0x00DD
	ldi		r0.w2, 0x0066
	sbco    &r0, ICSS_G_CFG_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG, 4
; Disable Global Count
	lbco	&r0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG , 4
	clr		r0, r0, 0
	sbco	&r0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG , 4
; Reset the Counters
	ldi		r0.w0, 0xffff
	ldi 	r0.w2, 0xffff
	sbco	&r0, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
; set IEP clock to ocp clock
    ldi32   r0, 0x26030
	ldi     r1, 1
    sbbo    &r1, r0, 0, 2

; set VBUSP sync mode to reduce latency from MSMC
    ldi32   r0, 0x2603C
	ldi     r1, 1
    sbbo    &r1, r0, 0, 2

; Enable Global Count
	ldi		r0.w0, 0x0041
	sbco	&r0.w0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG , 2

; eof shared register setup

;reset RX (Bit 18) and clear EOF(Bit 22), rx_error_clr (23)
    or      r31.b2, r31.b2, 0xF4

;reset TX Fifo (bit 30)
    or      r31.b3, r31.b3, 0x40

; enable task manager instrunction

;    .word 0x32800000

; set min frame count to 6 bytes (2 PE + 4 payload bytes) on both ports
; todo: N=N+1 bytes after SFD and including CRC, what if CRC is less than 4 bytes? Need to test
	ldi		r2.b0, 0x06
	sbco 	&r2.b0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0, 1
	sbco 	&r2.b0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1, 1


idle_loop:

	qba	idle_loop


	.endif ;e/o remove
