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
;     file:   main_PRU0.asm
;
;     brief:  SORTE_g master
;             uses PRU0 and RGMII interface with TX_L2 fifo
;             runs transmit in foreground and receive in task manager
;
;
;    Version        Description                                Author
;     0.1           Created                                    Thomas Leyrer
;     0.2           SORTE_G changes

;************************************* Includes *************************************
 .if $defined(PRU0)

; includes C header file to be shared with PRU assembler
   .cdecls C,NOLIST
%{
	#include "cslr_icss_g.h"

%}

; includes from \icss-g-industrial-fw\pru_fw_common
;     .include "pru_macros.h"

; includes from sorte inc folder
	.include "icss_regs.inc"
    .include "sorte_g_pru_master_register.inc"
	.include "sorte_g_master_cfg.inc"
	.include "sorte_g_packet_definition.inc"
	.include "sorte_g_host_interface.inc"
	.include "sorte_macros.inc"
	.include "firmware_version.inc"

; inlcudes from local source folder
;	.include "include/icss/icss_defines.inc"



;CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

	.global     rx_fb
	.global     rx_b2
	.global     rx_nb
	.global     rx_eof
	.global     rxio_fb
	.global     rxio_b2
	.global     rxio_nb
	.global     rxio_eof

;	.global		init_system
	.global     main
    .sect       ".text"

;PRU0 .set 0  moved to build options
;FULL_PREAMBLE 	.set 	0
ICSS_REV2	 	.set    1


;********
;* MAIN *
;********

main:
	zero	&r0, 120
; disable task manager
    .word   0x32000000

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
	
 .if $defined(PRU0)
    ldi32   TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_BASE       ; TM base
 .else
    ldi32   TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_BASE       ; TM base
 .endif
    ldi     TEMP_REG_3, 0x0fff
    sbbo    &TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4
    xin     TM_YIELD_XID, &R0.b3,1
    ldi     TEMP_REG_3, 0
    sbbo    &TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4


; Set ICSS_G_CFG - MII0/1 - RGMII Mode, RX_L2_G_EN - Enabled, TX_L2_EN - Enabled, TX_L1_EN- Enabled, Fiber Mode
	ldi		r0.w0, 0x022b
	ldi		r0.w2, 0x0001
	sbco    &r0, ICSS_MII_G_RT_CFG_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG, 4

; C28 for shared memory needs initialization through offset register
 .if $defined(PRU0)
    ldi     r2, 0x0100
 .else
    ldi     r2, 0x0110
 .endif
    sbco    &r2, c11, 0x28, 2

; configure task manager for receive processing

 .if $defined(PRU0)
    ldi32   r2, 0x0002a000	; TM base
 .else
    ldi32   r2, 0x0002a200	; TM base
 .endif
    ldi     r3, 0x0079		; Eth mode,T1_S1 = FB, T1_S2 = NB, T1_S3 = NB, T1_S4 = LB
    sbbo    &r3, r2,0, 2

; set receive tasks address
    ldi     r3.w0, $CODE(rx_fb)
    sbbo    &r3.w0,r2,0x0c, 2

    ldi     r3.w0, $CODE(rx_b2)
    sbbo    &r3.w0,r2,0x10, 2

    ldi     r3.w0, $CODE(rx_nb)
    sbbo    &r3.w0,r2,0x14, 2

    ldi     r3.w0, $CODE(rx_eof)
    sbbo    &r3.w0,r2,0x18, 2

; set receive task trigger same on FB1, FB2, NB = 32

    ldi     r3, 0x7fff
    sbbo    &r3,r2, 0x30, 2

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

; enable EDIO out32 (pin T27 - routed to pin1 of J21 on AM654x IDK)
; output is software programmable by setting SW_DATA_OUT_UPDATE and OUTVALID_OVR_EN

    ldi32   r0, 0x0002E300
    ldi     r1.w0, 0x0023
    sbbo    &r1, r0, 0x18, 1
; reset EDIO out31
    ldi     r29, 0x8000
    sbbo    &r29.b1, r0, 0x13, 1
    mov     r28, r0

; eof shared register setup

; init receive diagnostic array for time stamp min / avg / max 
; use r2..r4 for init data 0xfffffff/ 0 / 0 
    zero    &r3, 8    
	fill    &r2, 4
    ldi     r5.w0, IORX_TS_DIAG
	loop    INIT_RX_DIAG,32
    sbco    &r2, ICSS_DMEM0_CONST, r5.w0, 12
    add     r5.w0, r5.w0, 12
INIT_RX_DIAG:

; set SORTE control register to global enable, offline engineering and no debug/diags/retrain.
    ldi     r1.b2,0x01
	ldi		r1.w0, SORTE_REG_INTERFACE
	add 	r1.w0, r1.w0, CTRL_REG_OFFSET
	sbco    &r1.b2, ICSS_DMEM0_CONST, r1.w0, 1
	
; set cycle time to 10 us 
; Save the Calculated Cycle Time in Device Data Memory
    ldi32   TEMP_REG_1, 3996
	ldi     TEMP_REG_2.w0, SORTE_REG_INTERFACE
	add     TEMP_REG_2.w0, TEMP_REG_2.w0, PARAM_DATA_OFFSET
	sbco    &TEMP_REG_1, ICSS_DMEM0_CONST, TEMP_REG_2.w0, 4
	
; set the total OUT Data for Device from Memory
    ldi     r0.w0, 32
	ldi 	TEMP_REG_3.w0, DISCOV_RX_OFFSET
	add		TEMP_REG_3.w0, TEMP_REG_3.w0, 0x08
	sbco	&r0.w0, ICSS_DMEM0_CONST, TEMP_REG_3.w0, 2

; set device count	
    ldi     TEMP_REG_3.w0, 4
	ldi     TEMP_REG_1.w0, DISCOV_RX_OFFSET
	add 	TEMP_REG_1.w0, TEMP_REG_1.w0, 0x04			; Device Count
	sbco    &TEMP_REG_3.w0, ICSS_DMEM0_CONST, TEMP_REG_1.w0, 2

;reset RX (Bit 18) and clear EOF(Bit 22), rx_error_clr (23)
    ldi      r31.b2, 0xF4

;reset TX Fifo (bit 30)
    ldi      r31.b3, 0x40

; enable task manager instruction
    .word 0x32800000

; set min frame count to 6 bytes (2 PE + 4 payload bytes) on both ports
; todo: N=N+1 bytes after SFD and including CRC, what if CRC is less than 4 bytes? Need to test
	ldi		r2.b0, 0x06
	sbco 	&r2.b0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0, 1
	sbco 	&r2.b0, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1, 1

; test if there is an active link on the port

; test if there is an active link on the port using mdio interface
;    ldi     TEMP_REG_2.w0, MDIO_PHY_CONFIG_OFFSET
;    lbco    &TEMP_REG_2.w0, C28, TEMP_REG_2.w0, 2

LED_SET_STAT_RED:
	ldi32	TEMP_REG_1, 0x00601040
	ldi		TEMP_REG_2, 0x40
	sbbo	&TEMP_REG_2, TEMP_REG_1, 0, 4
	ldi		TEMP_REG_2, 0x80
	sbbo	&TEMP_REG_2, TEMP_REG_1, 4, 4

IDLE_WAIT_FOR_LINK_ACTIVE:
    lbco    &TEMP_REG_1.b0, ICSS_MDIO_CONST, ICSS_MDIO_LINK, 1
; update to fit phy address
    qbbc    IDLE_WAIT_FOR_LINK_ACTIVE,TEMP_REG_1.b0, 3
;    qbbc    IDLE_WAIT_FOR_LINK_ACTIVE,TEMP_REG_1.b0, TEMP_REG_2.b0

; check if enabled, enabling done at startup of pru firmware.
    lbco    &TEMP_REG_1.b0, ICSS_DMEM0_CONST, CTRL_REG_OFFSET , 1
    qbbc    IDLE_WAIT_FOR_LINK_ACTIVE,TEMP_REG_1.b0, 0
	
; set iteration count for DISCOVERY state
	ldi 	R25.b0, 10

STATE_DISCOVERY_ITERATION:

; reset RX- and TX-FIFO and clear TX errors
	ldi		r31.w2, (D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RESET_TXFIFO)

; start 1 ms timer using PRU cycle counter
; disable cycle counter to clear it
; bit 3 is cycle counter enable, need to keep PRU enable
    ldi32   r26, 0x22000
    ldi		TEMP_REG_1.w0, 0x0b03
    sbbo    &TEMP_REG_1.b0, r26, 0, 1
    zero    &TEMP_REG_2, 4

; clear cycle counter
    sbbo    &TEMP_REG_2, r26, 0x0c,4

; enable cycle counter
    sbbo    &TEMP_REG_1.b1, r26, 0, 1

; clear previous cycle counter
	zero    &TEMP_REG_3, 4
; clear bytes received
	zero    &TEMP_REG_6, 2
	zero    &TEMP_REG_6.b3, 1

; send DISCOVERY packet header

; send discovery packet using TX_L2 Fifo
; ------------------------------------------------------------------------------
; state: DISCOVERY
; Regsiter defines:
; n = TEMP_REG_4.b0 (iterations)
; m = TEMP_REG_4.b1 (counter for 32 bytes blocks in payload field)
; previous cycle counter = TEMP_REG_3
; tx_fifo_bytes_inserted = TEMP_REG_4.w2
; rx_bytes_received = TEMP_REG_6.w0
; rx_packet_cnt = TEMP_REG_6.b2
; rx_blocks_rcvd = TEMP_REG_6.b3
; ------------------------------------------------------------------------------
STATE_DISCOVERY:

; init r27 permanent register for receive handling in rx task
    ldi     r27, 0x0101
	
; 4 bytes header
	ldi32 	r2, DISC_HEADER

; state on last packet indicates last
	qbne	STATE_DISCOV_LAST, R25.b0, 1
	ldi		r2.b3, 0x08
STATE_DISCOV_LAST:

; 2 bytes: cnt (16 bit)
    ldi		r3.w0, 0
; 2 bytes: INPUT_BYTES_COUNT
;   mov		r3.w2, TEMP_REG_4.w0
    ldi		r3.w2, 0
; 2 bytes: OUTPUT_BYTES_COUNT
    ldi		r4.w0, 0
; 254 * 4 bytes to support 254 devices
    zero    &r4.w2,(64-10)
;    ldi     r10.w0, 0xdeaf
; send first 64 bytes to TX_L2 FIFO
    xout    TXL2_XID, &r2, 64
; clear all bytes
    zero    &r2,64
; 16 + 64 byte + 2 byes
;    loop endloop_disc, 16
    ldi     r20, 16
check_tx_l2_fifo_64b_1:
	xin		TXL2_XID, &r19, 4
	qbne	check_tx_l2_fifo_64b_1, r19.b2, 0

    xout    TXL2_XID, &r2, 64
; wait one cycle before TX_L2 fifo level is updated
    sub     r20,r20, 1
    qbne    check_tx_l2_fifo_64b_1,r20, 0
; endloop_disc:

; remaining 2 bytes
    xout    TXL2_XID, &r2, 2
;    nop
;    nop
; send eof and generate crc
;    or      R31.b3, R31.b3, 0x2c	;0x2c
	M_CMD16	(D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD | D_TX_EOF)

; wait for eof of packet. Last block receive resets R27 to 0
disc_wait_rx_eof:
    qbne    disc_wait_rx_eof,r27, 0

; wait 1 ms ( 250'000 cycles)
    ldi32     r1, 250000/2
disc_wait_1ms:
    sub     r1,r1,1
    qbne    disc_wait_1ms, r1, 0
    sub		r25.b0,r25.b0,1
	qbne    STATE_DISCOVERY_ITERATION,r25.b0, 0

; disable task manager to support foreground receive
    .word   0x32000000
; disable task manager for receive processing
 .if $defined(PRU0)
    ldi32   r2, 0x0002a000	; TM base
 .else
    ldi32   r2, 0x0002a200	; TM base
 .endif
    ldi     r3, 0x0000		; Eth mode,T1_S1 = FB, T1_S2 = NB, T1_S3 = NB
    sbbo    &r3, r2,0, 4
	xin     TM_YIELD_XID, &R0.b3, 1

	ldi		TEMP_REG_1.w0, SORTE_REG_INTERFACE
	add 	TEMP_REG_1.w0, TEMP_REG_1.w0, CTRL_REG_OFFSET
	lbco    &TEMP_REG_2.w0, ICSS_DMEM0_CONST, TEMP_REG_1.w0, 1
	qbbs	Online_Cycle_Time, TEMP_REG_2.w0, 1
	qba 	STATE_PARAM								; Check for Online or Offline Engineering

; v0.2 added - start
; Include the Calculations for cycle time
; Based on the number of devices only
; TO DO: Add the device Data parameters also to calculate the total transfer time
; At 100 Mbps, 1 byte takes 80 ns to transmit, we allocate 4 bytes of IN Data - 1 us of Cycle Time
; So based on this each byte is given an additional cycle time of 125 ns for tranmission and reception
; Hence, Comparison for Cycle Time based on Device Data can be carried out
; Both In and Out Data is compared, Higher value is taken
; For device data(In or Out) less than 4 bytes, it is rounded to give 1us min time
; above that each byte is given 125 ns for transmission or reception
;
Online_Cycle_Time:
	ldi32 	TEMP_REG_2, 0
	ldi32   TEMP_REG_1, 0
	ldi32 	TEMP_REG_3, 0
	ldi     TEMP_REG_1.w0, DISCOV_RX_OFFSET
	add 	TEMP_REG_1.w0, TEMP_REG_1.w0, 0xC			; Device Count + 8 bytes preamble
	lbco    &TEMP_REG_2.w0, ICSS_DMEM0_CONST, TEMP_REG_1.w0, 2
	ldi 	TEMP_REG_3.w0, 0x3E8						; 1000 ns
; Point to First In Data at 0x12
	add 	TEMP_REG_2.w2, TEMP_REG_1.w0, 0x6
	ldi 	TEMP_REG_1.w0, 0
calc_cycle_time: 
	ldi		TEMP_REG_3.w2, 0
; Load IN Data
	lbco 	&TEMP_REG_4.w0, ICSS_DMEM0_CONST, TEMP_REG_2.w2, 2
	add 	TEMP_REG_2.w2, TEMP_REG_2.w2, 0x2
; Load OUT Data
	lbco 	&TEMP_REG_4.w2, ICSS_DMEM0_CONST, TEMP_REG_2.w2, 2
	add 	TEMP_REG_2.w2, TEMP_REG_2.w2, 0x2
	qbge	compare_data, TEMP_REG_4.w0, TEMP_REG_4.w2	; Jump if OUT >= IN
	mov 	TEMP_REG_3.w2, TEMP_REG_4.w0
compare_data:
	qbne	data_set, TEMP_REG_3.w2, 0
	mov 	TEMP_REG_3.w2, TEMP_REG_4.w2
data_set:
	qbge	default_cycle, TEMP_REG_3.w2, 4			; Default Cylce time - 1us if IN Data is 4 >= IN/OUT_Data
; 	if not then calculate the required cycle time
;	add 125 ns for each additional Cycle Time
;	Calculate the Additional Required Time - Data - 4 => additional
	sub 	TEMP_REG_3.w2, TEMP_REG_3.w2, 4
data_cycle:
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, 0x7D		; 0x7D - 125 ns
	sub 	TEMP_REG_3.w2, TEMP_REG_3.w2, 1
	qbne	data_cycle, TEMP_REG_3.w2, 0
default_cycle:
	add TEMP_REG_1.w0, TEMP_REG_1.w0, TEMP_REG_3.w0
	sub TEMP_REG_2.w0, TEMP_REG_2.w0, 1
	qbne calc_cycle_time, TEMP_REG_2.w0, 0
	sub TEMP_REG_1.w0, TEMP_REG_1.w0, 5			; Save the Cycle Time

	; Save the Calculated Cycle Time in Device Data Memory
	ldi TEMP_REG_2.w0, SORTE_REG_INTERFACE
	add TEMP_REG_2.w0, TEMP_REG_2.w0, PARAM_DATA_OFFSET
	sbco &TEMP_REG_1, ICSS_DMEM0_CONST, TEMP_REG_2.w0, 4

	; Clear the used Registers to avoid any miscalculations
	ldi32	TEMP_REG_1, 0
	ldi32	TEMP_REG_2, 0
	ldi32	TEMP_REG_3, 0
	ldi32	TEMP_REG_4, 0

;v0.2 added - end									 
; ------------------------------------------------------------------------------
; state: PARAM
; Regsiter defines:
; n = TEMP_REG_4.b0 (iterations)
; m = TEMP_REG_4.b1 (counter for 32 bytes blocks in payload field)
; previous cycle counter = TEMP_REG_3
; ------------------------------------------------------------------------------

STATE_PARAM:

; stay in same host send mode as DISCOVERY state

; set protocol state
   	M_SET_STATE S_PARAM, TEMP_REG_1.b1

; set number of iterations
	ldi 	TEMP_REG_4.b0, PARAM_REPEAT

STATE_PARAM_ITERATION:
	;v0.2 - start

; Load the Cycle Time from Data memory
	ldi TEMP_REG_2.w0, SORTE_REG_INTERFACE
	add TEMP_REG_2.w0, TEMP_REG_2.w0, PARAM_DATA_OFFSET
	lbco 	&TEMP_REG_1, ICSS_DMEM0_CONST, TEMP_REG_2.w0, 4
; Load the Total IN Data Offset from Memory
	ldi 	TEMP_REG_3.w0, DISCOV_RX_OFFSET
	add		TEMP_REG_3.w0, TEMP_REG_3.w0, 0xE
	lbco	&TEMP_REG_6.w0, ICSS_DMEM0_CONST, TEMP_REG_3.w0, 2
; Load the Total OUT Data Offset from Memory
	add		TEMP_REG_3.w0, TEMP_REG_3.w0, 0x2
	lbco	&TEMP_REG_6.w2, ICSS_DMEM0_CONST, TEMP_REG_3.w0, 2
; v0.2 - end

; setup PARAM packet using struct
    ldi   	PARAM_P.da, 0xff
    ldi		PARAM_P.sa, 0
    ldi		PARAM_P.type, 0x02
    ldi		PARAM_P.status, 0x10
    qbne    STATE_PARAM_CONT,TEMP_REG_4.b0, 1
    ldi		PARAM_P.status, 0x18
STATE_PARAM_CONT:
    mov		PARAM_P.cycle_time,     TEMP_REG_1	;CYCLE_TIME_10us
    ldi32	PARAM_P.sync_cycle, 	SYNC_CYCLE_NONE
    ldi		PARAM_P.delay_period, 	DELAY_PERIOD_1s
    ldi 	PARAM_P.delay_burst, 	DELAY_BURST_CNT_16
	ldi 	PARAM_P.delay_gap, 		DELAY_GAP_TIME_1ms
	ldi		PARAM_P.delay_fwd_time,	DELAY_FWD_TIME_320ns
	ldi		PARAM_P.ipg,			IPG_260ns
	ldi		PARAM_P.t_in,			T_IN_0
	ldi		PARAM_P.t_out,			T_OUT_0
	ldi		PARAM_P.topology,		TOPOLOGY_NORMAL
	ldi		PARAM_P.diagnostics,	DIAGS_MODE_PAYLOAD
	ldi		PARAM_P.alarm,			ALARM_MODE_NO
	ldi		PARAM_P.crc_mode,		CRC32 | (CRC32<<4)
	mov 	PARAM_P.total_in_data, 	TEMP_REG_6.w0
	mov 	PARAM_P.total_out_data,	TEMP_REG_6.w2
; load the Device Count
	ldi     TEMP_REG_1.w0, DISCOV_RX_OFFSET
	add 	TEMP_REG_1.w0, TEMP_REG_1.w0, 0x04 ;  Device Count
	lbco    &TEMP_REG_6.w0, ICSS_DMEM0_CONST, TEMP_REG_1.w0, 2

	mov		PARAM_P.device_count,	TEMP_REG_6.w0
; reset RX- and TX-FIFO and clear TX errors
	ldi		R31.w2, (D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RESET_TXFIFO)

; start 1 ms timer using PRU cycle counter
; disable cycle counter to clear it
; bit 3 is cycle counter enable, need to keep PRU enable
    ldi		TEMP_REG_1.w0, 0x0b03
    sbbo    &TEMP_REG_1.b0, r26, 0, 1
    zero    &TEMP_REG_2, 4

; clear cycle counter
    sbbo    &TEMP_REG_2, r26, 0x0c, 4

; enable cycle counter
    sbbo    &TEMP_REG_1.b1, r26, 0, 1

; clear previous cycle counter
	zero    &TEMP_REG_3, 4

; send PARAM packet

; pre-amble
; todo: optimize for short pre-amble here

; set parameter for send loop
; R1.b0 points to start of structure
; R1.b1 used as loop counter
	ldi		R1.b1, $sizeof(PARAM_P)
    ldi		R1.b0, &R2.b0

;send PARAM packet (original size is 44, use 60 for trace with wireshark in debug phase)

    xout    TXL2_XID, &r2, 60
    nop
    nop
; send eof and generate crc
    ;or      R31.b3, R31.b3, 0x2c
    M_CMD16	(D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD | D_TX_EOF)

; receive packet and check if all accepted
STATE_PARAM_WAIT_BANK0:
	xin		RXL2_SIDEA, &R18, 1
    qbgt	STATE_PARAM_WAIT_BANK0, r18.b0, 32

    xin		RXL2_SIDEA, &r2, 32+16

; received packet has PA?
; find start offset
	ldi		R1.b0, &R2.b0
    .if $defined(FULL_PREAMBLE)
STATE_PARAM_PA_SEARCH:
;
; check on error: r1.b0 >8 (R2 offset) + 8 (max PA size)
    qblt	STATE_PARAM_PA_CONT, r1.b0, 16
    mvib	TEMP_REG_1.b1, *R1.b0++
    qbne	STATE_PARAM_PA_SEARCH, TEMP_REG_1.b1, 0xD5
STATE_PARAM_PA_CONT:
    .endif

; check on status byte ok
; add 3 bytes offset to get to status field
    add		R1.b0, R1.b0, 3
    mvib	TEMP_REG_1.b1, *R1.b0
;Check for Status bit of retrain, If yes jump to discovery
    qbbs	CONFIGURE_SORTE, TEMP_REG_1.b1, 6
    qbbs	STATE_PARAM_RCV_OK, TEMP_REG_1.b1, 0
;   not all slave accepted

STATE_PARAM_RCV_OK:

;  todo: read and verify rest of packet
;  param was 64 byte packet including CRC, wait for 2nd bank
STATE_PARAM_WAIT_BANK1:
;	xin		RXL2_SIDEB, &R18, 1
;   qblt	STATE_PARAM_WAIT_BANK1, r18.b0, 32

; wait for eof r31 (bit 20)
    qbbc    STATE_PARAM_WAIT_BANK1, r31, 20

;  wait for end of 1 ms period
    ldi32     r1, 250000
STATE_PARAM_WAIT:
    sub     r1,r1,1
    qbne    STATE_PARAM_WAIT, r1, 0

;reset RX (Bit 18) and clear EOF(Bit 22), rx_error_clr (23)
    or      r31.b2, r31.b2, 0xF4

	sub		TEMP_REG_4.b0,TEMP_REG_4.b0,1
	qbne    STATE_PARAM_ITERATION,TEMP_REG_4.b0, 0
	

; ------------------------------------------------------------------------------
; state: DELAY_MASTER
; Regsiter defines:
; n = TEMP_REG_4.w0 (iterations)
;
; min delay = TEMP_REG_6.w0
; max delay = TEMP_REG_6.w2
; avg delay = TEMP_REG_5.w2
;
; ------------------------------------------------------------------------------
STATE_DELAY:

; set protocol state
   	M_SET_STATE S_LINED, TEMP_REG_1.b1

; set const to control PRU cycle counter
    ldi32 	r27, ICSS_PRU0_CNTL_BASE

; reset iep timer
	ldi     TEMP_REG_1.b0, CNT_DISABLE
	sbco    &TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG, 1

	ldi32   TEMP_REG_1, 0
	sbco  	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
; for 64 bit iep
	.if $isdefed("ICSS_REV2")
    sbco      &TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG + 4, 4
    .endif

	ldi     TEMP_REG_1.b0, 0x41
	sbco    &TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG, 1

; init min, max and avg delay values
    ldi32	TEMP_REG_6,0x00007fff
    ldi		TEMP_REG_5.w2, 0
	ldi		TEMP_REG_4.w0, DELAY_BURST_CNT_16

STATE_DELAYM:
; dummy read of capture registers
	lbco	&r2,ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 12*4
; reset RX- and TX-FIFO and clear RX errors
	ldi		R31.w2, (D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RESET_TXFIFO)

; reset cycle counter
; start 1 ms timer using PRU cycle counter
; disable cycle counter to clear it
; bit 3 is cycle counter enable, need to keep PRU enable
    ldi		TEMP_REG_1.w0, 0x0b03
    sbbo    &TEMP_REG_1.b0, r27, 0, 1
    zero    &TEMP_REG_2, 4

; clear cycle counter
    sbbo    &TEMP_REG_2, r27, 0x0c,4

; enable cycle counter
    sbbo    &TEMP_REG_1.b1, r27, 0, 1

; clear previous cycle counter
	zero    &TEMP_REG_3, 4

; send DELAY_REQ packet
; start with delay master mode
;	M_SET_DELAY_M_P0

; pre-amble, for gigabit assume full pre-amble right now
; send 8 byte PA
	.if 0 ; FULL_PREAMBLE
	ldi		TX_DATA_WORD, 0x5555
	NOP
	M_PUSH_WORD
	ldi		TX_DATA_WORD, 0x5555
	NOP
	M_PUSH_WORD
;	.endif

; send 4 byte pre-amble
	ldi		TX_DATA_WORD, 0x5555
	NOP
	M_PUSH_WORD
	ldi		TX_DATA_WORD, 0xd555
	NOP
	M_PUSH_WORD
	.endif

; 4 bytes header
	ldi32 	r2,DELAY_REQ_P
	qbne	STATE_DELAY_LAST, TEMP_REG_4.b0, 1	; the frame counter runs down to 0
	ldi		r2.b3, 0x08
STATE_DELAY_LAST:
    xout    TXL2_XID, &r2, 60
    nop
    nop
; send eof and generate crc
    ;or      R31.b3, R31.b3, 0x2c
    M_CMD16	(D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD | D_TX_EOF)

; receive DELAY_RESP packet
STATE_DELAY_WAIT_BANK0:
	xin		RXL2_SIDEA, &R18, 1
    qbgt	STATE_DELAY_WAIT_BANK0, r18.b0, 4

; read rx and tx time stamp - sof based
; read all time stamp register in one step r2 maps to cap0 (RX_SOF), r6 maps to cap
	lbco	&r2,ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 12*4
    sub		TEMP_REG_1,r2,r10
	qbgt	STATE_DELAY_SKIP_MIN, TEMP_REG_6.w0, TEMP_REG_1.w0
    mov		TEMP_REG_6.w0, TEMP_REG_1.w0

STATE_DELAY_SKIP_MIN:
    qblt	STATE_DELAY_SKIP_MAX,TEMP_REG_6.w2,TEMP_REG_1.w0
    mov		TEMP_REG_6.w2, TEMP_REG_1.w0

STATE_DELAY_SKIP_MAX:
; calculate average, divide by count at the end
    add		TEMP_REG_5.w2, TEMP_REG_5.w2, TEMP_REG_1.w0

; wait gap (1ms)

STATE_DELAY_WAIT:
    ldi32	TEMP_REG_1, 0x0030d40
	lbbo 	&PRU_CC_REG, r27, 0x0c , 4
	qblt    STATE_DELAY_WAIT, TEMP_REG_1, PRU_CC_REG

; repeat burst (16 times)

	sub		TEMP_REG_4.w0,TEMP_REG_4.w0, 1
	qbne    STATE_DELAYM,TEMP_REG_4.w0, 0

; shift right by 4 works for DELAY_BURST_CNT_16
    lsr		TEMP_REG_5.w2, TEMP_REG_5.w2, 4
; subtract bridge delay = 320 ns
    ldi		TEMP_REG_1.w0, DELAY_FWD_TIME_320ns
    sub     TEMP_REG_5.w2, TEMP_REG_5.w2, TEMP_REG_1.w0
; shift right by one to divide by 2 for line delay measurement
    lsr		TEMP_REG_5.w2, TEMP_REG_5.w2, 1

; ------------------------------------------------------------------------------
; state: SYNC
;
; Regsiter defines:
; n = TEMP_REG_4.w0 (iterations)
; period = TEMP_REG_3 (continous sync compare value used in cmp3)
; previous cycle counter = TEMP_REG_3
; --------------------------------------------------------------------------------

; send 1024 sync packets every 10 us using iep timer and cmp3 time of 500 ns for first packet
; sub-sequent packets have 10.0 us period
STATE_SYNC:
; set protocol state
   	M_SET_STATE S_SYNC, TEMP_REG_1.b1
; dbg EDIO
    ldi32   r26, 0x0002E300
    sbbo    &r29.b0, r26, 0x13, 1

; set time triggered send mode where PRU0 uses cmp3 register, bit 2 is TX_EN_MODE0
	ldi32	TEMP_REG_1, 0x07
	sbco 	&TEMP_REG_1, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0, 1
;	M_SET_MIIRT_TTS

; disable IEP counter, keep cnt increment to 4
	ldi   	TEMP_REG_1, 0x40
	sbco  	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG, 1

; clear global status
	ldi   	TEMP_REG_1, 0x1
	sbco  	&TEMP_REG_1, ICSS_IEP_CONST,ICSS_IEP_GLOBAL_STATUS_REG, 1

; reset IEP timer
	ldi32   TEMP_REG_1, 0xffffffff
	sbco  	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
    sbco    &TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG+4, 4

; set iteration to 1024 sync packets
	;ldi		TEMP_REG_4.w0, 1024
	ldi		TEMP_REG_4.w0, 32	; Debug: reduce number of packets for validation

; set 500 ns offset for first sync packet
	ldi32	TEMP_REG_3, 500
	sbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG, 4

; load the IEP CMP CFG register and enable the cmp3
;	lbco 	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1
;	ldi		TEMP_REG_1.b0, R2.b0, 0x10
;	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1
	lbco   &TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 4
    set    TEMP_REG_1, TEMP_REG_1, 4
    clr	   TEMP_REG_1, TEMP_REG_1, 0
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 4

; clear cmp3 register
	ldi		TEMP_REG_1.b0, 0x08
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

; enable IEP counter, keep cnt increment to 4
	ldi   	TEMP_REG_1, 0x41
	sbco  	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG, 1

STATE_SYNC_ITERATION:
; need to enable tx fifo for every packet in TX_EN_MODE
	ldi32	TEMP_REG_1, 0x07
	sbco 	&TEMP_REG_1, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0, 1
;	M_SET_MIIRT_TTS

; reset RX- and TX-FIFO and clear RX errors
	ldi		R31.w2, (D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RESET_TXFIFO)


; send 4 byte pre-amble
;	ldi		TX_DATA_WORD, 0x5555
;	NOP
;	M_PUSH_WORD
;	ldi		TX_DATA_WORD, 0xd555
;	NOP
;	M_PUSH_WORD

; 4 bytes header
	ldi32 	r2, SYNC_P
;	ldi		TEMP_REG_2.w0, 1023
	;qbne	STATE_SYNC_LAST, TEMP_REG_4.w0, TEMP_REG_2.w0
	qbne	STATE_SYNC_LAST, TEMP_REG_4.w0, 1	; frame counter counts down to zero
    ldi     r2.b3, 0x08
;	ldi		TEMP_REG_1.b3, 0x08

STATE_SYNC_LAST:

; insert time stamp of outgoing packet which is cmp3+ld+ 40ns TX L0 Fifo delay
	lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG, 4
	add		TEMP_REG_1, TEMP_REG_1, TEMP_REG_5.w2
	add		r3, TEMP_REG_1, 40 ;
	;add		TEMP_REG_1, TEMP_REG_1, 100
	;add		TEMP_REG_1, TEMP_REG_1, 180
    ;xout    TXL2_XID, &r2, 60    ; used to trace with wireshark in standalone test
    xout    TXL2_XID, &r2, 10
    nop
    nop
; send eof and generate crc
    ;ldi      R31.b3, 0x2c
    M_CMD16	(D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD | D_TX_EOF)

; wait ~5 us after packet was sent
	ldi		TEMP_REG_2, 5000
	add		TEMP_REG_1, TEMP_REG_1, TEMP_REG_2
STATE_SYNC_WAIT:
	lbco  	&TEMP_REG_2, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	qblt	STATE_SYNC_WAIT, TEMP_REG_1,TEMP_REG_2

; calculate TTS to TS offset
	lbco	&TEMP_REG_1.w0, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG, 2
	lbco	&TEMP_REG_1.w2, ICSS_IEP_CONST, ICSS_IEP_CAPR4_REG, 2
	sub		R0.w2, TEMP_REG_1.w2, TEMP_REG_1.w0
	;sbco	&R0.w2, ICSS_SHARED_RAM_CONST, TTS_TO_TS_DELAY, 2

; add 10 us to cmp3
	ldi		TEMP_REG_1, 10000
	add	    TEMP_REG_3, TEMP_REG_3, TEMP_REG_1.w0
	sbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG, 4
	sbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CMP0_REG, 4

; clear cmp3 register
	ldi		TEMP_REG_1.b0, 0x08
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

	sub		TEMP_REG_4.w0,TEMP_REG_4.w0, 1
	qbne    STATE_SYNC_ITERATION,TEMP_REG_4.w0, 0
; last cycle has added 10us to the inital 500ns offset, correcting the cycle time here
	ldi		TEMP_REG_1.w0, 500
	sub		TEMP_REG_3, TEMP_REG_3, TEMP_REG_1.w0
;	sbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG, 4
	sbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CMP0_REG, 4

; ------------------------------------------------------------------------------
; state: IO Exchange
; Regsiter defines:
; n = TEMP_REG_4.b0 (iterations)
; m = TEMP_REG_4.b1 (counter for 32 bytes blocks in payload field)
; previous cycle counter = TEMP_REG_3
; IO_CYCLE_CNT = cycle counter which start from 0, overlapps with PRU_CC
; ------------------------------------------------------------------------------
STATE_IOEXHANGE:

LED_SET_STATE_GREEN:
	ldi32	TEMP_REG_1, 0x00601040
	ldi		TEMP_REG_2, 0x80
	sbbo	&TEMP_REG_2, TEMP_REG_1, 0, 4
	ldi		TEMP_REG_2, 0x40
	sbbo	&TEMP_REG_2, TEMP_REG_1, 4, 4

; Enable IEP counter for SYNC testing
	ldi     TEMP_REG_1.b0, 0x41
	sbco    &TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_GLOBAL_CFG_REG, 1
; dbg R28.w0 is write pointer for debug data
	ldi		R28.w0, 0
	sbbo    &r29.b1, r26, 0x13, 1

; clear in ptr
    ldi     r27.w0, 0

; configure task manager for receive processing
 .if $defined(PRU0)
    ldi32   r2, 0x0002a000	; TM base
 .else
    ldi32   r2, 0x0002a200	; TM base
 .endif
    ldi     r3, 0x0041		; 0x39 Eth mode,T1_S1 = FB, T1_S2 = NB, T1_S3 = NB
    sbbo    &r3, r2,0, 2

; set receive tasks address
    ldi     r3.w0, $CODE(rxio_fb)
    sbbo    &r3.w0,r2,0x0c, 2
    ldi     r3.w0, $CODE(rxio_b2)
    sbbo    &r3.w0,r2,0x10, 2
    ldi     r3.w0, $CODE(rxio_nb)
    sbbo    &r3.w0,r2,0x14, 2
    ldi     r3.w0, $CODE(rxio_eof)
    sbbo    &r3.w0,r2,0x18, 2
	
; set receive task trigger same on FB1, FB2, NB = 32

    ldi     r3, 0x7fea
    sbbo    &r3,r2, 0x30, 2
; .if $defined(rxio_tm)
; enable task manager instruction
    .word 0x32800000
; .endif
 
; start IO cycle count to 0
; IO cycle count is used for sample application
; 32 ms = 4 us * 8192 (2^13)
; 64 ms = 4 us * 16384 (2^14)
; 128 ms = 4us * 32768 (2^15)
; 256 ms = 4us * 65536 (2^16)
	ldi   IO_CYCLE_CNT, 0    ; r20

; set protocol state
   	M_SET_STATE S_IOEXCHANGE, TEMP_REG_1.b1
; Load the Cycle time from memory
	ldi TEMP_REG_1.w2, SORTE_REG_INTERFACE
	add TEMP_REG_1.w2, TEMP_REG_1.w2, PARAM_DATA_OFFSET
	lbco &TEMP_REG_1.w0, ICSS_DMEM0_CONST, TEMP_REG_1.w2, 2

	
; Set SYNC0 pulse width: e.g. 500 ns (125 * 4)
    ldi     TEMP_REG_1.w2, 125
	ldi     TEMP_REG_2.w0, ICSS_IEP_SYNC_PWIDTH_REG
	sbco    &TEMP_REG_1.w2, ICSS_IEP_CONST, TEMP_REG_2.w0, 2

; Set SYNC0 start time: e.g. 500 ns
    ldi		TEMP_REG_1, 500	; set SYNC0 to the same value as TX_EN for TTS
    sbco   	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG, 4

;	lsr		TEMP_REG_1.w0, TEMP_REG_1.w0, 1		; 1/8th of cycle time for TTS
;   sbco   	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP2_REG, 4

; set CMP0 wrap around mode
; load the IEP CMP CFG register and enable the cmp0 with wrap around mode
    lbco   	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 4
    set		TEMP_REG_1, TEMP_REG_1, 1+1 	;CMP[1]
    set		TEMP_REG_1, TEMP_REG_1, 1+0 	;CMP[0]
    set		TEMP_REG_1, TEMP_REG_1, 0		; CMP0_RST_CNT_EN
    set		TEMP_REG_1, TEMP_REG_1, 1+3		; CMP[3]
 ;   set		TEMP_REG_1, TEMP_REG_1, 1+2		; CMP[2]
    sbco   	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 4
    
; wait for CMP0 hit and loop back to new cycle
STATE_IO_WAIT0:
	lbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
	qbbc	STATE_IO_WAIT0,TEMP_REG_1.b0,0

; dbg EDIO
    sbbo    &r29.b0, r26, 0x13, 1
;    clear cmp0, cmp1, cmp2 and cmp3 events
	ldi		TEMP_REG_1.b0, 0x0f
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

; re-enable SYNC0
; clear and set sync_en bit in IEP timer to re-enable trigger
    ldi    TEMP_REG_1, 0x06    ; clear sync_en (sync0_en, sync1_en)
    ldi    TEMP_REG_2.w0, ICSS_IEP_SYNC_CTRL_REG
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4
    ldi    TEMP_REG_1, 0x07    ;  set sync_en (sync0_en, sync1_en)
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4

; dbg: read time stamp and send to SYNC1	
;	lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CAPR4_REG, 4
;	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP2_REG, 4

; Load the Calculated Cycle Time in Device Data Memory
	ldi     TEMP_REG_2, SORTE_REG_INTERFACE
	add     TEMP_REG_2, TEMP_REG_2, PARAM_DATA_OFFSET
	lbco    &TEMP_REG_1, ICSS_DMEM0_CONST, TEMP_REG_2, 4
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP0_REG, 4
	lsr 	TEMP_REG_1, TEMP_REG_1, 2	; 1/4th of Cycle Time
	ldi		TEMP_REG_2, 500
	qbge	MAX_MASTER_TTS, TEMP_REG_1, TEMP_REG_2
; Maximum value of TTS set to 500ns for Deterministic Behaviour
	ldi		TEMP_REG_1, 500
MAX_MASTER_TTS:

; set time offset for the out packet
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG, 4

; TEMP_REG_6.w0 gets number of devices to detect point to point connection
	ldi     TEMP_REG_1.w0, DISCOV_RX_OFFSET
	add 	TEMP_REG_1.w0, TEMP_REG_1.w0, 0x4			; Device Count with cut pre-amble
	lbco    &TEMP_REG_6.w0, ICSS_DMEM0_CONST, TEMP_REG_1.w0, 2

STATE_IO_REPEAT:
	;set last TX_TS as new SYNC0 pulse
	;lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CAPR4_REG, 4
	;sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG, 4
; set time triggered send mode where PRU0 uses cmp3 register, bit 2 is TX_EN_MODE0
	ldi32	TEMP_REG_1, 0x07
	sbco 	&TEMP_REG_1, ICSS_MII_RT_CONST, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0, 1
; reset RX- and TX-FIFO and clear TX errors
	ldi		R31.w2, (D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RESET_TXFIFO)
; load the total OUT Data for Device from Memory
	ldi 	TEMP_REG_3.w0, DISCOV_RX_OFFSET
	add		TEMP_REG_3.w0, TEMP_REG_3.w0, 0x08
	lbco	&r0.w0, ICSS_DMEM0_CONST, TEMP_REG_3.w0, 2

    ldi		r2.w0, 0x3000
    ldi		TEMP_REG_1.w0, OUT_DATA_OFFSET
; check whether there is less than 58 byte data	
    qblt    IOEX_multi_block, r0.w0, 58
; load data	
    lbco	&r2.b2, ICSS_DMEM0_CONST, TEMP_REG_1.w0, b0 
; 2 bytes header SA = 0 and Sataus = IO Exchange master state
	ldi 	r2.w0, 0x3000
	add     r0.b0, r0.b0, 2
; send header and out data field 
;    xout    TXL2_XID, &r2, b0
    xout    TXL2_XID, &r2, 60

; dbg: toggle EDIO
    sbbo    &r29.b1, r26, 0x13, 1
; send eof and generate crc32
    M_CMD16	(D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD | D_TX_EOF)
    qba     SKIP_IOEX_MULTI

IOEX_multi_block:	
; 2 bytes header SA = 0 and Sataus = IO Exchange master state
	ldi 	r2.w0, 0x3000
; r0.w0 has OUT data length which is > 58
; work with 32 byte blocks
    lbco	&r2.b2, ICSS_DMEM0_CONST, TEMP_REG_1.w0, 30
    sub     r0.w0, r0.w0, 30
	add     TEMP_REG_1.w0,TEMP_REG_1.w0, 30
; send header and out data field 
    xout    TXL2_XID, &r2, 32
IOEX_multi_block_next:   
    qbge	IOEX_send_last_block, r0.w0, 32
    lbco	&r2.b0, ICSS_DMEM0_CONST, TEMP_REG_1.w0, 32
IOEX_check_tx_l2_fifo:
	xin		TXL2_XID, &r19, 4
	qbne	IOEX_check_tx_l2_fifo, r19.b2, 0 	
	xout    TXL2_XID, &r2, 32
    sub     r0.w0, r0.w0, 32
	add     TEMP_REG_1.w0,TEMP_REG_1.w0, 32
	; is remaining length >=32	
	qblt    IOEX_multi_block_next, r0.w0, 32
IOEX_send_last_block:
; send last block
; no need to check fifo level as we work on 32 byte blocks
    lbco	&r2.b2, ICSS_DMEM0_CONST, TEMP_REG_1.w0, b0
	xout    TXL2_XID, &r2, b0
; send eof and generate crc32
    M_CMD16	(D_PUSH_CRC_MSWORD_CMD | D_PUSH_CRC_LSWORD_CMD | D_TX_EOF)

	
SKIP_IOEX_MULTI:	
	
   .if $defined("application_io")

; now is time to generate new output data
; running LEDs for slave 1..4 with 2^n rate difference
; takes 31 cycles = 155 ns

; Load the Device Count from the memory and change the application data as per number of devices
; in the network
	ldi     TEMP_REG_1.w0, DISCOV_RX_OFFSET
	add 	TEMP_REG_1.w0, TEMP_REG_1.w0, 0x04			; Device Count
	lbco    &TEMP_REG_3.w0, ICSS_DMEM0_CONST, TEMP_REG_1.w0, 2
	ldi 	TEMP_REG_3.w2, 18
; The device timings are generated based on the cycle time

; slave 1 = 32 ms runnig LED
; slave n = n us * 2^(20-n) running LED
	sub 	TEMP_REG_3.w2, TEMP_REG_3.w2, TEMP_REG_3.w0
	lsr     TEMP_REG_1, IO_CYCLE_CNT, TEMP_REG_3.w2
	and		TEMP_REG_1.b0,TEMP_REG_1, 0x07
	ldi		TEMP_REG_1.b1, 1
	lsl		TEMP_REG_1.b0, TEMP_REG_1.b1, TEMP_REG_1.b0
    ldi		TEMP_REG_2.w0, OUT_DATA_OFFSET
    sbco	&TEMP_REG_1.b0, ICSS_DMEM0_CONST, TEMP_REG_2.w0, 1

    ; show all devices with the same LED pattern
    mov		TEMP_REG_4.b0, TEMP_REG_1.b0	; save LED pattern of first device

    sub 	TEMP_REG_3.w0, TEMP_REG_3.w0, 1
device_timings:
; slave n+1 = n us * 2^(20-n) runnig LED
	qbeq 	skip_device_timings, TEMP_REG_3.w0, 0
	add		TEMP_REG_3.w2, TEMP_REG_3.w2, 1
	lsr     TEMP_REG_1, IO_CYCLE_CNT, TEMP_REG_3.w2
	and		TEMP_REG_1.b0,TEMP_REG_1, 0x07
	ldi		TEMP_REG_1.b1, 1
	lsl		TEMP_REG_1.b0, TEMP_REG_1.b1, TEMP_REG_1.b0
;    ldi		TEMP_REG_2.w0, OUT_DATA_OFFSET
    add		TEMP_REG_2.w0, TEMP_REG_2.w0, 12

    ; show all devices with the same LED pattern
    mov		TEMP_REG_1.b0, TEMP_REG_4.b0	; restore LED pattern of the first devices


    sbco	&TEMP_REG_1.b0, ICSS_DMEM0_CONST, TEMP_REG_2.w0, 1
    sub 	TEMP_REG_3.w0, TEMP_REG_3.w0, 1
    qbne 	device_timings, TEMP_REG_3.w0, 0
skip_device_timings:
    .endif

; increment cycle count and store to interface register
	add	   	IO_CYCLE_CNT, IO_CYCLE_CNT, 1
	qbeq	SKIP_IO_DBG2, TEMP_REG_6.w0, 1
    sbco    &IO_CYCLE_CNT, ICSS_DMEM0_CONST, CYCLE_CNT_P0_OFFSET, 4
;; todo?
    sbco    &R22,ICSS_DMEM0_CONST, 0x60, 32

	lbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
; STATE_IO_WAIT_BLOCK

SKIP_IO_DBG2:

	qbeq	STATE_IO_WAIT, TEMP_REG_6.w0, 1
    ; check on missing packet, bit 6 and 7 define retrain mode if packet is not received
    ;lbco    &r2, ICSS_DMEM0_CONST, CTRL_REG_OFFSET , 1
    ;and 	r2.b0, r2.b0, 0xc0
    ;qbeq	STATE_IOEX_SKIP_RETRAIN,r2.b0,0
	; Check for retrain
	; qbne	STATE_IOEX_ACTIVE, r4.b1, 0x40	; Check for retrain bit
	;jmp 	CONFIGURE_SORTE
	; TM: get the TX timestamp of the master frame for port 0

STATE_IOEX_ACTIVE:
; dbg
	lbco	&TEMP_REG_1.w0, ICSS_IEP_CONST, ICSS_IEP_CAPR4_REG, 2
	ldi 	TEMP_REG_2, SORTE_REG_INTERFACE
	add     TEMP_REG_2, TEMP_REG_2, PARAM_DATA_OFFSET
	lbco    &TEMP_REG_1.w2, ICSS_DMEM0_CONST, TEMP_REG_2, 2
	lsr		TEMP_REG_1.w2, TEMP_REG_1.w2, 3

	;ldi		TEMP_REG_1.w2, 500	; this is the desired TX timestamp
	sub		TEMP_REG_1.w0, TEMP_REG_1.w0, TEMP_REG_1.w2
	sbco	&TEMP_REG_1.w0, c31, R28.w0, 2
	add		R28.w0, R28.w0, 2

; receive diagnostic code 
; read all receive time stamps and store min, max and avg. 
; for e.g. 32 device we need range of 32 * 12 bytes = 384 bytes
; use IORX_TS_DIAG offset into pru dmem0 (4k offset)
; structure: min / avg / max , each is 32 bit , repeats for 32 device starting from 1..32
; array needs to be initalized: min = 0xffffffff (4.2 s), 0  , 0 

;   num devices is in TEMP_REG_6.w0 
;   time stamp is stored in in_data at the end of packet
;   todo: dynamic in data size per device  	

    ldi     TEMP_REG_1.w2, 16  					; 12 bytes per device + 4 bytes time stamp
    ldi     r10.w0, IN_DATA_OFFSET  			; init start address
    sub     r10.w0, r10.w0, 4           		; point to time stamp of current packet
	ldi     r10.w2, IORX_TS_DIAG				; init start address for diag field

    loop    IODIAG_END, 16
	add     r10.w0, r10.w0, TEMP_REG_1.w2		; point to next packet
    lbco    &r5, ICSS_DMEM0_CONST, r10.w0, 4	; read time stamps

; calculate min / avg / max 	
    lbco    &r2, ICSS_DMEM0_CONST, r10.w2, 12   ; load TS entry 

	qbgt	IODIAG_SKIP_MIN, r2, r5
    mov		r2, r5
IODIAG_SKIP_MIN:
    qblt	IODIAG_SKIP_MAX,r4, r5 
    mov		r4, r5
IODIAG_SKIP_MAX:	
    qbeq    IODIAG_INIT_AVG, r3, 0
	mov     r3, r5
IODIAG_INIT_AVG:
    add     r3,r3,r5
	lsr     r3,r3, 1

; store to diagnostic field
    sbco    &r2, ICSS_DMEM0_CONST, r10.w2, 12
	add     r10.w2, r10.w2, 12	
	
IODIAG_END:
	
	
; wait for CMP0 hit and loop back to new cycle
STATE_IO_WAIT:
	lbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
	qbbc	STATE_IO_WAIT,TEMP_REG_1.b0,0
    sbbo    &r29.b0, r26, 0x13, 1
; clear in ptr
    ldi     r27.w0, 0

; clear cmp0, cmp1 and cmp3 events
	ldi		TEMP_REG_1.b0, 0x1f
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

; re-enable SYNC0
; clear and set sync_en bit in IEP timer to re-enable trigger
; clear sync_en (sync0_en, sync1_en)
    ldi    TEMP_REG_1, 0x06    
    ldi    TEMP_REG_2.w0, ICSS_IEP_SYNC_CTRL_REG
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 2

;  set sync_en (sync0_en, sync1_en)
    ldi    TEMP_REG_1, 0x07    
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 2

    qba		STATE_IO_REPEAT


firmware_version:
	.if $defined(PRU0)
	.word 0x00		; PRU0
	.else
	.word 0x01		; PRU1
	.endif

	.word   ICSS_FIRMWARE_RELEASE_1
	.word   ICSS_FIRMWARE_RELEASE_2



CONFIGURE_SORTE:
    halt

	
wait_1ms:
    sub       r1, r1, 1
    qbne      wait_1ms, r1, 0

    qba       wait_1ms

 .endif
