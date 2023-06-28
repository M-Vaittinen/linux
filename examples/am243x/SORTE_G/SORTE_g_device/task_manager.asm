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
;     file:   task_manager.asm
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

; includes from \icss-g-industrial-fw\pru_fw_common
	;.include "icss_bsram_macros.h"
    ;.include "pru_constant_defines.h"
    ;.include "icss_xfer_defines.h"
    ;.include "icss_tm_macros.h"
    ;.include "pru_macros.h"

    .include "icss_regs.inc"
    .include "sorte_g_pru_device_register.inc"
    .include "sorte_g_state_define.inc"
    .include "sorte_g_host_interface.inc"
    .include "macros.inc"
    .include "sorte_g_device_cfg.inc"

  	.global		TSK_IDLE_RX_BK1
  	.global		TSK_IDLE_RX_BKN
  	.global		TSK_IDLE_RX_EOF

	.global 	TSK_DISCOVERY_BK1
	.global		TSK_DISCOVERY_BKN
	.global		TSK_DISCOVERY_BKN2
	.global		TSK_DISCOVERY_EOF

	.global 	TSK_PARAM_BK1
	.global		TSK_PARAM_BK2
	.global		TSK_PARAM_EOF

	.global		TSK_LINE_DELAY_EOF

	.global 	TSK_SYNC_STATE_FWD_SOF
	.global		TSK_SYNC_STATE_REV_EOF

	.global		TSK_IOEX_STATE_FWD_SOF
	.global 	TSK_IOEX_STATE_FWD_BKN
	.global		TSK_IOEX_STATE_FWD_EOF
	.global		TSK_IOEX_STATE_CMP0_EVENT
	.global		TSK_IOEX_STATE_TTS_COMPLETE_EVENT

  	.global		FN_TASK_MANAGER_SETUP


; check TX FIFO
M_WAIT_FOR_TX_EOF .macro temp1_8
WAIT_TXFIFO?:
	.if $defined(PRU0)
	qbbs	WAIT_TXFIFO_LASTDEVICE?, DEVICE_STATUS, LAST_DEVICE_FLAG
	lbco	&temp1_8, ICSS_MII_RT_CONST, ICSS_MIIRT_TX_FIFO_LEVEL1, 1
	qbne	WAIT_TXFIFO?, temp1_8, 0
	qba		WAIT_TXFIFO_DONE?
WAIT_TXFIFO_LASTDEVICE?:
	lbco	&temp1_8, ICSS_MII_RT_CONST, ICSS_MIIRT_TX_FIFO_LEVEL1, 1
	qbne	WAIT_TXFIFO?, temp1_8, 0
	.else
	qbbs	WAIT_TXFIFO_LASTDEVICE?, DEVICE_STATUS, LAST_DEVICE_FLAG
	lbco	&temp1_8, ICSS_MII_RT_CONST, ICSS_MIIRT_TX_FIFO_LEVEL0, 1
	qbne	WAIT_TXFIFO?, temp1_8, 0
	qba		WAIT_TXFIFO_DONE?
WAIT_TXFIFO_LASTDEVICE?:
	lbco	&temp1_8, ICSS_MII_RT_CONST, ICSS_MIIRT_TX_FIFO_LEVEL1, 1
	qbne	WAIT_TXFIFO?, temp1_8, 0
	.endif
WAIT_TXFIFO_DONE?:
	.endm


; setup task manager register configuration based on new SORTE state
; setup also port confiugratin
FN_TASK_MANAGER_SETUP:
	tsen 0		; disable task manager

	.if $defined(PRU0)
    ldi32   TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_BASE	; TM base
	.else
    ldi32   TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_BASE	; TM base
	.endif
	; DevNote: clear all existing tasks - maybe this is not needed if we only enable the task that we have configured

	qbeq	TMG_IDLE, PROTOCOL_STATE_CNT, IDLE_STATE
	qbeq	TMG_DISCOVERY, PROTOCOL_STATE_CNT, DISC_STATE
	qbeq	TMG_PARAM, PROTOCOL_STATE_CNT, PARAM_STATE
	qbeq	TMG_LINED, PROTOCOL_STATE_CNT, LINED_STATE
	qbeq	TMG_SYNC, PROTOCOL_STATE_CNT, SYNC_STATE
	qbeq	TMG_IOEX, PROTOCOL_STATE_CNT, IOEX_STATE
	; unknown state
	halt

; *****************
; Idle State
; *****************
TMG_IDLE:
	; setup task manager for idle state
	; enable S0, S1, S2, S3
	ldi		TEMP_REG_3, (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S1_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S2_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S3_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4
	; map S0=RX_BK1, S1=BK2, S2=BKN and S3=RX_EOF
	ldi32	TEMP_REG_3, (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S0_MX_SHIFT) | (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S1_MX_SHIFT) | (3<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S2_MX_SHIFT) | (4<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S3_MX_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1, 4
	; set task entry pointers
	ldi     TEMP_REG_3.w0, $CODE(TSK_IDLE_RX_BK1)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S0, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_IDLE_RX_BKN)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S1, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_IDLE_RX_BKN)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S2, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_IDLE_RX_EOF)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S3, 2


	ldi		TEMP_REG_1.b0, 0x03 ; enable only CMP0 for 32-bit mode
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	; set TX L2 water mark level to 0
	;ldi		TEMP_REG_3, 0
	;sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_TX_CFG, 4
	; configure RGMII Ethernet ports
	; both Ports should automatically forward any frame in both directions w/o PRU interaction.
	;DevNote: Should both PRUs configure MII_RT port?
	M_DISABLE_TXL2
	M_SET_MIIRT_AF
	qbbc	TMG_DONE, DEVICE_STATUS, LAST_DEVICE_FLAG
	; last slave must enable loopback
; TM debug: AF_LB is debug only.
	;M_SET_MIIRT_AF_LB
	M_ENABLE_TXL2
	M_SET_MIIRT_HR_HSL2_LB
	qba TMG_DONE

; *****************
; Discovery State
; *****************
TMG_DISCOVERY:
	; wait that no TX is ongoing
	;qbbc	TMG_DISCOVERY, R31, D_TX_EOF_FLAG_BITNUM
	M_WAIT_FOR_TX_EOF TEMP_REG_3.b0

	.if $defined(remove)
	lbco	&TEMP_REG_3, ICSS_MII_RT_CONST, ICSS_MIIRT_TX_FIFO_LEVEL0, 8
	.if $defined(PRU0)
	qbbs	TMG_DISC_WAIT_TXFIFO_LASTDEVICE, DEVICE_STATUS, LAST_DEVICE_FLAG
	qbne	TMG_DISCOVERY, TEMP_REG_4.b0, 0
	qba		TMG_DISC_WAIT_TXFIFO_DONE
TMG_DISC_WAIT_TXFIFO_LASTDEVICE:
	qbne	TMG_DISCOVERY, TEMP_REG_3.b0, 0
	.else
	qbbs	TMG_DISC_WAIT_TXFIFO_LASTDEVICE, DEVICE_STATUS, LAST_DEVICE_FLAG
	qbne	TMG_DISCOVERY, TEMP_REG_3.b0, 0
	qba		TMG_DISC_WAIT_TXFIFO_DONE
TMG_DISC_WAIT_TXFIFO_LASTDEVICE:
	qbne	TMG_DISCOVERY, TEMP_REG_4.b0, 0
	.endif
TMG_DISC_WAIT_TXFIFO_DONE:
	.endif ;remove

	; wait for 100 usec
	ldi		TEMP_REG_3, 100000/4
	loop	TMG_DISC_DELAY_AFTER_EOF, TEMP_REG_3
	nop
TMG_DISC_DELAY_AFTER_EOF:
	; enable S0, S1, S2 and S3
	ldi		TEMP_REG_3, (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S1_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S2_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S3_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4
	; map S0=RX_BK1, S1=RX_BK2, S2=RX_BKN and S3=RX_EOF
	ldi32	TEMP_REG_3, (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S0_MX_SHIFT) | (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S1_MX_SHIFT) | (3<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S2_MX_SHIFT) | (4<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S3_MX_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1, 4
	; set task entry pointers
	ldi     TEMP_REG_3.w0, $CODE(TSK_DISCOVERY_BK1)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S0, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_DISCOVERY_BKN)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S1, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_DISCOVERY_BKN2)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S2, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_DISCOVERY_EOF)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S3, 2

	; All Ethernet data is received by RXL2, preamble has to be forwareded by HW to TX FIFO to support constant bridge delay.
	M_ENABLE_TXL2
	; Only the PRU on the return channel does configure the MII_RT ports - or the last device
	qbbs	TMG_DISC_LAST_DEVICE, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	; configure RGMII Ethernet ports
	M_SET_MIIRT_HR_HSBS ; forward port must be HR_HS
	;M_SET_MIIRT_AF_LOCAL ; revers port must be AF
TMG_DISC_LAST_DEVICE:
	qbbc	TMG_DONE, DEVICE_STATUS, LAST_DEVICE_FLAG
	M_SET_MIIRT_HR_HSL2_LB ; last slave must have HR_HS and enable loopback
	qba TMG_DONE

; **********************
; Parametrization State
; **********************
TMG_PARAM:
	; wait that no TX is ongoing
	;qbbc	TMG_PARAM, R31, D_TX_EOF_FLAG_BITNUM
	M_WAIT_FOR_TX_EOF TEMP_REG_3.b0
	; wait for 100 usec
	ldi		TEMP_REG_3, 100000/4
	loop	TMG_PARAM_DELAY_AFTER_EOF, TEMP_REG_3
	nop
TMG_PARAM_DELAY_AFTER_EOF:
	; enable S0, S1, S2
	ldi		TEMP_REG_3, (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S1_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S2_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4
	; map S0=RX_BK1, S1=RX_BK2, S2=RX_EOF
	ldi32	TEMP_REG_3, (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S0_MX_SHIFT) | (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S1_MX_SHIFT) | (4<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S2_MX_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1, 4
	; set task entry pointers
	ldi     TEMP_REG_3.w0, $CODE(TSK_PARAM_BK1)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S0, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_PARAM_BK2)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S1, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_PARAM_EOF)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S2, 2

	M_DISABLE_TXL2
	; Only the PRU on the return channel does configure the MII_RT ports - or the last device
	qbbs	TMG_PARAM_LAST_DEVICE, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	; All devices will auto-forward (inclusive last device) with L2 sniffing. When using MII_RT hardware auto-forwarding function it is required to disable TXL2
	M_SET_MIIRT_AF
	; non master port needs to get slave address into register
	lbco	&DEVICE_ADDR, ICSS_SHARED_RAM_CONST, DEVICE_ADDR_OFFSET, 1
TMG_PARAM_LAST_DEVICE:
	qbbc	TMG_DONE, DEVICE_STATUS, LAST_DEVICE_FLAG
	M_SET_MIIRT_AF_LB
	qba TMG_DONE

; *****************
; Line Delay State
; *****************
TMG_LINED:
	; wait that no TX is ongoing
	;qbbc	TMG_LINED, R31, D_TX_EOF_FLAG_BITNUM
	M_WAIT_FOR_TX_EOF TEMP_REG_3.b0

	ldi		TEMP_REG_1, 0
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	; wait for 100 usec
	ldi		TEMP_REG_3, 100000/4
	loop	TMG_LINED_DELAY_AFTER_EOF, TEMP_REG_3
	nop
TMG_LINED_DELAY_AFTER_EOF:

	; read out CAP events to re-enable them
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 4	; PRU0_RX_SOF
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CAPR5_REG, 4	; PORT1_TX_SOF
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CAPR2_REG, 4	; PRU1_RX_SOF
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CAPR4_REG, 4	; PORT0_TX_SOF

	; set PORT_DELAY measurment to 0
	ldi		TEMP_REG_3, 0
	sbco	&TEMP_REG_3, ICSS_SHARED_RAM_CONST, PORT_DELAY, 4
	; PRU_MASTER_PORT_FLAG (timing slave): waits for RX EOF trigger to issue line delay measurement to timing master
	qbbs	TMG_LINED_TIMING_SLAVE, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
;The timing slave (TS) is the PRU with PRU_MASTER_PORT_FLAG set, i.e. PRU in fwd direction
;The timing slave (TM) is the PRU with PRU_MASTER_PORT_FLAG cleard, i.e. PRU in reverse direction
;TMG_LINED_TIMING_MASTER:
	; disable all task
	ldi		TEMP_REG_3, (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4
	ldi		TEMP_REG_3, 0
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1, 4
	qba		TMG_LINED_CONFIGURE_PORTS

TMG_LINED_TIMING_SLAVE:
	; enable S0
	ldi		TEMP_REG_3, (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4
	; map S0=RX_EOF
	ldi32	TEMP_REG_3, (4<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S0_MX_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1, 4
	; set task entry pointers
	ldi     TEMP_REG_3.w0, $CODE(TSK_LINE_DELAY_EOF)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S0, 2
	;qba		TMG_LINED_CONFIGURE_PORTS

	;inialize min max values for line delay measurement
	fill	&R4, 4
	zero	&R5, 4
	ldi		R6,  LD_BUFFER
	sbco	&R4, ICSS_SHARED_RAM_CONST, LD_MIN, 12	; store LD_MAX + LD_BUFFER_PTR


TMG_LINED_CONFIGURE_PORTS:
	; Only the PRU on the return channel does configure the MII_RT ports
	qbbs	TMG_DONE, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	M_DISABLE_TXL2	; DevNote: TXL2 has to be disabled for AF_LB, hence non-master will use R30 register to transmit frame
	M_SET_MIIRT_AF_LB_REMOTE	; master port configured as AF+LB
	M_SET_MIIRT_HR_HSR30_LOCAL			; non-master port configured as HS
	qbbc	TMG_DONE, DEVICE_STATUS, LAST_DEVICE_FLAG
	M_SET_MIIRT_AF_LB
	qba TMG_DONE

; *****************
; Sync State
; *****************
TMG_SYNC:
	; read out CAP events to re-enable them
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 4	; PRU0_RX_SOF
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CAPR5_REG, 4	; PORT1_TX_SOF
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CAPR2_REG, 4	; PRU1_RX_SOF
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CAPR4_REG, 4	; PORT0_TX_SOF

	; wait that no TX is ongoing
	;qbbc	TMG_SYNC, R31, D_TX_EOF_FLAG_BITNUM
	M_WAIT_FOR_TX_EOF TEMP_REG_3.b0

	;ldi32	TEMP_REG_3, 0x1FC00
	;sbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_CAP_CFG_REG, 4

	; wait for 50 usec
	ldi		TEMP_REG_3, 50000/4
	loop	TMG_SYNC_DELAY_AFTER_EOF, TEMP_REG_3
	nop
TMG_SYNC_DELAY_AFTER_EOF:
	; enable S0
	ldi		TEMP_REG_3, (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4

	M_DISABLE_TXL2
	qbbs	TMG_SYNC_TIMING_FWD, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
;TMG_SYNC_TIMING_REV: ; reverse port
	; map S0=RX_EOF
	; ToDo: As the reverse port does not receive any frame, this code below can be removed
	;ldi32	TEMP_REG_3, (4<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S0_MX_SHIFT)
	;sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1, 4
	; set task entry pointers
	;ldi     TEMP_REG_3.w0, $CODE(TSK_SYNC_STATE_REV_EOF)
	M_SET_MIIRT_AF_LOCAL
	;sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S0, 2
	qba		TMG_DONE

TMG_SYNC_TIMING_FWD: ; forward port
	; map S0=RX_SOF
	ldi32	TEMP_REG_3, (0<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S0_MX_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1, 4
	; set task entry pointers
	ldi     TEMP_REG_3.w0, $CODE(TSK_SYNC_STATE_FWD_SOF)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S0, 2

	M_SET_MIIRT_HR_HSL1_LOCAL			; non-master port configured as HS

	qbbc	TMG_DONE, DEVICE_STATUS, LAST_DEVICE_FLAG
	; last slave terminate the frame: disable TX
	M_DISABLE_TX_LOCAL
	qba TMG_DONE

; *****************
; IO Exchange State
; *****************
TMG_IOEX:
	; PRU with PRU_MASTER_PORT_FLAG set need to send event to second PRU to submit transition to IOEX state.
	qbbc	TMG_IOEX_LAST_SLAVE, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	; configuration of last device! Second PRU needs to transition to IO EX state via event.
	; LAST_DEVICE_FLAG && PRU_MASTER_PORT_FLAG
	; set IEP event channel to reverse port PRU and PRU2PRU event to reverse port PRU (swap INTC channel 3/4 to host mapping)
	.if $defined(PRU0)
	ldi		TEMP_REG_1.w0, 0x0001
	.else
	ldi		TEMP_REG_1.w0, 0x0100
	.endif
	ldi		TEMP_REG_1.w2, ICSS_INTC_HMR0 + 3
	sbco	&TEMP_REG_1.w0, ICSS_INTC_CONST, TEMP_REG_1.w2, 2
	loop	TMG_IOEX_DELAY1, 100
	nop
TMG_IOEX_DELAY1:
	; send event to second PRU
	LDI		R31.b0, PRU2PRU_INTR1

	; last slave will not tansmit
	qbbs	TMG_IOEX_LAST_SLAVE, DEVICE_STATUS, LAST_DEVICE_FLAG
TMG_IOEX_WAIT_FOR_EOF:
	; wait that no TX is ongoing
	;qbbc	TMG_IOEX_WAIT_FOR_EOF, R31, D_TX_EOF_FLAG_BITNUM
	M_WAIT_FOR_TX_EOF TEMP_REG_3.b0

TMG_IOEX_LAST_SLAVE:
	; wait for 2000 nsec	; DevNote: Check if the timing needs to be adopbed for longer lines
	;ldi		TEMP_REG_3, 4000/4
	;loop	TMG_IOEX_AFTER_EOF, TEMP_REG_3
	;nop
;TMG_IOEX_AFTER_EOF:
	; disable TX mask:
	fill	&R30.w2, 2

	qbbs	TMG_IOEX_FROM_MASTER, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
TMG_IOEX_TO_MASTER:	; TX - device to master direction

    ldi    TEMP_REG_1, 0x00    ; clear sync_en (sync0_en, sync1_en)
    ldi    TEMP_REG_3.w0, ICSS_IEP_SYNC_CTRL_REG
    sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_3.w0, 4

	; reconfigure IEP CMP0 wrap-around
	;lbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, SYNC_DELAY, 4	; previous timestamp for expected SYNC frame
	;lbco	&TEMP_REG_3.w2, ICSS_SHARED_RAM_CONST, SYNC_DELAY_1ST, 2	; timestamp of first SYNC frame
	;ldi		TEMP_REG_3.w0, 10000-500		; SYNC distance is 10us. last SYNC_DELAY (before going to IOEX) - 1st SYNC_DELAY + 10000 = is the master's IEP wrap-around
	;add		TEMP_REG_1, TEMP_REG_1, TEMP_REG_3.w0
	;sub		TEMP_REG_1, TEMP_REG_1, TEMP_REG_3.w2
	lbco	&TEMP_REG_3, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 4
	ldi32	TEMP_REG_1, 32*10000
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP0_REG, 4
	; set CMP1 and CMP3/4
	fill	&TEMP_REG_1, 4
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG, 4
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG, 4
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP4_REG, 4
	ldi		TEMP_REG_1.b0, 0x37 ; enable wrap-around, CMP0, CMP1, CMP3, CMP4
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_CFG_REG, 1

	; two tasks, CMP0 (wrap-around) for TTS setup and TX EOF after TTS to enable frame auto-forward.
	; enable S0 and S1
	ldi		TEMP_REG_3, (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S1_SHIFT))
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4
	.if $defined(PRU0)
	; map S0=CMP0 S1=CMP4
	ldi32	TEMP_REG_3, ((16<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S0_MX_SHIFT) | (20<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S1_MX_SHIFT))
	.else
	; map S0=CMP0 S1=CMP3
	ldi32	TEMP_REG_3, ((16<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S0_MX_SHIFT) | (19<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S1_MX_SHIFT))
	.endif
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1, 4
	; set task entry pointers
	ldi     TEMP_REG_3.w0, $CODE(TSK_IOEX_STATE_CMP0_EVENT)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S0, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_IOEX_STATE_TTS_COMPLETE_EVENT)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S1, 2

	; wait for IEP warp-around to reconfigure ports
TMG_IOEX_WAIT_FOR_IEP_WRAP:
	lbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1
	qbbc	TMG_IOEX_WAIT_FOR_IEP_WRAP, TEMP_REG_1.b0, 0	; CMP0
	; IEP hit and wrap-around has occurd, now clear all CMP events
	ldi		TEMP_REG_1.b0, 0x1f
	sbco	&TEMP_REG_1.b0, ICSS_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 1

	; port configuration is done in TSK_IOEX_STATE_CMP0_EVENT task
	; configure IEP timer cycle time via CMP0
	; Load the Calculated Cycle Time in Device Data Memory
	ldi		TEMP_REG_1.w2, PARAM_OFFSET
	lbco	&TEMP_REG_1, ICSS_SHARED_RAM_CONST, TEMP_REG_1.w2, 4;
	;sub		TEMP_REG_1, TEMP_REG_1, 4 ; adjust by 4ns as IEP starts from 0 / wrap-around
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP0_REG, 4
	; configure TTS tigger - all devices start at 500ns
	ldi32	TEMP_REG_1, 500
	;qbeq	TMG_IOEX_TTS_WRITE_IEP, DEVICE_ADDR, 1
	;lbco	&TEMP_REG_2.w0, ICSS_SHARED_RAM_CONST, LINE_DELAY_PREVIOUS, 2
	; DEBUG
	;ldi		TEMP_REG_1, 800
	;add		TEMP_REG_1, TEMP_REG_1, TEMP_REG_2.w0

TMG_IOEX_TTS_WRITE_IEP:
	.if $defined(PRU0)
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP4_REG, 4
	.else
	sbco	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP3_REG, 4
	.endif


	; set SYNC0 pulse width
	ldi		TEMP_REG_1.w2, 125		; 500ns = 125 * 4ns
	ldi     TEMP_REG_2.w0, ICSS_IEP_SYNC_PWIDTH_REG
	sbco    &TEMP_REG_1.w2, ICSS_IEP_CONST, TEMP_REG_2.w0, 2
	ldi		TEMP_REG_1, 500	; set SYNC0 time
    sbco   	&TEMP_REG_1, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG, 4

    ;ldi    TEMP_REG_1, 0x06    ; clear sync_en (sync0_en, sync1_en)
    ;ldi    TEMP_REG_2.w0, ICSS_IEP_SYNC_CTRL_REG
    ;sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4
    ;ldi    TEMP_REG_1, 0x07    ;  set sync_en (sync0_en, sync1_en)
    ;sbco   &TEMP_REG_1, ICSS_IEP_CONST, TEMP_REG_2.w0, 4

	QBA		TMG_DONE

TMG_IOEX_FROM_MASTER: ; master to device direction
	; Tasks to receive via RX L2
	; enable S0, S1, S2, S3, S4
	ldi		TEMP_REG_3, (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S1_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S2_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S3_SHIFT) | (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG_TS1_EN_S4_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_GLOBAL_CFG, 4
	; map S0=RX_BK1, S1=RX_BK2, S2=RX_BKN, S3=RX_SOF, S4=RX_EOF
	ldi32	TEMP_REG_3, (1<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S0_MX_SHIFT) | (2<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S1_MX_SHIFT) | (3<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S2_MX_SHIFT) | (0<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1_TS1_GEN_S3_MX_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG1, 4
	ldi32	TEMP_REG_3, (4<<CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG2_TS1_GEN_S4_MX_SHIFT)
	sbbo	&TEMP_REG_3, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_GEN_CFG2, 4

	; set task entry pointers
	ldi     TEMP_REG_3.w0, $CODE(TSK_IOEX_STATE_FWD_BKN)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S0, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_IOEX_STATE_FWD_BKN)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S1, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_IOEX_STATE_FWD_BKN)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S2, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_IOEX_STATE_FWD_SOF)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S3, 2
	ldi     TEMP_REG_3.w0, $CODE(TSK_IOEX_STATE_FWD_EOF)
    sbbo    &TEMP_REG_3.w0, TEMP_REG_2, CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_TS1_PC_S4, 2

	;DevNote: Clarify if e/o block is needed to receive the remaining bytes
	;M_DISABLE_TXL2 ; disabling TXL2 not needed as state before does not use TXL2.
	M_SET_MIIRT_AF_LOCAL_IOEX
	; last slave needs specific configuration
	qbbc	TMG_DONE, DEVICE_STATUS, LAST_DEVICE_FLAG
	M_DISABLE_TX_LOCAL

TMG_DONE:
	M_CMD16 (D_RESET_RXFIFO | D_RX_ERROR_CLEAR | D_RESET_TXFIFO | D_TX_CRC_ERR)
	clr		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	mov		PROTOCOL_STATE, PROTOCOL_STATE_CNT

	; debug
	; store protocol state in shared memory
	.if $defined(PRU0)
	sbco	&PROTOCOL_STATE, ICSS_SHARED_RAM_CONST, DEBUG_PRU0_SLAVE_STATE, 1
	.else
	sbco	&PROTOCOL_STATE, ICSS_SHARED_RAM_CONST, DEBUG_PRU1_SLAVE_STATE, 1
	.endif
	;e/o debug

	.if $defined(PRU0)
	qbeq	LED_SET_STATE_GREEN, PROTOCOL_STATE, IOEX_STATE
	qbeq	LED_SET_STAT_RED,  PROTOCOL_STATE, IDLE_STATE
LED_SET_STAT_YELLOW:
	ldi32	TEMP_REG_1, 0x00601040
	ldi		TEMP_REG_2, 0xC0
	sbbo	&TEMP_REG_2, TEMP_REG_1, 0, 4
	qba		LED_SET_STAT_DONE

LED_SET_STATE_GREEN:
	ldi32	TEMP_REG_1, 0x00601040
	ldi		TEMP_REG_2, 0x80
	sbbo	&TEMP_REG_2, TEMP_REG_1, 0, 4
	ldi		TEMP_REG_2, 0x40
	sbbo	&TEMP_REG_2, TEMP_REG_1, 4, 4
	qba		LED_SET_STAT_DONE

LED_SET_STAT_RED:
	ldi32	TEMP_REG_1, 0x00601040
	ldi		TEMP_REG_2, 0x40
	sbbo	&TEMP_REG_2, TEMP_REG_1, 0, 4
	ldi		TEMP_REG_2, 0x80
	sbbo	&TEMP_REG_2, TEMP_REG_1, 4, 4
	qba		LED_SET_STAT_DONE

LED_SET_STAT_DONE:
	.endif

	tsen 1		;enable task manager
	RET
