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
;     file:   event_handler.asm
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
	#include "sorte_g_device_host_interface.h"
%}

; includes from \icss-g-industrial-fw\pru_fw_common
	;.include "icss_bsram_macros.h"
    ;.include "pru_constant_defines.h"
    ;.include "icss_xfer_defines.h"
    ;.include "icss_tm_macros.h"
    ;.include "hw/icss_iep_regs.h"
    ;.include "hw/icss_intc_regs.h"
    ;.include "pru_macros.h"

    .include "icss_regs.inc"
    .include "sorte_g_pru_device_register.inc"
    .include "sorte_g_state_define.inc"
    .include "sorte_g_device_cfg.inc"
    .include "sorte_g_host_interface.inc"
    .include "macros.inc"
    .include "irq_num.inc"


;CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

  	.global     main
	.global		FN_EVENT_HANDLER
    .sect    ".text"


FN_EVENT_HANDLER:
	.if $defined (PRU0)	; PRU0 uses bit 30, PRU1 uses bit 31
	qbbc	EVH_DONE, R31, 30
	; load highest priority currently pending interrupt for host interrupt 0
EVENT_HANDLER_RESTART:
	ldi		TEMP_REG_2.w0, ICSS_INTC_HIPIR0
	.else
	qbbc	EVH_DONE, R31, 31
	; load highest priority currently pending interrupt for host interrupt 1
EVENT_HANDLER_RESTART:
	ldi		TEMP_REG_2.w0, ICSS_INTC_HIPIR1
	.endif
	; Store event number in TEMP_REG_6. None of ISRs below should overwrite this register
	lbco	&TEMP_REG_6, ICSS_INTC_CONST, TEMP_REG_2.w0, 4

	qbbs	EVH_DONE, TEMP_REG_6, NONE_HINT_BIT
	; clear event in INTC after event has been handled: TEMP_REG_6 should not be overwritten!

	; event jump table
	;.if $defined(PRU0)
	;.else
	;.endif
	qbeq	EVH_MDIO_EVENT_PEND, TEMP_REG_6, SYS_EVT_MDIO_MII_LINK1
	qbeq	EVH_MDIO_EVENT_PEND, TEMP_REG_6, SYS_EVT_MDIO_MII_LINK0
	qbeq	EVH_TX_FIFO_OVERFLOW, TEMP_REG_6, SYS_EVT_PORT0_TX_OVERFLOW
	qbeq	EVH_TX_FIFO_OVERFLOW, TEMP_REG_6, SYS_EVT_PORT1_TX_OVERFLOW
	qbeq	EVH_TX_FIFO_OVERFLOW, TEMP_REG_6, SYS_EVT_PORT0_TX_UNDERFLOW
	qbeq	EVH_TX_FIFO_OVERFLOW, TEMP_REG_6, SYS_EVT_PORT1_TX_UNDERFLOW
	qbeq	EVH_PRU2PRU_EVT1_PEND, TEMP_REG_6, PRU2PRU_EVT1
	qbeq	EVH_PRU2PRU_EVT2_PEND, TEMP_REG_6, PRU2PRU_EVT2

	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEBUG_UNREGISTERED_EVENT, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, 1
	sbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEBUG_UNREGISTERED_EVENT, 2

	; Received an event which is not registered.
	; clear this not registered event via EVH_CLR_INTC_EVENT
EVH_CLR_INTC_EVENT:
	sbco	&TEMP_REG_6, ICSS_INTC_CONST, ICSS_INTC_SICR, 4
	qba 	EVENT_HANDLER_RESTART

EVH_DONE:
	RET

EVH_RX_EOF_DONE:
	;clr		DEVICE_STATUS, DEVICE_STATUS, SWITCH_PORT_CONFIG
	qba		EVH_CLR_INTC_EVENT

;**********************
; Event to transition this PRU to RESET state
;**********************
EVH_PRU2PRU_EVT2_PEND:
	; initiate state change switch
	ldi		PROTOCOL_STATE_CNT, RESET_STATE
	set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	clr		DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	; ToDo: check if this is the last device
	;set		DEVICE_STATUS, DEVICE_STATUS, LAST_DEVICE_FLAG

	lbco	&DEVICE_ADDR, ICSS_SHARED_RAM_CONST, DEVICE_ADDR_OFFSET, 1
	; store protocol state in shared memory
	.if $defined(PRU0)
	sbco	&PROTOCOL_STATE, ICSS_SHARED_RAM_CONST, DEBUG_PRU0_SLAVE_STATE, 1
	.else
	sbco	&PROTOCOL_STATE, ICSS_SHARED_RAM_CONST, DEBUG_PRU1_SLAVE_STATE, 1
	.endif
	qba		EVH_CLR_INTC_EVENT

;**********************
; Event to transition this PRU to IOEX state
;**********************
EVH_PRU2PRU_EVT1_PEND:
	; initiate state change switch
	ldi		PROTOCOL_STATE_CNT, IOEX_STATE
	set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	clr		DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	; ToDo: check if this is the last device
	;set		DEVICE_STATUS, DEVICE_STATUS, LAST_DEVICE_FLAG

	lbco	&DEVICE_ADDR, ICSS_SHARED_RAM_CONST, DEVICE_ADDR_OFFSET, 1
	; store protocol state in shared memory
	.if $defined(PRU0)
	sbco	&PROTOCOL_STATE, ICSS_SHARED_RAM_CONST, DEBUG_PRU0_SLAVE_STATE, 1
	.else
	sbco	&PROTOCOL_STATE, ICSS_SHARED_RAM_CONST, DEBUG_PRU1_SLAVE_STATE, 1
	.endif
	qba		EVH_CLR_INTC_EVENT


;**********************
; TX FIFO Overflow event handler
;**********************
EVH_TX_FIFO_OVERFLOW:
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEBUG_TXFIFOOVERFLOW, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, 1
	sbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEBUG_TXFIFOOVERFLOW, 2
	; clear errors
	M_CMD16 D_RESET_TXFIFO | D_TX_CRC_ERR
	qba		EVH_CLR_INTC_EVENT

;**********************
; TX FIFO Uverflow event handler
;**********************
EVH_TX_FIFO_UNDERFLOW:
	lbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEBUG_TXFIFOUNDERFLOW, 2
	add		TEMP_REG_1.w0, TEMP_REG_1.w0, 1
	sbco	&TEMP_REG_1.w0, ICSS_SHARED_RAM_CONST, DEBUG_TXFIFOUNDERFLOW, 2
	; clear errors
	M_CMD16 D_RESET_TXFIFO | D_TX_CRC_ERR
	qba		EVH_CLR_INTC_EVENT

;**********************
; MDIO event handler
;**********************
EVH_MDIO_EVENT_PEND:
	; clear MDIO event in MDIO module
	ldi		TEMP_REG_1, 0x03
	sbco	&TEMP_REG_1, ICSS_MDIO_CONST, ICSS_MDIO_LINKINTMASKED, 4

	; get current link information from MDIO module
	lbco	&TEMP_REG_1, ICSS_MDIO_CONST, ICSS_MDIO_LINK, 4
	ldi		TEMP_REG_2.w0, MDIO_PHY_CONFIG_OFFSET
	lbco	&TEMP_REG_2, ICSS_SHARED_RAM_CONST, TEMP_REG_2.w0, 4
	zero	&TEMP_REG_3.w0, 2
PM_PHY_0:
	; parse link bit for PHY0
	qbbc	PM_PHY_0_ACT_HIGH, TEMP_REG_2, 16		; 0: active high; 1: active low
;PM_PHY_0_ACT_LOW:
	qbbs	PM_PHY_1, TEMP_REG_1, TEMP_REG_2.b0
	qba		PM_PHY_0_CONFIG
PM_PHY_0_ACT_HIGH:
	qbbc	PM_PHY_1, TEMP_REG_1, TEMP_REG_2.b0
PM_PHY_0_CONFIG:
	set		TEMP_REG_3.b0, TEMP_REG_3.b0, 0	;
PM_PHY_1:
	; parse link bit for PHY1
	qbbc	PM_PHY_1_ACT_HIGH, TEMP_REG_2, 24		; 0: active high; 1: active low
; PM_PHY_1_ACT_LOW:
	qbbs	PM_STORE_PHY_STATUS, TEMP_REG_1, TEMP_REG_2.b1
	qba		PM_PHY_1_CONFIG
PM_PHY_1_ACT_HIGH:
	qbbc	PM_STORE_PHY_STATUS, TEMP_REG_1, TEMP_REG_2.b1
PM_PHY_1_CONFIG:
	set		TEMP_REG_3.b1, TEMP_REG_3.b1, 0
PM_STORE_PHY_STATUS:

	;lbco	&TEMP_REG_3.w2, ICSS_SHARED_RAM_CONST, PHY_STATUS_OFFSET, 2
	.if $defined(PRU0)	;if(pru0)
	lbco	&TEMP_REG_3.w2, ICSS_SHARED_RAM_CONST, PHY_STATUS_OFFSET_PRU0, 2
	.else
	lbco	&TEMP_REG_3.w2, ICSS_SHARED_RAM_CONST, PHY_STATUS_OFFSET_PRU1, 2
	.endif

	qbeq	PM_STORE_PHY_STATUS_2, TEMP_REG_3.w0, TEMP_REG_3.w2
	; send event to other PRU about port change
	.if $defined(PRU0)
	ldi32	TEMP_REG_1, 0x00000200	; Set MDIO_MII_LINK[0] bit
	ldi		TEMP_REG_2.w0, ICSS_INTC_SRSR1
	sbco	&TEMP_REG_1, ICSS_INTC_CONST, TEMP_REG_2.w0, 4
	.else
	ldi32	TEMP_REG_1, 0x00200000	; Set MDIO_MII_LINK[1]
	ldi		TEMP_REG_2.w0, ICSS_INTC_SRSR1
	sbco	&TEMP_REG_1, ICSS_INTC_CONST, TEMP_REG_2.w0, 4
	.endif
PM_STORE_PHY_STATUS_2:

	;sbco	&TEMP_REG_3.w0, ICSS_SHARED_RAM_CONST, PHY_STATUS_OFFSET, 2
	.if $defined(PRU0)	;if(pru0)
	sbco	&TEMP_REG_3.w0, ICSS_SHARED_RAM_CONST, PHY_STATUS_OFFSET_PRU0, 2
	.else
	sbco	&TEMP_REG_3.w0, ICSS_SHARED_RAM_CONST, PHY_STATUS_OFFSET_PRU1, 2
	.endif

	;check the state of which port has been changed
	mov		TEMP_REG_4, TEMP_REG_3
	xor		TEMP_REG_4.w2, TEMP_REG_4.w2, TEMP_REG_4.w0

	and		TEMP_REG_3.b0, TEMP_REG_3.b0, TEMP_REG_3.b1
	qbbs	PM_SET_AF_CONFIG, TEMP_REG_3.b0, 0
	set		DEVICE_STATUS, DEVICE_STATUS, LAST_DEVICE_FLAG
	;set	DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	;or		DEVICE_STATUS, DEVICE_STATUS, (1<<LAST_DEVICE_FLAG) | (1<<PRU_MASTER_PORT_FLAG)
	; DevNote: whenever there is a link change, SORTE should restart this slave in idle state.
	;ldi	PROTOCOL_STATE_CNT, IDLE_STATE
	;set	DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	qba		PM_CHECK_FOR_RESTART
PM_SET_AF_CONFIG:
	; ToDo: use 'and' to clear the two flags
	clr		DEVICE_STATUS, DEVICE_STATUS, LAST_DEVICE_FLAG
	;clr	DEVICE_STATUS, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	; DevNote: whenever there is a link change, SORTE should restart this slave in idle state.
	;ldi	PROTOCOL_STATE_CNT, IDLE_STATE
	;set	DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
PM_CHECK_FOR_RESTART:

	.if ! $defined(TI_MDIO_EVENT_HANDLING)
	qbeq	PM_MDIO_EVENT_IDLE, PROTOCOL_STATE, IDLE_STATE
	qbeq	PM_MDIO_EVENT_IOEX, PROTOCOL_STATE, IOEX_STATE
	qba		PM_NO_RESTART

PM_MDIO_EVENT_IDLE:
	; force to reconfigure ports in indle state
	set		DEVICE_STATUS, DEVICE_STATUS, SWITCH_STATE
	qba		PM_NO_RESTART

PM_MDIO_EVENT_IOEX:
	zero	&TEMP_REG_1, 4
	; generate int to trigger LINK_CHANGE_EVENT
	ldi		TEMP_REG_1.w0, IRQ_SVC_ID_OFFSET

	.if $defined(PRU0)

PM_MDIO_TEST_PHY_0_STATUS:
	qbbc	PM_MDIO_TEST_PHY_1_STATUS, TEMP_REG_4.b2, 0
PM_MDIO_LINK_CHANGE_PHY0:
	qbbc	PM_MDIO_SEND_SLAVE_PORT_EVENT_PHY_0, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
PM_MDIO_SEND_MASTER_PORT_EVENT_PHY_0:
	qbbc	PM_MDIO_SET_MASTER_PORT_LINK_DOWN_PHY_0, TEMP_REG_4.b0, 0
PM_MDIO_SET_MASTER_PORT_LINK_UP_PHY_0:
	ldi		TEMP_REG_1.w2, IRQ_SVC_MASTER_PORT_LINK_UP
	qba		PM_MDIO_TEST_PHY_1_STATUS
PM_MDIO_SET_MASTER_PORT_LINK_DOWN_PHY_0:
	ldi		TEMP_REG_1.w2, IRQ_SVC_MASTER_PORT_LINK_DOWN
	qba		PM_MDIO_TEST_PHY_1_STATUS
PM_MDIO_SEND_SLAVE_PORT_EVENT_PHY_0:
	qbbc	PM_MDIO_SET_SLAVE_PORT_LINK_DOWN_PHY_0, TEMP_REG_4.b0, 0
PM_MDIO_SET_SLAVE_PORT_LINK_UP_PHY_0:
	ldi		TEMP_REG_1.w2, IRQ_SVC_SLAVE_PORT_LINK_UP
	qba		PM_MDIO_TEST_PHY_1_STATUS
PM_MDIO_SET_SLAVE_PORT_LINK_DOWN_PHY_0:
	ldi		TEMP_REG_1.w2, IRQ_SVC_SLAVE_PORT_LINK_DOWN
PM_MDIO_TEST_PHY_1_STATUS:
	qbbc	PM_MDIO_SEND_APPL_EVENT, TEMP_REG_4.b3, 0
PM_MDIO_LINK_CHANGE_PHY1:
	qbbc	PM_MDIO_SEND_SLAVE_PORT_EVENT_PHY_1, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
PM_MDIO_SEND_MASTER_PORT_EVENT_PHY_1:
	qbbc	PM_MDIO_SET_SLAVE_PORT_LINK_DOWN_PHY_1, TEMP_REG_4.b1, 0
PM_MDIO_SET_SLAVE_PORT_LINK_UP_PHY_1:
	ldi		TEMP_REG_1.w2, IRQ_SVC_SLAVE_PORT_LINK_UP
	qba 	PM_MDIO_SEND_APPL_EVENT
PM_MDIO_SET_SLAVE_PORT_LINK_DOWN_PHY_1:
	ldi		TEMP_REG_1.w2, IRQ_SVC_SLAVE_PORT_LINK_DOWN
	qba 	PM_MDIO_SEND_APPL_EVENT
PM_MDIO_SEND_SLAVE_PORT_EVENT_PHY_1:
	qbbc	PM_MDIO_SET_MASTER_PORT_LINK_DOWN_PHY_1, TEMP_REG_4.b1, 0
PM_MDIO_SET_MASTER_PORT_LINK_UP_PHY_1:
	ldi		TEMP_REG_1.w2, IRQ_SVC_MASTER_PORT_LINK_UP
	qba 	PM_MDIO_SEND_APPL_EVENT
PM_MDIO_SET_MASTER_PORT_LINK_DOWN_PHY_1:
	ldi		TEMP_REG_1.w2, IRQ_SVC_MASTER_PORT_LINK_DOWN

	;--------------
	.else	; .if $defined(PRU0)
	;--------------

PM_MDIO_TEST_PHY_0_STATUS:
	qbbc	PM_MDIO_TEST_PHY_1_STATUS, TEMP_REG_4.b2, 0
PM_MDIO_LINK_CHANGE_PHY0:
	qbbc	PM_MDIO_SEND_SLAVE_PORT_EVENT_PHY_0, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
PM_MDIO_SEND_MASTER_PORT_EVENT_PHY0:
	qbbc	PM_MDIO_SET_SLAVE_PORT_LINK_DOWN_PHY_0, TEMP_REG_4.b0, 0
PM_MDIO_SET_SLAVE_PORT_LINK_UP_PHY_0:
	ldi		TEMP_REG_1.w2, IRQ_SVC_SLAVE_PORT_LINK_UP
	qba		PM_MDIO_TEST_PHY_1_STATUS
PM_MDIO_SET_SLAVE_PORT_LINK_DOWN_PHY_0:
	ldi		TEMP_REG_1.w2, IRQ_SVC_SLAVE_PORT_LINK_DOWN
	qba		PM_MDIO_TEST_PHY_1_STATUS
PM_MDIO_SEND_SLAVE_PORT_EVENT_PHY_0:
	qbbc	PM_MDIO_SET_MASTER_PORT_LINK_DOWN_PHY_0, TEMP_REG_4.b0, 0
PM_MDIO_SET_MASTER_PORT_LINK_UP_PHY_0:
	ldi		TEMP_REG_1.w2, IRQ_SVC_MASTER_PORT_LINK_UP
	qba		PM_MDIO_TEST_PHY_1_STATUS
PM_MDIO_SET_MASTER_PORT_LINK_DOWN_PHY_0:
	ldi		TEMP_REG_1.w2, IRQ_SVC_MASTER_PORT_LINK_DOWN
PM_MDIO_TEST_PHY_1_STATUS:
	qbbc	PM_MDIO_SEND_APPL_EVENT, TEMP_REG_4.b3, 0
PM_MDIO_LINK_CHANGE_PHY1:
	qbbc	PM_MDIO_SEND_SLAVE_PORT_EVENT_PHY_1, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
PM_MDIO_SEND_MASTER_PORT_EVENT_PHY_1:
	qbbc	PM_MDIO_SET_MASTER_PORT_LINK_DOWN_PHY_1, TEMP_REG_4.b1, 0
PM_MDIO_SET_MASTER_PORT_LINK_UP_PHY_1:
	ldi		TEMP_REG_1.w2, IRQ_SVC_MASTER_PORT_LINK_UP
	qba 	PM_MDIO_SEND_APPL_EVENT
PM_MDIO_SET_MASTER_PORT_LINK_DOWN_PHY_1:
	ldi		TEMP_REG_1.w2, IRQ_SVC_MASTER_PORT_LINK_DOWN
	qba		PM_MDIO_SEND_APPL_EVENT
PM_MDIO_SEND_SLAVE_PORT_EVENT_PHY_1:
	qbbc	PM_MDIO_SET_SLAVE_PORT_LINK_DOWN_PHY_1, TEMP_REG_4.b1, 0
PM_MDIO_SET_SLAVE_PORT_LINK_UP_PHY_1:
	ldi		TEMP_REG_1.w2, IRQ_SVC_SLAVE_PORT_LINK_UP
	qba 	PM_MDIO_SEND_APPL_EVENT
PM_MDIO_SET_SLAVE_PORT_LINK_DOWN_PHY_1:
	ldi		TEMP_REG_1.w2, IRQ_SVC_SLAVE_PORT_LINK_DOWN

	.endif ; .if $defined(PRU0)

PM_MDIO_SEND_APPL_EVENT:

	; no event - no interrupt
	qbeq	PM_NO_RESTART, TEMP_REG_1.w2, 0

	; write irq num to the shared mem
	sbco	&TEMP_REG_1.w2, ICSS_SHARED_RAM_CONST, TEMP_REG_1.w0, 2
	;ldi	R31, 36		; generate int
	ldi		R31, IRQ_SVC		; generate int
	qba		PM_NO_RESTART

	.else ; .if ! $defined(TI_MDIO_EVENT_HANDLING)

	; restart if device is in non-idle state and if master port detects MDIO event
	qbeq	PM_NO_RESTART, PROTOCOL_STATE, IDLE_STATE
	qbbc	PM_NO_RESTART, DEVICE_STATUS, PRU_MASTER_PORT_FLAG
	.if $defined(PRU0)
	qbbs	EVH_CLR_INTC_EVENT, TEMP_REG_3.w0, 0		; Port 0 is bit 0
	.else
	qbbs	EVH_CLR_INTC_EVENT, TEMP_REG_3.w0, 8		; Port 1 is bit 8
	.endif
	;jmp		FN_RESTART_FIRMWARE

	.endif ; .if ! $defined(TI_MDIO_EVENT_HANDLING)

PM_NO_RESTART:
	qba		EVH_CLR_INTC_EVENT
