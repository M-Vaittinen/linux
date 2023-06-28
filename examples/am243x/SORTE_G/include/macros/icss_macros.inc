;******************************************************************************
;+--------------------------------------------------------------------------+**
;|                            ****                                          |**
;|                            ****                                          |**
;|                            ******o***                                    |**
;|                      ********_///_****                                   |**
;|                      ***** /_//_/ ****                                   |**
;|                       ** ** (__/ ****                                    |**
;|                           *********                                      |**
;|                            ****                                          |**
;|                            ***                                           |**
;|                                                                          |**
;|         Copyright (c) 1998-2010 Texas Instruments Incorporated           |**
;|                        ALL RIGHTS RESERVED                               |**
;|                                                                          |**
;| Permission is hereby granted to licensees of Texas Instruments           |**
;| Incorporated (TI) products to use this computer program for the sole     |**
;| purpose of implementing a licensee product based on TI products.         |**
;| No other rights to reproduce, use, or disseminate this computer          |**
;| program, whether in part or in whole, are granted.                       |**
;|                                                                          |**
;| TI makes no representation or warranties with respect to the             |**
;| performance of this computer program, and specifically disclaims         |**
;| any responsibility for any damages, special or consequential,            |**
;| connected with the use of this program.                                  |**
;|                                                                          |**
;+--------------------------------------------------------------------------+**
;*****************************************************************************/
; file:   icss_macros.inc
;
; brief:  Implements common macros & defines.
;
;
;  (C) Copyright 2010, Texas Instruments, Inc
;
;  author     Kanad Kanhere
;
;  version    0.1     Created
;



    ;------------------------------------
    ; Macros
    ;------------------------------------

        ;-----------------------------------
        ; Macro Name: M_POLL_WORD_RDY
        ; Description: Polls for word ready or EOF or RX_ERR.
        ; Input Parameters: Label if EOF, Label if RX_ERR
        ; Output Parameters: none
        ;-----------------------------------
		;** The word ready check is done twice to ensure
		;** minimum delay between a word to be ready and
		;** word ready detection. Also word ready should
		;** be checked before EOF to ensure it has
		;** highest priority to handle scenarios like
		;** RX FIFO having lot of unprocessed words
		;** at the end of the frame.
M_POLL_WORD_RDY	.macro EOF_LABEL, RX_ERR_LABEL
			;QBBS WORD_RDY, R31.D_WORD_READY_FLAG_BITNUM
		WORD_CHECK_EOF:
		.if	$isdefed("NO_RX_EOF_ERROR_FLAG_BITNUM")
			QBBS EOF_LABEL, R31.D_EOF_FLAG_BITNUM
			QBBS RX_ERR_LABEL, R31.D_RX_ERROR_FLAG_BITNUM
		.else
			QBBS EOF_LABEL, R31.D_RX_EOF_ERROR_FLAG_BITNUM
		.endif			
			QBBC WORD_CHECK_EOF, R31.D_WORD_READY_FLAG_BITNUM
		WORD_RDY:
        .endm

        ;-----------------------------------
        ; Macro Name: M_POLL_BYTE_RDY
        ; Description: Polls for byte ready or EOF or RX_ERR
        ; Input Parameters: Label if EOF, Label if RX_ERR
        ; Output Parameters: none
        ;-----------------------------------
		;** The byte ready check is done twice to ensure
		;** minimum delay between a byte to be ready and
		;** byte ready detection. Also byte ready should
		;** be checked before EOF to ensure it has
		;** highest priority to handle scenarios like
		;** RX FIFO having lot of unprocessed bytes
		;** at the end of the frame.
M_POLL_BYTE_RDY	.macro  EOF_LABEL, RX_ERR_LABEL
            ;QBBS BYTE_RDY, R31.D_BYTE_READY_FLAG_BITNUM
		BYTE_CHECK_EOF:
			.if	$isdefed("NO_RX_EOF_ERROR_FLAG_BITNUM")
			QBBS EOF_LABEL, R31.D_EOF_FLAG_BITNUM
			QBBS RX_ERR_LABEL, R31.D_RX_ERROR_FLAG_BITNUM
			.else
			QBBS EOF_LABEL, R31.D_RX_EOF_ERROR_FLAG_BITNUM
			.endif	
			QBBC BYTE_CHECK_EOF, R31.D_BYTE_READY_FLAG_BITNUM
		BYTE_RDY:
        .endm

        ;-----------------------------------
        ; Macro Name: M_CMD16
        ; Description: Issue the given command.
        ; Input Parameters: 16bit command value
        ; Output Parameters: none
        ;-----------------------------------
M_CMD16	.macro cmd_val16
            LDI R31.w2, cmd_val16
        .endm

        ;-----------------------------------
        ;-----------------------------------
        ; Macro Name: M_SET_DATA_MASK16
        ; Description: Sets the given mask in R30.w2.
        ; Input Parameters: 16bit mask value
        ; Output Parameters: none
        ;-----------------------------------
M_SET_DATA_MASK16	.macro mask_val16
            MOV R30.w2, mask_val16
        .endm
        ;-----------------------------------
        ;-----------------------------------
        ; Macro Name: M_SET_CMD
        ; Description: Sets the given command R31.w2.
        ; Input Parameters: 16bit mask value
        ; Output Parameters: none
        ;-----------------------------------
M_SET_CMD	.macro  cmd_val16 
            LDI R31.w2, cmd_val16
        .endm
        ; Macro Name: M_POP_BYTE
        ; Description: Pop a byte from RX FIFO.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_POP_BYTE	.macro
            M_CMD16 D_POP_BYTE_CMD
        .endm

        ;-----------------------------------
        ; Macro Name: M_POP_WORD
        ; Description: Pop a word from RX FIFO.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_POP_WORD	.macro
            M_CMD16 D_POP_WORD_CMD
        .endm

        ;-----------------------------------
        ; Macro Name: M_PUSH_BYTE
        ; Description: Push a byte into TX fifo.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_PUSH_BYTE	.macro
            M_CMD16 D_PUSH_BYTE_CMD
        .endm

        ;-----------------------------------
        ; Macro Name: M_PUSH_WORD
        ; Description: Push a word into TX fifo.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_PUSH_WORD_CMD	.macro
            M_CMD16 D_PUSH_WORD_CMD
        .endm

        ;-----------------------------------
        ; Macro Name: M_PUSH_N_POP_BYTE
        ; Description: Push a byte into TX fifo and pop byte from RX FIFO.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_PUSH_N_POP_BYTE	.macro  
           M_CMD16 D_PUSH_N_POP_BYTE_CMD
        .endm

        ;-----------------------------------
        ; Macro Name: M_PUSH_N_POP_WORD
        ; Description: Push a word into TX fifo and pop word from RX FIFO.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_PUSH_N_POP_WORD	.macro
            M_CMD16 D_PUSH_N_POP_WORD_CMD
        .endm

        ;-----------------------------------
        ; Macro Name: M_PUSH_CRC_MSWORD
        ; Description: Push 31:16 bits of CRC into TX FIFO.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_PUSH_CRC_MSWORD	.macro
            M_CMD16 D_PUSH_CRC_MSWORD_CMD
        .endm

        ;-----------------------------------
        ; Macro Name: M_PUSH_CRC_LSWORD
        ; Description: Push 15:0 bits of CRC into TX FIFO.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_PUSH_CRC_LSWORD	.macro
            M_CMD16 D_PUSH_CRC_LSWORD_CMD
        .endm

        ;-----------------------------------
        ; Macro Name: M_PUSH_ERR_NIBBLE
        ; Description: Push error marker nibble in TX FIFO.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_PUSH_ERR_NIBBLE	.macro
            M_CMD16 D_PUSH_ERR_NIBBLE_CMD
        .endm
		
		;-----------------------------------
        ; Macro Name: M_PUSH_TX_EOF
        ; Description: Push error marker nibble in TX FIFO.
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_PUSH_TX_EOF	.macro
            M_CMD16 D_TX_EOF	;D_PUSH_TX_EOF_CMD
        .endm

		;-----------------------------------
        ; Macro Name: M_RESET_RXFIFO
        ; Description: Reset RXFIFO
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_RESET_RXFIFO	.macro
            M_CMD16 D_RESET_RXFIFO
        .endm
		;-----------------------------------
        ; Macro Name: M_RESET_TXFIFO
        ; Description: Reset TXFIFO
        ; Input Parameters: none
        ; Output Parameters: none
        ;-----------------------------------
M_RESET_TXFIFO	.macro  
            M_CMD16 D_RESET_TXFIFO
        .endm

wait	.macro delay
l1:	sub delay,delay,1
	qblt l1,delay,0
	.endm

M_PUSH_WORD	.macro	arg1, arg2
loop1:
	MVIW	TX_DATA_WORD, *arg1
	M_PUSH_WORD_CMD
	.if	$isdefed("xmt_debug")	
	MVIW	Debug_reg, *arg1
	SBBO	Debug_reg, Debug_base, R12.w0, 2
	ADD		R12.w0, R12.w0, 2
	.endif	
	ADD		arg1, arg1, 2
	SUB		arg2, arg2, 1
	QBNE	loop1, arg2, 0
	.endm

M_TX_PORT_CFG	.macro	
	LDI		TEMP_REG_1.b0, 0xb8  ; This translates to 920ns( 0xb8 = 184 *5ns) , 40ns is hardware delay, so we get IFG of 960ns
	SBCO	TEMP_REG_1.b0, MII_RT_CFG_CONST, ICSS_MIIRT_TXIPG0, #1
	SBCO	TEMP_REG_1.b0, MII_RT_CFG_CONST, ICSS_MIIRT_TXIPG1, #1
	LDI		TEMP_REG_1.w2, 0x0040
	LDI		TEMP_REG_1.w0, 0x0003
	.if	$isdefed("TWO_PORT_CFG")
	SBCO 	TEMP_REG_1, MII_RT_CFG_CONST, 0x14, 4
	LDI		TEMP_REG_1.w0, 0x0103
	SBCO 	TEMP_REG_1, MII_RT_CFG_CONST, 0x10, 4
	.else
	SBCO 	TEMP_REG_1, MII_RT_CFG_CONST, 0x10, 4
	LDI		TEMP_REG_1.w0, 0x0103
	SBCO 	TEMP_REG_1, MII_RT_CFG_CONST, 0x14, 4	

	.endif
	.endm

M_TX_EN	.macro
	.if	$isdefed("TWO_PORT_CFG")

	.if	$isdefed("PRU0")
	LDI	 TEMP_REG_1.b0, 0x03	
	SBCO TEMP_REG_1.b0, MII_RT_CFG_CONST, 0x14, #1 
	
	.else
	LDI	 TEMP_REG_1.b0, 0x03	
	SBCO TEMP_REG_1.b0, MII_RT_CFG_CONST, 0x10, #1 
	.endif	
	.else
	.if	$isdefed("PRU0")
		LDI	 TEMP_REG_1.b0, 0x03	
		SBCO TEMP_REG_1.b0, MII_RT_CFG_CONST, 0x10, #1
	.else
		LDI	 TEMP_REG_1.b0, 0x03
		SBCO TEMP_REG_1.b0, MII_RT_CFG_CONST, 0x14, #1
	.endif	

	.endif ;TWO_PORT_CFG
	.endm

M_TX_DISABLE	.macro
	.if	$isdefed("TWO_PORT_CFG")
	.if	$isdefed("PRU0")
	LDI  TEMP_REG_1.b0, 0x02
	SBCO TEMP_REG_1.b0, MII_RT_CFG_CONST, 0x14, #1 
	.else
	LDI  TEMP_REG_1.b0, 0x02
	SBCO TEMP_REG_1.b0, MII_RT_CFG_CONST, 0x10, #1 
	.endif	
	.else
	.if	$isdefed("PRU0")
		LDI  TEMP_REG_1.b0, 0x02
		SBCO TEMP_REG_1.b0, MII_RT_CFG_CONST, 0x10, #1
	.else
		LDI  TEMP_REG_1.b0, 0x02
		SBCO TEMP_REG_1.b0, MII_RT_CFG_CONST, 0x14, #1
	.endif	

	.endif ;TWO_PORT_CFG
	.endm

M_TX_RESET	.macro
        SET R31, R31.t30
	.endm

M_PUSH_8	.macro
        SET R31.t24
	.endm
		; DIVU5 : integer divide by 5  
; n is in arg1, result is in arg3, arg2 has remainder 1..4 if not modulo 5
; arg2 and arg3 are registers used, arg1 remains unchanged
; cycles: 14/15  (70/75ns)
; instructions: 14/15
DIVU5	.macro	 arg1, arg2, arg3
	; q = (n>>1) + (n>>2)
	LSR     arg2, arg1, 1
	LSR     arg3, arg1, 2
	ADD     arg2, arg2, arg3
	; q = q + (q>>4)
	LSR     arg3, arg2, 4
	ADD		arg2, arg2, arg3
	; q = q + (q>>8)
	LSR     arg3, arg2, 8
	ADD		arg2, arg2, arg3
	; q = q + (q>>16)
	LSR     arg3, arg2, 16
	ADD		arg2, arg2, arg3
	; q = q >> 2
    LSR		arg3, arg2, 2
    ; r = n - (q << 2) + q
    ; arg2 has already q << 2, but LSB is not correct
	LSL 	arg2, arg3, 2
    ADD  	arg2, arg2 , arg3	
	SUB     arg2, arg1, arg2
	; r is in arg2 and q in arg3
	; if arg2 >= 5 then q++
    QBGT    L_DIVU5, arg2, 5
    ADD     arg3, arg3, 1
L_DIVU5:	
	.endm	

