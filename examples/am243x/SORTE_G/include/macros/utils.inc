;******************************************************************************
;+--------------------------------------------------------------------------+**
;|                            ****                                          |**
;|                            ****                                          |**
;|                            ******o***                                    |**
;|                      ********_;/_****                                    |**
;|                      ***** /_;_/ ****                                    |**
;|                       ** ** (__/ ****                                    |**
;|                           *********                                      |**
;|                            ****                                          |**
;|                            ***                                           |**
;|                                                                          |**
;|         Copyright (c) 1998-2014 Texas Instruments Incorporated           |**
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
;******************************************************************************
;  (C) Copyright 2016, Texas Instruments, Inc
;
;	Version		Description								Author
; 	0.1     	Created						Thomas Leyrer


; crc8 calculation.
; input: crc_8 initial value. Should be 0 at start of frame or interim value in block processing
;        src_8 register at which the frame data starts 
;        lenght_8 number of bytes of frame
;        

CRC8_CALC .macro crc_8, src_8, lenght_8, temp

	ldi		R1.b0, &src_8

	LOOP    l?, lenght_8
	mvib	temp.b0, *R1.b0++
	xor		:crc_8:, :crc_8:, temp.b0
    ldi		temp.w2, CRC8_TABLE_OFFESET
	add		temp.w2, crc_8, temp.w2
	lbco	&crc_8, PRU0_DMEM_CONST, temp.w2, 1
l?
	.endm

