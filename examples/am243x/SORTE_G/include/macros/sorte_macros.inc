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
; 	0.1     	Created								Thomas Leyrer


; set state register with actual protocol state
;

M_SET_STATE .macro state_3, temp_8
	lbco	&temp_8, ICSS_DMEM0_CONST, 4, 1
    and		:temp_8:, :temp_8:, 0xf8
    or		:temp_8:, :temp_8:, state_3
	sbco	&temp_8, ICSS_DMEM0_CONST, 4, 1

	.endm

	; STATES Field
	.define		1,S_DISCOV
	.define		2,S_PARAM
	.define 	3,S_LINED
	.define 	4,S_SYNC
	.define		5,S_IOEXCHANGE
	.define		6,S_ERROR
