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

M_INIT_SIN2_TBL .macro
; 1/sin(x)² * 1000 in hex
	ldi32    r2, 0x65677
	ldi32    r3, 0x19698
	ldi    r4.w0, 0xB570
	ldi    r4.w2, 0x66A3
	ldi    r5.w0, 0x422A
	ldi    r5.w2, 0x2E5B
	ldi    r6.w0, 0x226B
	ldi    r6.w2, 0x1AAD
	ldi    r7.w0, 0x155E
	ldi    r7.w2, 0x1194
	ldi    r8.w0, 0xEC8
	ldi    r8.w2, 0xCA8
	ldi    r9.w0, 0xB02
	ldi    r9.w2, 0x9B5
	ldi    r10.w0, 0x8A9
	ldi    r10.w2, 0x7D0
	ldi    r11.w0, 0x71D
	ldi    r11.w2, 0x68A
	ldi    r12.w0, 0x60E
	ldi    r12.w2, 0x5A6
	ldi    r13.w0, 0x54F
	ldi    r13.w2, 0x506
	ldi    r14.w0, 0x4C8
	ldi    r14.w2, 0x494
	ldi    r15.w0, 0x468
	ldi    r15.w2, 0x444
	ldi    r16.w0, 0x427
	ldi    r16.w2, 0x410
	ldi    r17.w0, 0x3FE
	ldi    r17.w2, 0x3F2
	ldi    r18.w0, 0x3EA
	ldi    r18.w2, 0x3e8
    ldi	   r19, SIN2_TBL32_OFFSET
    sbco	&r2,PRU0_DMEM_CONST, r19, 68
	.endm

	; 1/fstep[MHz]*2^16
note_cis	.set	932886
note_E		.set	780121
note_fis	.set	693009
note_G		.set	653191
note_gis	.set	615572
note_A		.set	580294
note_H		.set	515897
note_cis2	.set	458659
note_D2		.set	432486
note_fis2	.set	342190
note_pause	.set 	0

; 125 ms at fstep
p125_cis	.set	8781
p125_E		.set	10501
p125_fis	.set 	11821
p125_G		.set    12542
p125_gis	.set	13308
p125_A		.set	14117
p125_H		.set	15879
p125_cis2	.set	17861
p125_D2		.set	18942
p125_fis2	.set	23940
p125		.set	31250
p1			.set	250


M_INIT_AXEL_F_TBL .macro
; format: 32 bit 1/fstep
	ldi32	r2, 	note_fis    ; 1
	ldi32	r3,     2*p125_fis
	ldi32	r4,     4*p125
	ldi32	r5, 	note_G		;2
	ldi32	r6,     p125_G
	ldi32	r7,     2*p125
	ldi32	r8, 	note_fis	;3
	ldi32	r9,     p125_fis
	ldi32	r10,    p125
	ldi32	r11, 	note_fis	;4
	ldi32	r12,    p125_fis
	ldi32	r13,    p125
	ldi32	r14, 	note_H      ;5
	ldi32	r15,    p125_H
	ldi32	r16,    p1
	ldi32	r17, 	note_fis	;6
	ldi32	r18,    p125_fis
	ldi32	r19,    p125
	ldi32	r20, 	note_E      ;7
	ldi32	r21,    p125_E
	ldi32	r22, 	p125
	ldi32	r23, 	note_fis    ;8
	ldi32	r24,    2*p125_fis
	ldi32	r25,    p125
	ldi32	r26, 	note_cis2    ;9
	ldi32	r27,    2*p125_cis2
	ldi32	r28,    2*p125
	ldi	   	r29, AXEL_F_TBL_OFFSET+4
	sbco	&r2,PRU0_DMEM_CONST, r29, 27*4

	ldi32	r2, 	note_fis    ; 1
	ldi32	r3,     p125_fis
	ldi32	r4,     p125
	ldi32	r5, 	note_fis		;2
	ldi32	r6,     p125_fis
	ldi32	r7,     p125
	ldi32	r8, 	note_D2		;3
	ldi32	r9,     p125_D2
	ldi32	r10,    p1
	ldi32	r11, 	note_cis2	;4
	ldi32	r12,    p125_cis2
	ldi32	r13,    p1
	ldi32	r14, 	note_A      ;5
	ldi32	r15,    p125_A
	ldi32	r16,    p125
	ldi32	r17, 	note_fis	;6
	ldi32	r18,    p125_fis
	ldi32	r19,    p125
	ldi32	r20, 	note_cis2      ;7
	ldi32	r21,    p125_cis2
	ldi32	r22, 	p125
	ldi32	r23, 	note_fis2   ;8
	ldi32	r24,    p125_fis2
	ldi32	r25,    p125
	ldi32	r26, 	note_fis    ;9
	ldi32	r27,    p125_fis
	ldi32	r28,    p125
	ldi	   	r29, AXEL_F_TBL_OFFSET+4+27*4
	sbco	&r2,PRU0_DMEM_CONST, r29, 27*4

	ldi32	r2, 	note_E    ; 1
	ldi32	r3,     p125_E
	ldi32	r4,     p1
	ldi32	r5, 	note_E		;2
	ldi32	r6,     p125_E
	ldi32	r7,     p125
	ldi32	r8, 	note_cis		;3
	ldi32	r9,     p125_cis
	ldi32	r10,    p1
	ldi32	r11, 	note_gis	;4
	ldi32	r12,    p125_gis
	ldi32	r13,    p125
	ldi32	r14, 	note_fis      ;5
	ldi32	r15,    4*p125_fis
	ldi32	r16,    p125

	ldi	   	r29, AXEL_F_TBL_OFFSET+4+27*8
	sbco	&r2,PRU0_DMEM_CONST, r29, 15*4
	.endm
