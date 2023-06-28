; Copyright (C) 2018 Texas Instruments Incorporated - http:;www.ti.com/
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
;     file:  rx.asm
;
;     brief: rx function for ICCS_G uses macros from

 
    .cdecls C,NOLIST
%{
#include "cslr_icss_g.h"
%}
 
    .include "icss_regs.inc"

    .include "pru_macros.h"

	.include "sorte_g_master_cfg.inc"
;	.include "include/icss/icss_event_defs.inc"
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


; work with permanent register r27 using the following definition
; r27.b0 = 32 byte block counter
; r27.b1 = # of bytes in last block 0..31 bytes
; r27.b1 bit 0 (LSB) = side flag used during receive and replaced by final block length

rx_fb:
; enter nrt first block after 32 bytes are received
 .if $defined(PRU0)
    xout    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xout    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
    zero    &r10, 8                 ; clear R10 and R11
    ldi     r10.w0, DISCOV_RX_OFFSET; init start address
 .if $defined(AM65X)
    ldi     r10.w2, 0x0b20          ; host memory address of ICSS_G2 = 0x0b200000
 .else
    ldi     r10.w2, 0x3008          ; host memory address of ICSS_G1 = 0x030080000 - AM243x/AM64x
     .endif
    ldi     r27.w0, 0x0101			; 32 bytes received, pointing to side b

; exit code
    xin     RXL2_SIDEA,&r2, 32
    xin     TM_YIELD_XID, &R0.b3,1  ; exit task after two instructions/cycles
    xout    xfr2vbus_wr0, &r2, 38	; dma to buffer
  .if $defined(PRU0)
    xin    BANK0_ID, &r0, 27*4     ; save R0-r27
 .else
    xin    BANK1_ID, &r0, 27*4     ; save R0-r27
 .endif
    halt                            ; should never happen


rx_b2:
; enter nrt next block after 32 bytes are received (1 byte ~ 2 cycles)
  .if $defined(PRU0)
    xout    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xout    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
;    add
    xin     RXL2_SIDEB, &r2, 32
    zero    &r10, 8                 ; clear R10 and R11
    ldi     r10.w0, DISCOV_RX_OFFSET; init start address
 .if $defined(AM65X)
    ldi     r10.w2, 0x0b20          ; host memory address of ICSS_G2 = 0x0b200000
 .else
    ldi     r10.w2, 0x3008          ; host memory address of ICSS_G1 = 0x030080000 - AM243x/AM64x
     .endif
    lsl     r12, r27.b0, 5          ; block counter *32
    add     r10, r10, r12
    ldi     r27.w0, 0002			; 32 bytes received, pointing to side a

    xin     TM_YIELD_XID, &R0.b3,1
    xout    xfr2vbus_wr0, &r2, 38  ; dma to buffer
  .if $defined(PRU0)
    xin    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xin    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
   halt                            ; should never happen


rx_nb:
; enter nrt next block after 29 bytes are received (1 byte ~ 2 cycles)
 .if $defined(PRU0)
    xout    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xout    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
    qbbs    rx_nb_side_b, r27.b1, 0 ; process side B
    xin     RXL2_SIDEA, &r2, 32
    ldi     r27.b1, 1				; change side pointer
    qba     rx_nb_cont
rx_nb_side_b:
    xin     RXL2_SIDEB, &r2, 32
    ldi     r27.b1, 0				; change side pointer
rx_nb_cont:

    zero    &r10, 8                 ; clear R10 and R11
    ldi     r10.w0, DISCOV_RX_OFFSET; init start address
 .if $defined(AM65X)
    ldi     r10.w2, 0x0b20          ; host memory address of ICSS_G2 = 0x0b200000
 .else
    ldi     r10.w2, 0x3008          ; host memory address of ICSS_G1 = 0x030080000 - AM243x/AM64x
     .endif
    lsl     r12, r27.b0, 5          ; block counter *32
    add     r10, r10, r12
    add     r27.b0, r27.b0, 1		; increment block counter
    xin     TM_YIELD_XID, &R0.b3,1
    xout    xfr2vbus_wr0, &r2, 38  ; dma to buffer
  .if $defined(PRU0)
    xin    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xin    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
   halt                            ; should never happen

	
rx_eof:
; enter nrt last block after RX_EOF (1 byte ~ 2 cycles)
  .if $defined(PRU0)
    xout    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xout    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
    qbbs    rx_lb_side_b, r27.b1, 0 ; process side B
    xin     RXL2_SIDEA, &r2, 32
    qba     rx_lb_cont
rx_lb_side_b:
    xin     RXL2_SIDEB, &r2, 32
    ldi     r27.b1, 0				; change side pointer
rx_lb_cont:
    zero    &r10, 8                 ; clear R10 and R11
    ldi     r10.w0, DISCOV_RX_OFFSET; init start address
 .if $defined(AM65X)
    ldi     r10.w2, 0x0b20          ; host memory address of ICSS_G2 = 0x0b200000
 .else
    ldi     r10.w2, 0x3008          ; host memory address of ICSS_G1 = 0x030080000 - AM243x/AM64x
     .endif
    lsl     r12, r27.b0, 5          ; block counter *32
    add     r10, r10, r12
;reset RX (Bit 18) and clear EOF(Bit 22), rx_error_clr (23)
    or      r31.b2, r31.b2, 0xF4
; clear r27 to indicate eof to main task
    ldi     r27, 0
	
    xin     TM_YIELD_XID, &R0.b3,1
    xout    xfr2vbus_wr0, &r2, 38  ; dma to buffer
 .if $defined(PRU0)
    xin    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xin    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
	nop
	halt

; **************** RXIO taks *****************
; new scheme to wait for eof task which assumes payload of device is <= 32 bytes 
; 

rxio_fb:
; enter nrt first block after 32 bytes are received
 .if $defined(PRU0)
    xout    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xout    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
    zero    &r10, 8                 ; clear R10 and R11
    ldi     r10.w0, IN_DATA_OFFSET  ; init start address
 .if $defined(AM65X)
    ldi     r10.w2, 0x0b20          ; host memory address of ICSS_G2 = 0x0b200000
 .else 
    ldi     r10.w2, 0x3008          ; host memory address of ICSS_G1 = 0x030080000 - AM243x/AM64x
 .endif
    add     r10.w0,r10.w0,r27.w0	; 32 bytes received, pointing to side b

; exit code
    xin     RXL2_SIDEA,&r18,1
    xin     RXL2_SIDEA,&r2,32
    ldi		R31.w2, (D_RESET_RXFIFO | D_RX_ERROR_CLEAR)
    add     r27.w0,r27.w0, 12
    xin     TM_YIELD_XID, &R0.b3,1  ; exit task after two instructions/cycles
    xout    xfr2vbus_wr0, &r2, 38	; dma to buffer
  .if $defined(PRU0)
    xin    BANK0_ID, &r0, 27*4     ; save R0-r27
 .else
    xin    BANK1_ID, &r0, 27*4     ; save R0-r27
 .endif
    halt                            ; should never happen

rxio_b2:
; enter nrt next block after 32 bytes are received (1 byte ~ 2 cycles)
  .if $defined(PRU0)
    xout    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xout    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
;    add
    xin     RXL2_SIDEB, &r2, 32
    zero    &r10, 8                 ; clear R10 and R11
    ldi     r10.w0, IN_DATA_OFFSET  ; init start address
 .if $defined(AM65X)
    ldi     r10.w2, 0x0b20          ; host memory address of ICSS_G2 = 0x0b200000
 .else 
    ldi     r10.w2, 0x3008          ; host memory address of ICSS_G1 = 0x030080000 - AM243x/AM64x
     .endif
    lsl     r12, r27.b0, 5          ; block counter *32
    add     r10, r10, r12
    ldi     r27.w0, 0002			; 32 bytes received, pointing to side a

    xin     TM_YIELD_XID, &R0.b3,1
    xout    xfr2vbus_wr0, &r2, 38  ; dma to buffer
  .if $defined(PRU0)
    xin    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xin    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
   halt                            ; should never happen


rxio_nb:
; enter nrt next block 
 .if $defined(PRU0)
    xout    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xout    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
    qbbs    rxio_nb_side_b, r27.b1, 0 ; process side B
    xin     RXL2_SIDEA, &r2, 32
    ldi     r27.b1, 1				; change side pointer
    qba     rxio_nb_cont
rxio_nb_side_b:
    xin     RXL2_SIDEB, &r2, 32
    ldi     r27.b1, 0				; change side pointer
rxio_nb_cont:

    zero    &r10, 8                 ; clear R10 and R11
    ldi     r10.w0, DISCOV_RX_OFFSET; init start address
 .if $defined(AM65X)
    ldi     r10.w2, 0x0b20          ; host memory address of ICSS_G2 = 0x0b200000
 .else 
    ldi     r10.w2, 0x3008          ; host memory address of ICSS_G1 = 0x030080000 - AM243x/AM64x
     .endif
    lsl     r12, r27.b0, 5          ; block counter *32
    add     r10, r10, r12
    add     r27.b0, r27.b0, 1		; increment block counter
    xin     TM_YIELD_XID, &R0.b3,1
    xout    xfr2vbus_wr0, &r2, 38  ; dma to buffer
  .if $defined(PRU0)
    xin    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xin    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif
   halt                            ; should never happen

rxio_eof:
; assumes max frame size of SORTE_G device <=32 bytes
 .if $defined(PRU0)
    xout    BANK0_ID, &r0, 27*4     ; save R0-r26
 .else
    xout    BANK1_ID, &r0, 27*4     ; save R0-r26
 .endif

    xin     RXL2_SIDEA,&r2,68
; swap sida and b for 64 bytes
;   xout    RXL2 -> scratch
;   xin     RXL2_SIDEB,&r2,68
    ldi		R31.w2, (D_RESET_RXFIFO | D_RX_ERROR_CLEAR)

    ldi     r10.w0, IN_DATA_OFFSET  ; init start address
    add     r10.w0,r10.w0,r27.w0	
	add     r0.b0, r18.b0, 4        ; add 4 bytes for RX time stamp
    add     r1.b0, r18.b0, 8        ; pointer for mvid starting from r2 , i.e. +8
; debug: read receive time stamp
    lbco	&r26, ICSS_IEP_CONST, ICSS_IEP_CAPR0_REG, 4
    mvid    *r1.b0, r26	
    sbco    &r2, ICSS_DMEM0_CONST, r10.w0, b0
; debug: write time stamp to pa_stat
    ldi32   r2, 0x00026178
    lsr     r26, r26, 3
    sbbo    &r26, r2, 0, 4

; todo: error handling

    add     r27.w0,r27.w0, r0.b0
    xin     TM_YIELD_XID, &R0.b3,1  ; exit task after two instructions/cycles
    nop     
  .if $defined(PRU0)
    xin    BANK0_ID, &r0, 27*4     ; save R0-r27
 .else
    xin    BANK1_ID, &r0, 27*4     ; save R0-r27
 .endif
    halt                           ; should never happen
	

	
