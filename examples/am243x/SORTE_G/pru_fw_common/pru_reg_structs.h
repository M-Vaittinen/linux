;************************************************************************************
;**+------------------------------------------------------------------------------+**
;**|                              ******                                          |**
;**|                              ******     o                                    |**
;**|                              *******__////__****                             |**
;**|                              ***** /_ //___/ ***                             |**
;**|                           ********* ////__ ******                            |**
;**|                             *******(_____/ ******                            |**
;**|                                 **********                                   |**
;**|                                   ******                                     |**
;**|                                      ***                                     |**
;**|                                                                              |**
;**|            Copyright (c) 2017 Texas Instruments Incorporated                 |**
;**|                           ALL RIGHTS RESERVED                                |**
;**|                                                                              |**
;**|    Permission is hereby granted to licensees of Texas Instruments            |**
;**|    Incorporated (TI) products to use this computer program for the sole      |**
;**|    purpose of implementing a licensee product based on TI products.          |**
;**|    No other rights to reproduce, use, or disseminate this computer           |**
;**|    program, whether in part or in whole, are granted.                        |**
;**|                                                                              |**
;**|    TI makes no representation or warranties with respect to the              |**
;**|    performance of this computer program, and specifically disclaims          |**
;**|    any responsibility for any damages, special or consequential,             |**
;**|    connected with the use of this program.                                   |**
;**|                                                                              |**
;**+------------------------------------------------------------------------------+**
;************************************************************************************
; file:     pru_reg_structs.h
;
; brief:    Register Structs
;           Includes:
;           1. 
;

    .if !$isdefed("____pru_reg_structs_h")
____pru_reg_structs_h   .set    1




;--------------------------------------------------------------------------;
;   Struct: Struct_1_Reg
;
;   Struct to expose 1 register as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_1_Reg .struct
reg1 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
    .endstruct

;--------------------------------------------------------------------------;
;   Struct: Struct_2_Reg
;
;   Struct to expose 2 registers as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_2_Reg .struct
reg1 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg2 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
    .endstruct

;--------------------------------------------------------------------------;
;   Struct: Struct_3_Reg
;
;   Struct to expose 3 registers as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_3_Reg .struct
reg1 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg2 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg3 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
    .endstruct

;--------------------------------------------------------------------------;
;   Struct: Struct_4_Reg
;
;   Struct to expose 4 registers as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_4_Reg .struct
reg1 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg2 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg3 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg4 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
    .endstruct

;--------------------------------------------------------------------------;
;   Struct: Struct_5_Reg
;
;   Struct to expose 5 registers as dword, words and bytes
;
;--------------------------------------------------------------------------;
Struct_5_Reg .struct
reg1 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg2 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg3 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg4 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
reg5 .union
x .uint
x_w .struct
w0 .ushort
w2 .ushort
    .endstruct
x_b .struct
b0 .ubyte
b1 .ubyte
b2 .ubyte
b3 .ubyte
    .endstruct
    .endunion
    .endstruct


    .endif ; ____pru_reg_structs_h