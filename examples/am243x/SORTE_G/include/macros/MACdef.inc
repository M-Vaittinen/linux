; defines
MAC_DEV_ID   .set	0
MAC_MODE_MPY .set	0
MAC_MODE_MAC .set	1


MAC_s .struct
ctlreg32 	.uint
resL32 		.uint
resH32 		.uint
op1_32 		.uint
op2_32 		.uint
sMAC_size	.endstruct

sMAC .sassign R25,MAC_s
	.asg  sMAC.ctlreg32 ,R25


; MACclear : Clear the carry bit and the accumulator

MACclear	.macro
			OR sMAC.ctlreg32,sMAC.ctlreg32,2
			XOUT MAC_DEV_ID,&sMAC.ctlreg32,1
			XOR sMAC.ctlreg32,sMAC.ctlreg32,2
			.endm

;MACsetmode : Set the MAC module in MPY or MAC mode

MACsetmode 	.macro mode
			LDI	sMAC.ctlreg32, mode
			xout MAC_DEV_ID, &sMAC.ctlreg32, 1
			.endm

