obj-m += bd71828-test.o bd71828-gpio-test.o bd71828-clktest.o

all: bd71828-test dtb
bd71828-test:
	make -C ${KERNEL_DIR} CROSS_COMPILE=${CC} ARCH=arm M=$(PWD) modules
dtb: bd71828_test.dts
	dtc -O dtb -o bd71828_test.dtbo -b 0 -@ bd71828_test.dts

debug_cfg:
	make -C ${KERNEL_DIR} CROSS_COMPILE=${CC} ARCH=arm M=$(PWD) CONFIG_DEBUG_SECTION_MISMATCH=y modules
