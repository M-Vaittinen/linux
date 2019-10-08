obj-m += mva_overlay.o


all:
	make -C ${KERNEL_DIR} CROSS_COMPILE=${CC} ARCH=arm M=$(PWD) C=2 modules

clean:
	make -C ${KERNEL_DIR} CROSS_COMPILE=${CC} ARCH=arm M=$(PWD) clean
install:
	cp mva_overlay.ko /home/mvaittin/nfs/.
