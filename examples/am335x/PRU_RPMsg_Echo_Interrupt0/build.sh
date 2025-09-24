#!/bin/bash

EXENAME=PRU_RPMsg_Echo_Interrupt0.out
NFSPATH=/home/mvaittin/nfs
FIRMWAREPATH="$NFSPATH/lib/firmware"

source /home/mvaittin/gits/pru-software-support-package/build.sh

if [[ $1 == "install" ]]
then
		BINNAME=PRU-adc-out-$(date "+%Y-%m-%d-%s%N")
		mv "gen/$EXENAME" "$FIRMWAREPATH/$BINNAME"
		cd "$FIRMWAREPATH"
		ln -sf "$BINNAME" "am335x-pru0-fw"
		exit 0
fi

echo "rm -rf gen/*"
rm -rf gen/*

make
