#!/bin/bash

OWN_ADDRESSES='Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>, Matti Vaittinen <mazziesaccount@gmail.com>'

# These color/format variables were stolen from the webs:
# https://defragged.org/2019/12/28/how-do-you-add-color-or-bold-the-echo-outputs-in-a-shell-script/
# (Not sure if this is the original author though)

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

BOLD=$(tput bold)
NORM=$(tput sgr0)

BASE_FOLDER="${BASH_ARGV[0]}"
ALL_RECIPIENTS=""
OK="a"

function print_help() {
	echo ""
	echo -e "${BOLD}Usage:${NORM} ${GREEN}$BASE_FOLDER/format-patch.sh <arguments>${NC}"
	echo "	Arguments are same as would be for git format-patch excluding the recipients and --thread"
	echo "	by minimum this is the base commmit/tag/branch"
	echo ""
	echo "${BOLD}typical examples:${NORM}"
	echo "	$BASE_FOLDER/format-patch.sh -v2 --base=v6.1-rc3 v6.1-rc3"
	echo "	$BASE_FOLDER/format-patch.sh --cover-letter --rfc v6.0"

}

if [[ $1 = "" ]] || [[ $1 = "--help" ]] || [[ $1 = "-h" ]] || [[ $1 = "-?" ]]
then
	print_help
	exit 0
fi

echo "Try creating patch dir"
mkdir tmp_patch_dir || exit -1

echo "dir created"
echo "base=$BASE_FOLDER"

git format-patch -o tmp_patch_dir -q --to="$OWN_ADDRESSES" --cc=__KASTANNETAYTTELEVASTAANOTTAGATELICCMIEHET__ --thread "$@"

echo "patches formatted"

for f in tmp_patch_dir/*
do
	printf "Patch ${GREEN} $f${NC}:\n"

	if [ $OK != "N" ]
	then
		printf "${BOLD}Checkpatch this?${NORM}\n${RED}a${NC}=abort, ${GREEN}y${NC}=YES, ${RED}n${NC}=NO, others=skip this patch ${RED}N${NC}=don't chack any of the patches "
		read -p"?" OK
		echo ""
	fi

	if [ $OK = "a" ]
	then
		exit 1
	fi

	if [ $OK = "y" ]
	then
		scripts/checkpatch.pl "$f"
	else
		if [ $OK != "n" ] && [ $OK != "N" ]
		then
			continue;
		fi
	fi

	case "$f" in
 		*cover-letter.patch)
			COVER_FILE="$f"
			echo "cover-letter, skip recipients for now..."
			continue ;;
	esac

	RECIPIENTS=$(scripts/get_maintainer.pl --separator=, --no-rolestats $f)
	ALL_RECIPIENTS="$RECIPIENTS,$ALL_RECIPIENTS"

	echo "--cc='$RECIPIENTS'"
	printf "${BOLD}Recipients Ok?${NORM}\n${GREEN}y${NC}=proceed, ${RED}n${NC}=abort, others=Don't add recipients "
	read -p"?" KO
	echo ""
	if [ $KO = "n" ]
	then
		exit 1
	fi
	if [ $KO != "y" ]
	then
		continue;
	fi
	sed -i "s/__KASTANNETAYTTELEVASTAANOTTAGATELICCMIEHET__/$RECIPIENTS/" $f
done

declare -A ARRAY
IFS=','
for w in  $ALL_RECIPIENTS; do
	ARRAY+=( [$w]="" )
done

IFS=''

FINAL_LIST=""

for I in "${!ARRAY[@]}"
do
  FINAL_LIST="$I,$FINAL_LIST"
done

if [ "$FINAL_LIST" == "" ]
then
	exit 0
fi

echo "$FINAL_LIST"
printf " is this ok for cover-letter CC-list? ${RED}y${NC}=yes ${RED}n${NC}=Don't add CC to cover-letter "
read -p"?" OK

if [ $OK != "y" ]
then
	exit 0;
fi
sed -i "s/__KASTANNETAYTTELEVASTAANOTTAGATELICCMIEHET__/$FINAL_LIST/" "$COVER_FILE"

