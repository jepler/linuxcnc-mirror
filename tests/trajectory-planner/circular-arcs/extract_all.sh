#!/usr/bin/env bash
LOGFILE=${1-test.log}
echo "Extracting data from $LOGFILE"
for f in spindle tc_state displacement
do
	awk -f extract_"$f".awk $LOGFILE > ./octave/"$f"_data.txt
done
