#!/bin/sh 

# Finds duplicates based on this SO suggestion: https://stackoverflow.com/a/16278407
dirname=${1:-'./'}
find $dirname -type f | sed 's_.*/__' | sort|  uniq -d| 
while read fileName
do
find $dirname -type f | grep "$fileName"
done
