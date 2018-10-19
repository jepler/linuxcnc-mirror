#!/bin/bash

linuxcnc -r test.ini
res=$?
echo "Exit status $res" >&2
if test $res = 166; then
    echo "Test failed:  unexpected pitch" >&2
    exit 1
fi
exit 0
