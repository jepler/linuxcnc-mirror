#!/usr/bin/env bash
cp position.blank position.txt
linuxcnc batch-test.ini 2> batch-test-err.log | tee batch-test-out.log
exit $?
