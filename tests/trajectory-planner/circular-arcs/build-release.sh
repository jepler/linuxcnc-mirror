#/bin/bash
cd ../../../src
#Ugly way to force rebuild of kinematics, which assumes that tp_debug isn't
#used anywhere else...
touch emc/tp/tp_debug.h
make -j12 EXTRA_DEBUG=''
