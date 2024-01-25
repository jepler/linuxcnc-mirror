#/bin/bash
cd ../../../src
#Ugly way to force rebuild of kinematics, which assumes that tp_debug isn't
#used anywhere else...
touch emc/tp/tp_debug.h
MAKECMD="make -j4 "
# Show debug info for each timestep, as well as segment creation
#$MAKECMD EXTRA_DEBUG='-DTC_DEBUG -DTP_DEBUG'
# Show debugging info for segment creation and optimization
#make EXTRA_DEBUG='-DTP_DEBUG -DTP_INFO_LOGGING'
$MAKECMD EXTRA_DEBUG='-DTC_DEBUG -DTP_DEBUG -DTP_INFO_LOGGING'
#make EXTRA_DEBUG='-DTP_DEBUG'
#make EXTRA_DEBUG='-DTC_DEBUG'
