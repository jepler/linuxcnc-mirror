#!/usr/bin/env python

import linuxcnc, hal
import sys, os, time


# Params
spindle_speed = 2000 # RPM
z_axis_maxvel = 4 * 60 # IPM
# Pitch at which Z axis must travel at double maxvel if spindle speed
# is constant
max_pitch = float(z_axis_maxvel) / float(spindle_speed) * 2


# Set up component
h = hal.component("python-ui")
# - Monitor spindle and Z axis speed for debugging
h.newpin("spindle-speed", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("Zvel", hal.HAL_FLOAT, hal.HAL_IN)
# - Estimated pitch
h.newpin("pitch", hal.HAL_FLOAT, hal.HAL_IN)
# - Signal program start/stop
h.newpin("running", hal.HAL_BIT, hal.HAL_IN)
h.ready() # mark the component as 'ready'
os.system("halcmd source ./postgui.hal")


# Initialization
c = linuxcnc.command()
s = linuxcnc.stat()
c.state(linuxcnc.STATE_ESTOP_RESET)
c.state(linuxcnc.STATE_ON)
c.mode(linuxcnc.MODE_MDI)

# Spindle:  set speed and start
c.mdi('S%d M3' % spindle_speed)
    
# Spindle synchronized motion at 2x Z axis maxvel
c.mdi('G33 Z-5 K%.3f' % max_pitch)

# While above MDI command runs, take an average of pitch while the Z
# axis is in motion
running = True
timeout = 5
avg_sum = 0; avg_count = 0
while timeout > 0:
    if abs(h['pitch']) > 0.001:
        rps = h['spindle-speed']/60
        ips = - h['Zvel']
        pitch = ips/rps
        print "spindle velocity, revs/second: %.3f" % rps
        print "Z velocity: %.3f" % ips
        print "pitch: %.3f" % pitch
        print
        avg_sum += pitch
        avg_count += 1
    timeout = 5 if h['running'] else timeout - 1
    time.sleep(0.1)
avg_pitch = avg_sum/avg_count
print "Average pitch = %.3f" % avg_pitch

# Pitch should either be as specified (by slowing down spindle) or
# else program should have aborted; if it didn't abort and pitch is
# incorrect, then return the special failure result 166
if (abs(avg_pitch - max_pitch) < 0.01):
    res = 0
    print "OK:  average pitch = %.3f" % avg_pitch
else:
    res = 166
    print "Error:  expected pitch = %.3f; got %.3f" % (max_pitch, avg_pitch)

# Shutdown
c.wait_complete()
sys.exit(res)
