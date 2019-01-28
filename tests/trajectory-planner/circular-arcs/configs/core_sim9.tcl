# core HAL config file for simulation

# first load all the RT modules that will be needed
# kinematics
loadrt trivkins
# motion controller, get name and thread periods from ini file
loadrt [EMCMOT]EMCMOT servo_period_nsec=[EMCMOT]SERVO_PERIOD num_joints=[TRAJ]AXES
# load 6 differentiators (for velocity and accel signals

# add motion controller functions to servo thread
addf motion-command-handler servo-thread
addf motion-controller servo-thread

set next_ddt_idx = 0

# link the differentiator functions into the code
addf ddt_x servo-thread
addf ddt_xv servo-thread
addf ddt_y servo-thread
addf ddt_yv servo-thread
addf ddt_z servo-thread
addf ddt_zv servo-thread
addf vel_xy servo-thread
addf vel_xyz servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net Xpos axis.0.motor-pos-cmd => axis.0.motor-pos-fb ddt_x.in
net Ypos axis.1.motor-pos-cmd => axis.1.motor-pos-fb ddt_y.in
net Zpos axis.2.motor-pos-cmd => axis.2.motor-pos-fb ddt_z.in
net Apos axis.3.motor-pos-cmd => axis.3.motor-pos-fb
net Bpos axis.4.motor-pos-cmd => axis.4.motor-pos-fb
net Cpos axis.5.motor-pos-cmd => axis.5.motor-pos-fb
net Upos axis.6.motor-pos-cmd => axis.6.motor-pos-fb
net Vpos axis.7.motor-pos-cmd => axis.7.motor-pos-fb
net Wpos axis.8.motor-pos-cmd => axis.8.motor-pos-fb

# send the position commands thru differentiators to
# generate velocity and accel signals
net Xvel ddt_x.out => ddt_xv.in vel_xy.in0
net Xacc <= ddt_xv.out
net Yvel ddt_y.out => ddt_yv.in vel_xy.in1
net Yacc <= ddt_yv.out
net Zvel ddt_z.out => ddt_zv.in vel_xyz.in0
net Zacc <= ddt_zv.out

# Cartesian 2- and 3-axis velocities
net XYvel vel_xy.out => vel_xyz.in1
net XYZvel <= vel_xyz.out

# estop loopback
net estop-loop iocontrol.0.user-enable-out iocontrol.0.emc-enable-in

# create signals for tool loading loopback
net tool-prep-loop iocontrol.0.tool-prepare iocontrol.0.tool-prepared
net tool-change-loop iocontrol.0.tool-change iocontrol.0.tool-changed

net spindle-fwd motion.spindle-forward
net spindle-rev motion.spindle-reverse
#net spindle-speed motion.spindle-speed-out

net lube iocontrol.0.lube
net flood iocontrol.0.coolant-flood
net mist iocontrol.0.coolant-mist
