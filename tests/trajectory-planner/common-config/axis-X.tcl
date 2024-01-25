set xv_id 0
set xa_id 10

# Add axis components to servo thread
addf wcomp.$xv_id servo-thread
addf wcomp.$xa_id servo-thread
addf minmax.$xv_id servo-thread
addf minmax.$xa_id servo-thread
addf d2dt2.$xv_id servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net x_pos axis.$xv_id.motor-pos-cmd => axis.$xv_id.motor-pos-fb d2dt2.$xv_id.in

# send the position commands thru differentiators to
# generate velocity and accel signals
net x_vel <= d2dt2.$xv_id.out1 => vel_xyz.in$xv_id wcomp.$xv_id.in  minmax.$xv_id.in
net x_acc <= d2dt2.$xv_id.out2 => wcomp.$xa_id.in  minmax.$xa_id.in

#Conservative limits
set acc_limit 1.0001
set vel_limit 1.01

setp wcomp.$xa_id.max $::AXIS_0(MAX_ACCELERATION)*$acc_limit
setp wcomp.$xa_id.min $::AXIS_0(MAX_ACCELERATION)*-1.0*$acc_limit
setp wcomp.$xv_id.max $::AXIS_0(MAX_VELOCITY)*$vel_limit
setp wcomp.$xv_id.min $::AXIS_0(MAX_VELOCITY)*-1.0*$vel_limit

net x_acc-ok <= wcomp.$xv_id.out => match_xyz.a0
net x_vel-ok <= wcomp.$xa_id.out => match_xyz.a1

# Enable match_all pins for axis
setp match_xyz.b0 1
setp match_xyz.b1 1

