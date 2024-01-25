set yv_id 1
set ya_id 11

# Add axis components to servo thread
addf wcomp.$yv_id servo-thread
addf wcomp.$ya_id servo-thread
addf minmax.$yv_id servo-thread
addf minmax.$ya_id servo-thread
addf d2dt2.$yv_id servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net y_pos axis.$yv_id.motor-pos-cmd => axis.$yv_id.motor-pos-fb d2dt2.$yv_id.in

# send the position commands thru differentiators to
# generate velocity and accel signals
net y_vel <= d2dt2.$yv_id.out1 => vel_xyz.in$yv_id wcomp.$yv_id.in  minmax.$yv_id.in
net y_acc <= d2dt2.$yv_id.out2 => wcomp.$ya_id.in  minmax.$ya_id.in

#Conservative limits
set acc_limit 1.0001
set vel_limit 1.01

setp wcomp.$ya_id.max $::AXIS_1(MAX_ACCELERATION)*$acc_limit
setp wcomp.$ya_id.min $::AXIS_1(MAX_ACCELERATION)*-1.0*$acc_limit
setp wcomp.$yv_id.max $::AXIS_1(MAX_VELOCITY)*$vel_limit
setp wcomp.$yv_id.min $::AXIS_1(MAX_VELOCITY)*-1.0*$vel_limit

net y_acc-ok <= wcomp.$yv_id.out => match_xyz.a2
net y_vel-ok <= wcomp.$ya_id.out => match_xyz.a3

# Enable match_all pins for axis
setp match_xyz.b2 1
setp match_xyz.b3 1

