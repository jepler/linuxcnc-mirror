set zv_id 2
set za_id 12

# Add axis components to servo thread
addf wcomp.$zv_id servo-thread
addf wcomp.$za_id servo-thread
addf minmax.$zv_id servo-thread
addf minmax.$za_id servo-thread
addf d2dt2.$zv_id servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net z_pos axis.$zv_id.motor-pos-cmd => axis.$zv_id.motor-pos-fb d2dt2.$zv_id.in

# send the position commands thru differentiators to
# generate velocity and accel signals
net z_vel <= d2dt2.$zv_id.out1 => vel_xyz.in$zv_id wcomp.$zv_id.in  minmax.$zv_id.in
net z_acc <= d2dt2.$zv_id.out2 => wcomp.$za_id.in  minmax.$za_id.in

#Conservative limits
set acc_limit 1.0001
set vel_limit 1.01

setp wcomp.$za_id.max $::AXIS_2(MAX_ACCELERATION)*$acc_limit
setp wcomp.$za_id.min $::AXIS_2(MAX_ACCELERATION)*-1.0*$acc_limit
setp wcomp.$zv_id.max $::AXIS_2(MAX_VELOCITY)*$vel_limit
setp wcomp.$zv_id.min $::AXIS_2(MAX_VELOCITY)*-1.0*$vel_limit

net z_acc-ok <= wcomp.$zv_id.out => match_xyz.a4
net z_vel-ok <= wcomp.$za_id.out => match_xyz.a5

# Enable match_all pins for axis
setp match_xyz.b4 1
setp match_xyz.b5 1
