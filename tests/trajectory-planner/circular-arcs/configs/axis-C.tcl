set cv_id 5
set ca_id 15

# Add axis components to servo thread
addf wcomp.$cv_id servo-thread
addf wcomp.$ca_id servo-thread
addf minmax.$cv_id servo-thread
addf minmax.$ca_id servo-thread
addf d2dt2.$cv_id servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net c_pos axis.$cv_id.motor-pos-cmd => axis.$cv_id.motor-pos-fb d2dt2.$cv_id.in

# send the position commands thru differentiators to
# generate velocity and accel signals
net c_vel <= d2dt2.$cv_id.out1 => wcomp.$cv_id.in  minmax.$cv_id.in
net c_acc <= d2dt2.$cv_id.out2 => wcomp.$ca_id.in  minmax.$ca_id.in

#Conservative limits
set acc_limit 1.0001
set vel_limit 1.01

setp wcomp.$ca_id.max $::AXIS_5(MAX_ACCELERATION)*$acc_limit
setp wcomp.$ca_id.min $::AXIS_5(MAX_ACCELERATION)*-1.0*$acc_limit
setp wcomp.$cv_id.max $::AXIS_5(MAX_VELOCITY)*$vel_limit
setp wcomp.$cv_id.min $::AXIS_5(MAX_VELOCITY)*-1.0*$vel_limit

net c_acc-ok <= wcomp.$cv_id.out => match_abc.a4
net c_vel-ok <= wcomp.$ca_id.out => match_abc.a5

# Enable match_all pins for axis
setp match_abc.b4 1
setp match_abc.b5 1

