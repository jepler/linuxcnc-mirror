set av_id 3
set aa_id 13

# Add axis components to servo thread
addf wcomp.$av_id servo-thread
addf wcomp.$aa_id servo-thread
addf minmax.$av_id servo-thread
addf minmax.$aa_id servo-thread
addf d2dt2.$av_id servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net a_pos axis.$av_id.motor-pos-cmd => axis.$av_id.motor-pos-fb d2dt2.$av_id.in

# send the position commands thru differentiators to
# generate velocity and accel signals
net a_vel <= d2dt2.$av_id.out1 => wcomp.$av_id.in  minmax.$av_id.in
net a_acc <= d2dt2.$av_id.out2 => wcomp.$aa_id.in  minmax.$aa_id.in

#Conservative limits
set acc_limit 1.0001
set vel_limit 1.01

setp wcomp.$aa_id.max $::AXIS_3(MAX_ACCELERATION)*$acc_limit
setp wcomp.$aa_id.min $::AXIS_3(MAX_ACCELERATION)*-1.0*$acc_limit
setp wcomp.$av_id.max $::AXIS_3(MAX_VELOCITY)*$vel_limit
setp wcomp.$av_id.min $::AXIS_3(MAX_VELOCITY)*-1.0*$vel_limit

net a_acc-ok <= wcomp.$av_id.out => match_abc.a0
net a_vel-ok <= wcomp.$aa_id.out => match_abc.a1

# Enable match_all pins for axis
setp match_abc.b0 1
setp match_abc.b1 1

