set bv_id 4
set ba_id 14

# Add axis components to servo thread
addf wcomp.$bv_id servo-thread
addf wcomp.$ba_id servo-thread
addf minmax.$bv_id servo-thread
addf minmax.$ba_id servo-thread
addf d2dt2.$bv_id servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net b_pos axis.$bv_id.motor-pos-cmd => axis.$bv_id.motor-pos-fb d2dt2.$bv_id.in

# send the position commands thru differentiators to
# generate velocity and accel signals
net b_vel <= d2dt2.$bv_id.out1 => wcomp.$bv_id.in  minmax.$bv_id.in
net b_acc <= d2dt2.$bv_id.out2 => wcomp.$ba_id.in  minmax.$ba_id.in

#Conservative limits
set acc_limit 1.0001
set vel_limit 1.01

setp wcomp.$ba_id.max $::AXIS_4(MAX_ACCELERATION)*$acc_limit
setp wcomp.$ba_id.min $::AXIS_4(MAX_ACCELERATION)*-1.0*$acc_limit
setp wcomp.$bv_id.max $::AXIS_4(MAX_VELOCITY)*$vel_limit
setp wcomp.$bv_id.min $::AXIS_4(MAX_VELOCITY)*-1.0*$vel_limit

net b_acc-ok <= wcomp.$bv_id.out => match_abc.a2
net b_vel-ok <= wcomp.$ba_id.out => match_abc.a3

# Enable match_all pins for axis
setp match_abc.b2 1
setp match_abc.b3 1

