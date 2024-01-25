set wv_id 8
set wa_id 18

# Add axis components to servo thread
addf wcomp.$wv_id servo-thread
addf wcomp.$wa_id servo-thread
addf minmax.$wv_id servo-thread
addf minmax.$wa_id servo-thread
addf d2dt2.$wv_id servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net w_pos axis.$wv_id.motor-pos-cmd => axis.$wv_id.motor-pos-fb d2dt2.$wv_id.in

# send the position commands thru differentiators to
# generate velocity and accel signals
net w_vel <= d2dt2.$wv_id.out1 => wcomp.$wv_id.in  minmax.$wv_id.in
net w_acc <= d2dt2.$wv_id.out2 => wcomp.$wa_id.in  minmax.$wa_id.in

#Conservative limits
set acc_limit 1.0001
set vel_limit 1.01

setp wcomp.$wa_id.max $::AXIS_8(MAX_ACCELERATION)*$acc_limit
setp wcomp.$wa_id.min $::AXIS_8(MAX_ACCELERATION)*-1.0*$acc_limit
setp wcomp.$wv_id.max $::AXIS_8(MAX_VELOCITY)*$vel_limit
setp wcomp.$wv_id.min $::AXIS_8(MAX_VELOCITY)*-1.0*$vel_limit

net w_acc-ok <= wcomp.$wv_id.out => match_uvw.a4
net w_vel-ok <= wcomp.$wa_id.out => match_uvw.a5

# Enable match_all pins for axis
setp match_uvw.b4 1
setp match_uvw.b5 1

