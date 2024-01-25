set uv_id 6
set ua_id 16

# Add axis components to servo thread
addf wcomp.$uv_id servo-thread
addf wcomp.$ua_id servo-thread
addf minmax.$uv_id servo-thread
addf minmax.$ua_id servo-thread
addf d2dt2.$uv_id servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net u_pos axis.$uv_id.motor-pos-cmd => axis.$uv_id.motor-pos-fb d2dt2.$uv_id.in

# send the position commands thru differentiators to
# generate velocity and accel signals
net u_vel <= d2dt2.$uv_id.out1 => wcomp.$uv_id.in  minmax.$uv_id.in
net u_acc <= d2dt2.$uv_id.out2 => wcomp.$ua_id.in  minmax.$ua_id.in

#Conservative limits
set acc_limit 1.0001
set vel_limit 1.01

setp wcomp.$ua_id.max $::AXIS_6(MAX_ACCELERATION)*$acc_limit
setp wcomp.$ua_id.min $::AXIS_6(MAX_ACCELERATION)*-1.0*$acc_limit
setp wcomp.$uv_id.max $::AXIS_6(MAX_VELOCITY)*$vel_limit
setp wcomp.$uv_id.min $::AXIS_6(MAX_VELOCITY)*-1.0*$vel_limit

net u_acc-ok <= wcomp.$uv_id.out => match_uvw.a0
net u_vel-ok <= wcomp.$ua_id.out => match_uvw.a1

# Enable match_all pins for axis
setp match_uvw.b0 1
setp match_uvw.b1 1

