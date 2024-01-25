set vv_id 7
set va_id 17

# Add axis components to servo thread
addf wcomp.$vv_id servo-thread
addf wcomp.$va_id servo-thread
addf minmax.$vv_id servo-thread
addf minmax.$va_id servo-thread
addf d2dt2.$vv_id servo-thread

# create HAL signals for position commands from motion module
# loop position commands back to motion module feedback
net v_pos axis.$vv_id.motor-pos-cmd => axis.$vv_id.motor-pos-fb d2dt2.$vv_id.in

# send the position commands thru differentiators to
# generate velocity and accel signals
net v_vel <= d2dt2.$vv_id.out1 => wcomp.$vv_id.in  minmax.$vv_id.in
net v_acc <= d2dt2.$vv_id.out2 => wcomp.$va_id.in  minmax.$va_id.in

#Conservative limits
set acc_limit 1.0001
set vel_limit 1.01

setp wcomp.$va_id.max $::AXIS_7(MAX_ACCELERATION)*$acc_limit
setp wcomp.$va_id.min $::AXIS_7(MAX_ACCELERATION)*-1.0*$acc_limit
setp wcomp.$vv_id.max $::AXIS_7(MAX_VELOCITY)*$vel_limit
setp wcomp.$vv_id.min $::AXIS_7(MAX_VELOCITY)*-1.0*$vel_limit

net v_acc-ok <= wcomp.$vv_id.out => match_uvw.a2
net v_vel-ok <= wcomp.$va_id.out => match_uvw.a3

# Enable match_all pins for axis
setp match_uvw.b2 1
setp match_uvw.b3 1

