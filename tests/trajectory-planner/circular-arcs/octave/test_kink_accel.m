% Test behavior of acceleration as TP hits a sharp corner.
% this demonstrates the momentary acceleration spike encountered due to a not-quite-tangent intersection
% The sharp corner, or "kink" requires a potentially large reduction in velocity to avoid violating overall limits.

a_bound=[30;30;30];
dt = 0.001;
v_max = 1.05;

u0=[0.900637; -0.434573; 0.000000]
u1=[-0.766314; 0.642466; 0.000000]

a_kink = v_max / dt * (u1-u0)

a_kink_violation = a_kink ./ a_bound;

accel_loss_ratio = max(abs(a_kink_violation(:)))

% How much segment acceleration to budget for the end point
allowed_acceleration_loss = 0.1;

if (accel_violation_factor > allowed_acceleration_loss)
  disp('Too much acceleration loss')
  kink_vel = v_max * allowed_acceleration_loss / accel_loss_ratio
else
  kink_vel = v_max * allowed_acceleration_loss / accel_loss_ratio
end



