% Find optimal maximum velocity and acceleration ratio for circular motions. 
% This shows the transition from velocity-limited to acceleration-limited motions
% Circles with large radii relative to their max velocity can use more of their
% acceleration budget for tangential acceleration, since normal accelerations are lower. 
% 
% NOTE: this calculation currently does not account for helical motion (which increases the effective radius)
a_max = 10.0
v_max = 1.0

a_t_max_nominal = 0.5 * a_max;
a_n_max_nominal = sqrt(1.0 - a_t_max_nominal^2);

r_eff = logspace(-2,.1,1000);

v_max_cutoff = sqrt(a_max*sqrt(3)/2*r_eff);
v_max_req = 0*r_eff + v_max;

figure(1)
plot(r_eff,v_max_cutoff,r_eff, v_max_req)
grid on
title('Circle maximum velocity vs radius')
xlabel('radius, units')
ylabel('velocity, units / sec')

a_n_max_cutoff = v_max^2 ./ r_eff;
a_t_max_cutoff = sqrt(a_max^2 - a_n_max_cutoff.^2);

figure(2)
plot(r_eff, a_t_max_cutoff / a_max,r_eff, r_eff*0+a_t_max_nominal/ a_max)
grid on
title('Circle maximum tangential acceleration vs radius')
xlabel('radius, units')
ylabel('velocity, units / sec')

