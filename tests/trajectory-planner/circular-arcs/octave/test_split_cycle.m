%% Estimate time remaining in a motion segment given some assumptions.
% From time 0 to dt_1, the TP tries to reach the final velocity, accelerating at max acceleration to do so.
% From time dt_1 onwards, the TP accelerates at max acceleration, then
%    decelerates back to final velocity, such that it reaches the end position
%    just when it also reaches the final velocity. This handles the case where the
%    TP reaches vf with some distance to spare in region 1.
% TODO: handle case where final velocity is lower than current velocity by accelerating to peak first
vc = 1.0;
vf = 0.5;
a_sign = sign(vf-vc);
dx = logspace(-3,-1,200);


a_max = 10.0;

t_step = 0.001;

dt_cutoff = abs(vf-vc) / a_max;

dx_1 = abs(vc.^2 - vf.^2) ./ (2.0 * a_max);

dx_2 = dx-dx_1;

for k = 1:length(dx_2)
    if dx_2(k) > 0
        % Case where we can accelerate to meet vf and still have distance left.
        % Assumes that the TP accelerates to peak velocity, then back down to final velocity
        A=a_max/4;
        B=vf;
        C=-dx_2(k);

        disc = sqrt(B^2-4*A*C);
        dt_2_a=(-B+disc)/(2*A);
        dt_2_b=(-B-disc)/(2*A);
        dt_2(k) = max(dt_2_a, dt_2_b);

        dt_1(k) = dt_cutoff;
    else
        dt_2(k) = 0;
        A=a_sign*a_max/2;
        B=vc;
        C=-dx(k);

        disc = sqrt(B^2-4*A*C);

        dt_1_a(k)=(-B+disc)/(2*A);
        dt_1_b(k)=(-B-disc)/(2*A);
        if dt_1_a(k) < 0
            dt_1(k) = dt_1_b(k);
        elseif dt_1_b < 0 
            dt_1(k) = dt_1_a(k);
        else
            dt_1(k) = min(dt_1_a(k), dt_1_b(k));
        end
    end
end

dt_2flat = max(dx_2,0) ./ vf;
plot(dx,dt_1,dx,dt_2,dx,dt_1+dt_2,'--')
grid on
legend('dt velocity match','dt position match','total time','no reaccel')
xlabel('net distance')
ylabel('time, seconds')
