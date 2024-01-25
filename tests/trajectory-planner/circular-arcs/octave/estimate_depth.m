%% -- estimate_depth(a_limit, v_limit, L_min, dt)
%    
%   For the given a_limit (max acceleration), v_limit (max velocity), estimate the queue depth 
%   to reach peak velocity, for a series of worst-case moves of length L_min
function depth = estimate_depth(a_limit,v_limit,L_min,dt)
    v_sample= L_min/(2.0*dt);
    v_max = min(v_sample,v_limit);
    v = 0;
    depth = 0;
    while v<v_max
        v = sqrt(v^2 + 2*a_limit*L_min);
        depth+=1;
    end
end
