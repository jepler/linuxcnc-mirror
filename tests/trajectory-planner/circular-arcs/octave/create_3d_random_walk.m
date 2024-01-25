%% Create a random path for G code testing.
%  Produces random lengths, and random 3d bends, constrained by the provided settings. Writes a unique file name to the nc_files directory 
% 
%    num_moves: number of moves in the generated file
%    v_range: interval [min, max] of desired feeds (each move gets a randomly selected feed in this range)
%    dp: length of moves (can be a min / max interval instead of a single value)
%    limits: specify a bounding box for the program in a 6-element array [-X,+X,-Y,+Y,-Z,+Z] (default +/- 10.0 in XY, 5.0 in Z)
%    tol: blend tolerance in user units (default 0.005)
%    ncd_tol: naivecam tolerance (default 0)
function create_3d_random_walk(num_moves, v_range, dp, theta, limits, tol, ncd_tol, file_path, file_prefix)
    if ~exist('file_path','var')
        file_path = '../nc_files/generated'
    end
    mkdir(file_path); 
    if ~exist('file_prefix','var')
        file_prefix = 'random_walk_linear'
    end
   
    if ~exist('limits','var') || min(size(limits)) < 1
        limits=[-10,10,-10,10,-5,5]
    end
    if ~exist('tol','var')
        tol = 0.005
    end
    if ~exist('ncd_tol','var')
        ncd_tol = 0.0
    end

    tag = datestr(now(),"YYYY-MM-DD_HH-MM");
    fsuf=sprintf('%dsteps_%gin-%gin_%gips_%gdeg_max_angle_%s.ngc',num_moves,dp(1),dp(end),v_range(end),max(theta)*180/pi,tag); 
    fid=fopen(sprintf('%s/%s_%s', file_path, file_prefix, fsuf),'w');
    if fid < 1
      error('Got invalid file handle')
    end
      
    %Initial state
    header= sprintf('G61 G90\n G0 X-.1 Y0 Z0\n G1 X0 Y0 Z0 F20\nG64 P%f Q%f\n', tol, ncd_tol);
    fprintf(fid, header);
    move=create_3d_line(v_range, dp, theta,[],limits);
    for k=1:num_moves
        move_prev=move;
        move=create_3d_line(v_range, dp, theta, move_prev, limits);
        fprintf(fid,'G1 X%f Y%f Z%f F%f\n', move_prev.P2(1), move_prev.P2(2), move.P2(3), move_prev.v_req*60);
    end
    fprintf(fid,'M30\n');
    fclose(fid);

end

function move=create_3d_line(v_range,dp,theta,prev_move,limits)
    %% Create a random trajectory segment based on the input

    if ~isstruct(prev_move)
        %wlog assume +X as initial direction and origin as start
        prev_move.P1=[-1,0,0];
        prev_move.P2=[0,0,0];
    end

        move.P1=prev_move.P2;
    
    %Hard-coded settings for G-code dump, make this more general later if need be
    if length(v_range)==2
        move.v_req=ezrand(v_range(1),v_range(2));
    else
        move.v_req=v_range;
    end

    %dummy value
    move.a_max=1;

    move.tolerance=10;

    dp_old=prev_move.P2-prev_move.P1;
    u_old=dp_old/norm(dp_old);

    if length(theta)==1
        theta_vec=[-theta,theta,-theta,theta,-theta,theta];
    elseif length(theta)==2
        theta_vec=[theta,theta,theta];
        theta_vec=theta_vec(:);
    else
        theta_vec=theta;
    end

    inlimits = false;
    iter = 0;
    while ~inlimits
        %Rotate about Z 
        theta_z=rand(1)*(theta_vec(2)-theta_vec(1)) + theta_vec(1);
        u_raw=u_old*[cos(theta_z),-sin(theta_z),0;sin(theta_z),cos(theta_z),0;0,0,1];

        %rotate about Y
        theta_y=rand(1)*(theta_vec(4)-theta_vec(3)) + theta_vec(3);
        u_raw=u_raw*[cos(theta_y),0,-sin(theta_y);0,1,0;sin(theta_y),0,cos(theta_y)];

        theta_x=rand(1)*(theta_vec(6)-theta_vec(5)) + theta_vec(5);
        u_raw=u_raw*[1, 0, 0;0, cos(theta_x),sin(theta_x);0,-sin(theta_x),cos(theta_x)];

        %Hack to make a minimum length move
        if length(dp) == 1
            move.P2=move.P1+u_raw*dp;
        else
            move.P2=move.P1+u_raw*(rand(1)*(dp(2)-dp(1))+dp(1));
        end

        if move.P2(1)<limits(2) && move.P2(1)>limits(1) && move.P2(2)<limits(4) && move.P2(2)>limits(3) && move.P2(3)<limits(6) && move.P2(3)>limits(5)
            inlimits = true;
        end
        iter = iter + 1;
        if iter > 100
          error('Exceeded max iterations trying to fit random move within limits, try a smaller length');
        end
    end
end

