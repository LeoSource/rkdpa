function [sim_pos,sim_q] = CleanMirror(rbt,via_pos,q0,dt)

    global g_cvmax g_stowed_pos g_camax
    sim_pos = []; sim_q = []; pos = [];
    % pre-clean action
    pos0 = rbt.FKSolve(q0).t;
    line_length = norm(via_pos(:,1)-pos0);
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = pos0+up.*(via_pos(:,1)-pos0)/line_length;
    pos = [pos, pos_tmp];
    % clean mirror action
    s = []; sv = []; sa = [];
    cpath = ArcTransPathPlanner(via_pos, 0);
    varc = sqrt(g_camax*cpath.radius);
    for idx=1:length(cpath.dis_interval)-1
        if mod(idx,2)==1
            if idx==1
                vel_cons = [0,varc];
            elseif idx==length(cpath.dis_interval)-1
                vel_cons = [varc,0];
            else
                vel_cons = [varc,varc];
            end
            splanner = LspbTrajPlanner([cpath.dis_interval(idx),cpath.dis_interval(idx+1)],g_cvmax,g_camax,[],vel_cons);
            [s_tmp,sv_tmp,sa_tmp] = splanner.GenerateTraj(dt);
        else
            t_len = (cpath.dis_interval(idx+1)-cpath.dis_interval(idx))/varc;
            num_interval = floor(t_len/dt+1);
            sa_tmp = zeros(1, num_interval);
            sv_tmp = ones(1, num_interval)*varc;
            s_tmp = linspace(cpath.dis_interval(idx),cpath.dis_interval(idx+1), num_interval);
        end
        s =[s, s_tmp]; sv = [sv, sv_tmp]; sa = [sa, sa_tmp];
    end
    [pos_tmp, vel, acc] = cpath.GenerateTraj(s, sv, sa);
    pos = [pos, pos_tmp];
    % post-clean action
    posn = rbt.FKSolve(g_stowed_pos).t;
    line_length = norm(posn-via_pos(:,end));
    uplanner = LspbTrajPlanner([0,line_length],g_cvmax,g_camax);
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = via_pos(:,end)+up.*(posn-via_pos(:,end))/line_length;
    pos = [pos, pos_tmp];
    % robot inverse kinematics
    ik_option = 'q2first';
    alph = zeros(size(pos,2),1);
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alph(idx), pre_q);    
        sim_q = [sim_q, tmp_q]; pre_q = tmp_q;
        pose_tmp = rbt.FKSolve(tmp_q);
        sim_pos = [sim_pos, pose_tmp.t];
    end




















end