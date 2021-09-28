function [sim_pos, sim_q] = BrushRotXPlane(rbt,via_pos,pitch_x,q0,dt)

    global g_cvmax g_stowed_pos g_camax g_jvmax g_jamax
    sim_pos = []; pos = [];
    sim_q = [];
    pitch = []; yaw = [];
    
    %% pre-clean action
    % initialize planner
    yaw0 = q0(1)+q0(end);
    pitch0 = q0(3)+rbt.tool_pitch;
    pre_yawplanner = LspbPlanner([yaw0,0],g_jvmax(1),g_jamax(1));
    pre_pitchPLanner = LspbPlanner([pitch0, pitch_x], g_jvmax(3), g_jamax(3));
    pos0 = rbt.FKSolveTool(q0).t;
    line_length = norm(via_pos(:,1)-pos0);
    pre_uplanner = LspbPlanner([0,line_length], g_cvmax(1), g_camax(1));
    tf_pre = max([pre_yawplanner.tf, pre_pitchPLanner.tf, pre_uplanner.tf]);
    pre_yawplanner = LspbPlanner([yaw0,0],g_jvmax(1),g_jamax(1), tf_pre);
    pre_pitchPLanner = LspbPlanner([pitch0, pitch_x], g_jvmax(3), g_jamax(3), tf_pre);
    pre_uplanner = LspbPlanner([0,line_length], g_cvmax(1), g_camax(1), tf_pre);
    % generate trajectory
    [up,~,~] = pre_uplanner.GenerateTraj(dt);
    pos_tmp = pos0+up.*(via_pos(:,1)-pos0)/line_length;
    pos = [pos, pos_tmp];
    pitch = [pitch, pre_pitchPLanner.GenerateTraj(dt)];
    yaw = [yaw, pre_yawplanner.GenerateTraj(dt)];

    %% clean plane action
    s = []; sv = []; sa = [];
    cpath = ArcTransPathPlanner(via_pos, 0);
    varc = sqrt(g_camax(1)*cpath.radius);
    for idx=1:length(cpath.dis_interval)-1
        if mod(idx,2)==1
            if idx==1
                vel_cons = [0,varc];
            elseif idx==length(cpath.dis_interval)-1
                vel_cons = [varc,0];
            else
                vel_cons = [varc,varc];
            end
            splanner = LspbPlanner([cpath.dis_interval(idx),cpath.dis_interval(idx+1)],g_cvmax(1),g_camax(1),[],vel_cons);
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
    [pos_tmp, ~, ~] = cpath.GenerateTraj(s, sv, sa);
    pos = [pos, pos_tmp];
    yaw = [yaw, zeros(1,size(pos_tmp,2))];
    pitch = [pitch, ones(1,size(pos_tmp,2))*pitch_x];

    %% post-clean action
    posn = rbt.FKSolveTool(g_stowed_pos).t;
    line_length = norm(posn-via_pos(:,end));
    uplanner = LspbPlanner([0,line_length],g_cvmax(1),g_camax(1));
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = via_pos(:,end)+up.*(posn-via_pos(:,end))/line_length;
    pos = [pos, pos_tmp];
    yaw = [yaw, zeros(1,size(pos_tmp,2))];
    pitch = [pitch, ones(1,size(pos_tmp,2))*pitch_x];

    %% robot inverse kinematics
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolvePitchYaw(pos(:,idx), pitch(idx), yaw(idx), pre_q);
        sim_q = [sim_q, tmp_q];
        pre_q = tmp_q;
        pose_tmp = rbt.FKSolveTool(tmp_q);
        sim_pos = [sim_pos, pose_tmp.t];
    end

end