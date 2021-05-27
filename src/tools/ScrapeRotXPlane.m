function [sim_pos, sim_q] = ScrapeRotXPlane(rbt,via_pos,pitch_x,q0,dt)

    global g_cvmax g_stowed_pos g_camax g_jvmax g_jamax
    sim_pos = []; pos = [];
    sim_q = [];
    pitch = []; yaw = [];
    inc_angle = [20, 50]*pi/180;
    pitch_high = pitch_x-inc_angle(1);
    pitch_low = pitch_x-inc_angle(2);
    rbt.SetPitchRange(pitch_high, pitch_low);

    %% pre-clean action
    % initialize planner
    yaw0 = q0(1)+q0(end);
    pitch0 = q0(3)+rbt.tool_pitch;
    pre_yawplanner = LspbTrajPlanner([yaw0,0],g_jvmax(1),g_jamax(1));
    pre_pitchPLanner = LspbTrajPlanner([pitch0, rbt.pitch_high], g_jvmax(3), g_jamax(3));
    pos0 = rbt.FKSolveTool(q0).t;
    line_length = norm(via_pos(:,1)-pos0);
    pre_uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    tf_pre = max([pre_yawplanner.tf, pre_pitchPLanner.tf, pre_uplanner.tf]);
    pre_yawplanner = LspbTrajPlanner([yaw0,0],g_jvmax(1),g_jamax(1), tf_pre);
    pre_pitchPLanner = LspbTrajPlanner([pitch0, rbt.pitch_high], g_jvmax(3), g_jamax(3), tf_pre);
    pre_uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax, tf_pre);
    % generate trajectory
    [up,~,~] = pre_uplanner.GenerateTraj(dt);
    pos_tmp = pos0+up.*(via_pos(:,1)-pos0)/line_length;
    pos = [pos, pos_tmp];
    pitch = [pitch, pre_pitchPLanner.GenerateTraj(dt)];
    yaw = [yaw, pre_yawplanner.GenerateTraj(dt)];

    %% clean plane action
    % initialize planner
    pitchplanner = LspbTrajPlanner([rbt.pitch_high, rbt.pitch_low], g_jvmax(3), g_jamax(3));
    line_length = norm(via_pos(:,2)-via_pos(:,1));
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    tf_clean = max(pitchplanner.tf,uplanner.tf);
    pitchplanner = LspbTrajPlanner([rbt.pitch_high, rbt.pitch_low], g_jvmax(3), g_jamax(3),tf_clean);
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax,tf_clean);
    % generate trajectory
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = via_pos(:,1)+up.*(via_pos(:,2)-via_pos(:,1))/line_length;
    pos = [pos, pos_tmp];
    pitch = [pitch, pitchplanner.GenerateTraj(dt)];
    yaw = [yaw, zeros(size(up))];

    %% post-clean action
    % initialize planner
    posn = via_pos(:,2)+[0;-0.1;0];
    line_length = norm(posn-via_pos(:,2));
    post_uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    % generate trajectory
    [up,~,~] = post_uplanner.GenerateTraj(dt);
    pos_tmp = via_pos(:,2)+up.*(posn-via_pos(:,2))/line_length;
    pos = [pos, pos_tmp];
    pitch = [pitch, ones(size(up))*rbt.pitch_low];
    yaw = [yaw, zeros(size(up))];

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