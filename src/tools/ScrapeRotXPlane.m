function [sim_pos, sim_q] = ScrapeRotXPlane(rbt,via_pos,pitch_x,q0,dt)

    global g_cvmax g_stowed_pos g_camax g_jvmax g_jamax
    sim_pos = []; pos = [];
    sim_q = [];
    pitch = [];
    inc_angle = [20, 50]*pi/180;
    pitch_high = pitch_x-inc_angle(1);
    pitch_low = pitch_x-inc_angle(2);
    rbt.SetPitchRange(pitch_high, pitch_low);

    %% pre-clean action
    pre_j1planner = LspbTrajPlanner([q0(1), 0], g_jvmax(1), g_jamax(1));
    pre_j5planner = LspbTrajPlanner([q0(5),0], g_jvmax(5), g_jamax(5));
    tf1_pre = max(pre_j1planner.tf, pre_j5planner.tf);
    pre_j1planner = LspbTrajPlanner([q0(1), 0], g_jvmax(1), g_jamax(1), tf1_pre);
    pre_j5planner = LspbTrajPlanner([q0(5),0], g_jvmax(5), g_jamax(5), tf1_pre);
    [j1pos,~,~] = pre_j1planner.GenerateTraj(dt);
    [j5pos,~,~] = pre_j5planner.GenerateTraj(dt);
    tmp_q = [j1pos; ones(1,length(j1pos))*q0(2);...
            ones(1,length(j1pos))*q0(3);ones(1,length(j1pos))*q0(4);j5pos];
    for idx=1:length(j1pos)
        sim_pos = [sim_pos, rbt.FKSolveTool(tmp_q(:,idx)).t];
    end
    sim_q = [sim_q, tmp_q];
    pos0 = rbt.FKSolveTool(tmp_q(:,end)).t;
    line_length = norm(via_pos(:,1)-pos0);
    pitch0 = q0(3)+rbt.tool_pitch;
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    pitchplanner = LspbTrajPlanner([pitch0,rbt.pitch_high], g_jvmax(3), g_jamax(3));
    tf2_pre = max(uplanner.tf,pitchplanner.tf);
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax, tf2_pre);
    pitchplanner = LspbTrajPlanner([pitch0,rbt.pitch_high], g_jvmax(3), g_jamax(3),tf2_pre);
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = pos0+up.*(via_pos(:,1)-pos0)/line_length;
    pos = [pos, pos_tmp];
    pitch = [pitch, pitchplanner.GenerateTraj(dt)];

    %% clean plane action
    pitchplanner = LspbTrajPlanner([rbt.pitch_high, rbt.pitch_low], g_jvmax(3), g_jamax(3));
    line_length = norm(via_pos(:,2)-via_pos(:,1));
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    tf_clean = max(pitchplanner.tf,uplanner.tf);
    pitchplanner = LspbTrajPlanner([rbt.pitch_high, rbt.pitch_low], g_jvmax(3), g_jamax(3),tf_clean);
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax,tf_clean);
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = via_pos(:,1)+up.*(via_pos(:,2)-via_pos(:,1))/line_length;
    pos = [pos, pos_tmp];
    pitch = [pitch, pitchplanner.GenerateTraj(dt)];

    %% post-clean action
    posn = via_pos(:,2)+[0;-0.1;0];
    line_length = norm(posn-via_pos(:,2));
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = via_pos(:,2)+up.*(posn-via_pos(:,2))/line_length;
    pos = [pos, pos_tmp];
    pitch = [pitch, ones(1,length(up))*rbt.pitch_low];

    %% robot inverse kinematics
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolvePitch(pos(:,idx), pitch(idx));
        sim_q = [sim_q, tmp_q];
        pre_q = tmp_q;
        pose_tmp = rbt.FKSolveTool(tmp_q);
        sim_pos = [sim_pos, pose_tmp.t];
    end


end