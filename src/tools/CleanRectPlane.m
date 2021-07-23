function [sim_pos, sim_q, sim_qd] = CleanRectPlane(rbt,via_pos,q0,dt)

    global g_cvmax g_stowed_pos g_camax g_jvmax g_jamax
    sim_pos = []; pos = []; vel = [];
    sim_q = []; sim_qd = [];
    alph = []; 
    %%  pre-clean action
    jplanner = LspbPlanner([q0(3), 0], g_jvmax(3), g_jamax(3));
    [jpos, jvel,~] = jplanner.GenerateTraj(dt);
    tmp_q = [ones(1,length(jpos))*q0(1); ones(1,length(jpos))*q0(2); jpos;...
             ones(1,length(jpos))*q0(4); ones(1,length(jpos))*q0(5)];
    tmp_qd = [zeros(2, length(jvel)); jvel; zeros(2, length(jvel))];
    for idx=1:length(jpos)
        sim_pos = [sim_pos, rbt.FKSolveTool(tmp_q(:,idx)).t];
    end
    sim_q = [sim_q, tmp_q]; sim_qd = [sim_qd, tmp_qd];
    pos0 = rbt.FKSolveTool(tmp_q(:,end)).t;
    line_length = norm(via_pos(:,1)-pos0);
    alph0 = q0(1)+q0(end);
    alphplanner = LspbPlanner([alph0,0], g_jvmax(1), g_jamax(end));
    uplanner = LspbPlanner([0,line_length], g_cvmax(1), g_camax(1));
    tf_pre = max(alphplanner.tf,uplanner.tf);
    alphplanner = LspbPlanner([alph0,0], g_jvmax(1), g_jamax(end), tf_pre);
    uplanner = LspbPlanner([0,line_length], g_cvmax(1), g_camax(1), tf_pre);
    [up,uv,~] = uplanner.GenerateTraj(dt);
    pos_tmp = pos0+up.*(via_pos(:,1)-pos0)/line_length;
    vel_tmp = (via_pos(:,1)-pos0)/line_length.*uv;
    pos = [pos, pos_tmp]; vel = [vel, vel_tmp];
    alph = [alph, alphplanner.GenerateTraj(dt)];
    
    %%  clean mirror action
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
    [pos_tmp, vel_tmp, ~] = cpath.GenerateTraj(s, sv, sa);
    pos = [pos, pos_tmp]; vel = [vel, vel_tmp(1:3,:)];
    alph = [alph, zeros(1,size(pos_tmp,2))];
    
    %%  post-clean action
    posn = rbt.FKSolveTool(g_stowed_pos).t;
    line_length = norm(posn-via_pos(:,end));
    uplanner = LspbPlanner([0,line_length],g_cvmax(1),g_camax(1));
    [up,uv,~] = uplanner.GenerateTraj(dt);
    pos_tmp = via_pos(:,end)+up.*(posn-via_pos(:,end))/line_length;
    vel_tmp = uv.*(posn-via_pos(:,end))/line_length;
    pos = [pos, pos_tmp]; vel = [vel, vel_tmp];
    alph = [alph, zeros(1,size(pos_tmp,2))];
    
    %% robot inverse kinematics
    ik_option = 'q2first';
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alph(idx), pre_q);
%         tmp_q = rbt.IKSolveYaw(pos(:,idx), alph(idx), pre_q);
        jaco_tool = rbt.CalcJacoTool(pre_q);
        tmp_qd = pinv(jaco_tool(1:3,:))*vel(:,idx);
        sim_q = [sim_q, tmp_q]; sim_qd = [sim_qd, tmp_qd];
        pre_q = tmp_q;
        pose_tmp = rbt.FKSolveTool(tmp_q);
        sim_pos = [sim_pos, pose_tmp.t];
    end


end