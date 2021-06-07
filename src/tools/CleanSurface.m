function [sim_pos, sim_q, pos] = CleanSurface(rbt,npts,posn,via_pos,q0,dt)

    global g_cvmax g_camax
    sim_pos = []; sim_q = []; pos = []; rpy = []; alph = [];
    %% pre-clean action
    yaw0 = q0(1)+q0(end);
    pitch0 = q0(3)+rbt.tool_pitch;
    rpy0 = [0;pitch0;yaw0];
    pos0 = rbt.FKSolveTool(q0).t;
    line_traj = LineTrajPlanner(pos0,via_pos(:,1),g_cvmax,g_camax,[0,0],...
                                rpy0,[0;-pi/6;0],0.15,0.3,[0,0],'both');
    [pos_tmp,~,~,rpy_tmp,~,~] = line_traj.GenerateTraj(dt);
    pos = [pos,pos_tmp];
    rpy = [rpy,rpy_tmp];
    
    %% clean washbasin action
    planner = CubicBSplinePlanner(via_pos, 'approximation', 60);
    uplanner = LspbTrajPlanner([planner.uknot_vec(1),planner.uknot_vec(end)],2,1,planner.uknot_vec(end));
    aplanner = LspbTrajPlanner([0,2*pi*length(npts)], 1, 2, 60);
    for t=planner.uknot_vec(1):dt:planner.uknot_vec(end)
        [u,du,ddu] = uplanner.GenerateMotion(t);
        [p,v,a] = planner.GenerateMotion(u,du,ddu);
        pos = [pos, p];
    end
    alph = [alph; aplanner.GenerateTraj(dt)'];
    rpy_tmp = [zeros(1,length(alph)); ones(1,length(alph))*rpy(2,end);alph'];
    rpy = [rpy,rpy_tmp];
    
    %% post-clean action
    line_traj = LineTrajPlanner(via_pos(:,end),posn,g_cvmax,g_camax,[0,0],...
                                rpy(:,end),[],[],[],[],'pos');
    [pos_tmp,~,~,rpy_tmp,~,~] = line_traj.GenerateTraj(dt);
    pos = [pos, pos_tmp];
    rpy = [rpy, rpy_tmp];
    
    %% robot inverse kinematics
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolvePitchYaw(pos(:,idx),rpy(2,idx),rpy(3,idx),pre_q);
        sim_q = [sim_q, tmp_q]; pre_q = tmp_q;
        sim_pos = [sim_pos, rbt.FKSolveTool(tmp_q).t];
    end
    
end