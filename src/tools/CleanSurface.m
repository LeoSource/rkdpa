function [sim_pos, sim_q, pos] = CleanSurface(rbt,npts,posn,via_pos,q0,dt)

    global g_cvmax g_camax g_jvmax g_jamax
    sim_pos = []; sim_q = []; pos = []; alph = [];
    %% pre-clean action
    jplanner = LspbTrajPlanner([q0(3),-pi/6], g_jvmax(3), g_jamax(3));
    [jpos,~,~] = jplanner.GenerateTraj(dt);
    tmp_q = [ones(1,length(jpos))*q0(1); ones(1,length(jpos))*q0(2); jpos;...
             ones(1,length(jpos))*q0(4); ones(1,length(jpos))*q0(5)];
    for idx=1:length(jpos)
        sim_pos = [sim_pos, rbt.FKSolve(tmp_q(:,idx)).t];
    end
    sim_q = [sim_q, tmp_q];
    pos0 = rbt.FKSolve(tmp_q(:,end)).t;
    line_length = norm(via_pos(:,1)-pos0);
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    alphplanner = LspbTrajPlanner([q0(1)+q0(5),0], g_jvmax(1), g_jamax(1));
    tf_preclean = max(uplanner.tf, alphplanner.tf);
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax, tf_preclean);
    alphplanner = LspbTrajPlanner([q0(1)+q0(5),0], g_jvmax(1), g_jamax(1), tf_preclean);
    [up, ~, ~] = uplanner.GenerateTraj(dt);
    pos_tmp = pos0+up.*(via_pos(:,1)-pos0)/line_length;
    pos = [pos, pos_tmp];
    alph = [alph; alphplanner.GenerateTraj(dt)'];
    
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
    
    %% post-clean action
    %         posn = rbt.FKSolve(g_stowed_pos).t;
    line_length = norm(posn-via_pos(:,end));
    uplanner = LspbTrajPlanner([0,line_length],g_cvmax,g_camax);
    [up, ~, ~] = uplanner.GenerateTraj(dt);
    pos_tmp = via_pos(:,end)+up.*(posn-via_pos(:,end))/line_length;
    pos = [pos, pos_tmp];
    max_alph = alph(end);
    alph = [alph; ones(size(pos_tmp,2) ,1)*max_alph];
    
    %% robot inverse kinematics
    ik_option = 'q3firstn';
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alph(idx), pre_q);
        sim_q = [sim_q, tmp_q]; pre_q = tmp_q;
        sim_pos = [sim_pos, rbt.FKSolve(tmp_q).t];
    end
    
end