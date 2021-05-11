function [sim_pos, sim_q, pos] = CleanSurface(rbt,npts,posn,via_pos,q0,dt)

    global g_cvmax g_camax
    sim_pos = []; sim_q = []; pos = []; alph = [];
    % pre-clean action
    pos0 = rbt.FKSolve(q0).t;
    line_length = norm(via_pos(:,1)-pos0);
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    [up, uv, ~] = uplanner.GenerateTraj(dt);
    pos_tmp = pos0+up.*(via_pos(:,1)-pos0)/line_length;
    pos = [pos, pos_tmp];
    alph = [alph; zeros(size(pos,2) ,1)];
    % clean washbasin action
    planner = CubicBSplinePlanner(via_pos, 'approximation', 60);
    uplanner = LspbTrajPlanner([planner.uknot_vec(1),planner.uknot_vec(end)],2,1,planner.uknot_vec(end));
    aplanner = LspbTrajPlanner([0,2*pi*length(npts)], 1, 2, 60);
    for t=planner.uknot_vec(1):dt:planner.uknot_vec(end)
        [u,du,ddu] = uplanner.GenerateMotion(t);
        [p,v,a] = planner.GenerateMotion(u,du,ddu);
        pos = [pos, p];
    end
    alph = [alph; aplanner.GenerateTraj(dt)'];
    % post-clean action
    %         posn = rbt.FKSolve(g_stowed_pos).t;
    line_length = norm(posn-via_pos(:,end));
    uplanner = LspbTrajPlanner([0,line_length],g_cvmax,g_camax);
    [up, uv, ~] = uplanner.GenerateTraj(dt);
    pos_tmp = via_pos(:,end)+up.*(posn-via_pos(:,end))/line_length;
    pos = [pos, pos_tmp];
    max_alph = alph(end);
    alph = [alph; ones(size(pos_tmp,2) ,1)*max_alph];
    % robot inverse kinematics
    ik_option = 'q3firstn';
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alph(idx), pre_q);
        sim_q = [sim_q, tmp_q]; pre_q = tmp_q;
        sim_pos = [sim_pos, rbt.FKSolve(tmp_q).t];
    end
    
end