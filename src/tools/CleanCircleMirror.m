function [sim_pos, sim_q] = CleanCircleMirror(rbt,center,radius,interval,q0,dt)

    global g_cvmax g_stowed_pos g_camax g_jvmax g_jamax
    sim_pos = []; sim_q = []; pos = []; alph = [];
    % pre-clean action
    pos0 = rbt.FKSolve(q0).t;
    line_length = norm(center-pos0);
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
    alph0 = q0(1)+q0(end);
    alphplanner = LspbTrajPlanner([alph0,0], g_jvmax(1), g_jamax(end));
    tf_pre = max(alphplanner.tf,uplanner.tf);
    alphplanner = LspbTrajPlanner([alph0,0], g_jvmax(1), g_jamax(end), tf_pre);
    uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax, tf_pre);
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = pos0+up.*(center-pos0)/line_length;
    pos = [pos, pos_tmp];
    alph = [alph, alphplanner.GenerateTraj(dt)];
    % clean mirror action
    v = interval/2/pi;
    theta_end = radius/v;
    thplanner = LspbTrajPlanner([0,theta_end], 0.7, 0.2);
    [thp, thv, tha] = thplanner.GenerateTraj(dt);
    r = v*thp;
    p(1,:) = center(1)+r.*cos(thp);
    p(2,:) = center(2)*ones(size(thp));
    p(3,:) = center(3)+r.*sin(thp);
    pos = [pos, p];
    alph = [alph, zeros(1,size(p,2))];
    % post-clean action
    posn = rbt.FKSolve(g_stowed_pos).t;
    line_length = norm(posn-pos(:,end));
    uplanner = LspbTrajPlanner([0,line_length],g_cvmax,g_camax);
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = pos(:,end)+up.*(posn-pos(:,end))/line_length;
    pos = [pos, pos_tmp];
    alph = [alph, zeros(1,size(pos_tmp,2))];
    % robot inverse kinematics
    ik_option = 'q2first';
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alph(idx), pre_q);
        sim_q = [sim_q, tmp_q]; pre_q = tmp_q;
        pose_tmp = rbt.FKSolve(tmp_q);
        sim_pos = [sim_pos, pose_tmp.t];
    end

end