function [circle_pos, sim_pos, sim_q] = CleanCircleMirror(rbt, center, radius, norm_vec, interval, q0, dt)

    global g_cvmax g_stowed_pos g_camax g_jvmax g_jamax
    sim_pos = []; sim_q = []; pos = []; alph = []; circle_pos = [];

    %% pre-clean action
    jplanner = LspbPlanner([q0(3), 0], g_jvmax(3), g_jamax(3));
    [jpos, ~,~] = jplanner.GenerateTraj(dt);
    tmp_q = [ones(1,length(jpos))*q0(1); ones(1,length(jpos))*q0(2); jpos;...
             ones(1,length(jpos))*q0(4); ones(1,length(jpos))*q0(5)];
    for idx=1:length(jpos)
        sim_pos = [sim_pos, rbt.FKSolveTool(tmp_q(:,idx)).t];
    end
    sim_q = [sim_q, tmp_q];
    pos0 = rbt.FKSolveTool(tmp_q(:,end)).t;
    line_length = norm(center-pos0);
    uplanner = LspbPlanner([0,line_length], g_cvmax(1), g_camax(1));
    alph0 = q0(1)+q0(end);
    alphplanner = LspbPlanner([alph0,0], g_jvmax(1), g_jamax(end));
    tf_pre = max(alphplanner.tf,uplanner.tf);
    alphplanner = LspbPlanner([alph0,0], g_jvmax(1), g_jamax(end), tf_pre);
    uplanner = LspbPlanner([0,line_length], g_cvmax(1), g_camax(1), tf_pre);
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = pos0+up.*(center-pos0)/line_length;
    pos = [pos, pos_tmp];
    alph = [alph, alphplanner.GenerateTraj(dt)];
    
    %% define circle function and clean mirror action
    D = -dot(norm_vec,center);%Ax+By+Cz+D=0
    p1 = zeros(3,1); p1(3) = center(3); p1(1) = center(1)+1;
    p1(2) = (-norm_vec(1)*p1(1)-norm_vec(3)*p1(3)-D)/norm_vec(2);
    a = norm_vec/norm(norm_vec); n = p1-center; n = n/norm(a);
    o = cross(a,n);
    circle_rot = [n,o,a];
    
    v = interval/2/pi;
    theta_end = radius/v;
    thplanner = LspbPlanner([0,theta_end], 0.7, 0.2);
    [thp, thv, tha] = thplanner.GenerateTraj(dt);
    r = v*thp;
    for idx=1:length(thp)
        p = center+circle_rot*[r(idx)*cos(thp(idx)); r(idx)*sin(thp(idx)); 0];
        pos = [pos, p];
    end
    alph = [alph, zeros(1,length(thp))];
    
    %% post-clean action
    posn = rbt.FKSolveTool(g_stowed_pos).t;
    line_length = norm(posn-pos(:,end));
    uplanner = LspbPlanner([0,line_length],g_cvmax(1),g_camax(1));
    [up,~,~] = uplanner.GenerateTraj(dt);
    pos_tmp = pos(:,end)+up.*(posn-pos(:,end))/line_length;
    pos = [pos, pos_tmp];
    alph = [alph, zeros(1,size(pos_tmp,2))];
    
    %% robot inverse kinematics
    ik_option = 'q2first';
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alph(idx), pre_q);
        sim_q = [sim_q, tmp_q]; pre_q = tmp_q;
        pose_tmp = rbt.FKSolveTool(tmp_q);
        sim_pos = [sim_pos, pose_tmp.t];
    end
    
    for th=0:0.01*pi:2*pi
        tmp_pos = center+circle_rot*[r(end)*cos(th); r(end)*sin(th); 0];
        circle_pos = [circle_pos, tmp_pos];
    end

end