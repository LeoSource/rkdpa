clear
close all
clc

addpath('classes');
addpath('tools');

rbt = CleanRobot;
g_jvmax = [pi, 0.15, 0.8*pi, 0.5, 0.8*pi];
g_jamax = [2*pi, 0.3, 1.6*pi, 1, 1.6*pi];
g_cvmax = 0.4; g_camax = 0.8;
g_stowed_pos = [0;0;0;0;0];
g_cycle_time = 0.001;
%% task setting and trajectory plan
clean_task = {'mirror', 'table', 'circle', 'sphere', 'ellipsoid'};
task = 'sphere';
show_power = 0;
q0 = [0.2,0.8,0,0,0]';
sim_q = []; sim_pos = [];
switch task
    case clean_task(1)
        %% wipe the mirror
        dt = g_cycle_time;
        pos1 = [0.7, 0.8, 1]; pos2 = [-0.7, 0.8, 1]; pos3 = [-0.7, 0.8, 2.4]; pos4 = [0.7, 0.8, 2.4];
        % pre-clean action
        q = q0;
        q_clean_start = rbt.IKSolve(pos1, 'q2first', 0);
        for idx=1:5
            jplanner(idx) = LspbTrajPlanner([q(idx), q_clean_start(idx)], g_jvmax(idx), g_jamax(idx));
        end
        tf_preclean = max([jplanner(1).tf, jplanner(2).tf, jplanner(3).tf, jplanner(4).tf, jplanner(5).tf]);
        for idx=1:5
            jplanner(idx) = LspbTrajPlanner([q(idx), q_clean_start(idx)], g_jvmax(idx), g_jamax(idx), tf_preclean);
            [s(idx,:), sv(idx,:), sa(idx,:)] = jplanner(idx).GenerateTraj(dt);
        end
        sim_q = [sim_q, s];
        % clean mirror action
        clear s sv sa
        s = []; sv = []; sa = [];
        via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 's');
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
        [pos, vel, acc] = cpath.GenerateTraj(s, sv, sa);
        % robot inverse kinematics
        ik_option = 'q2first';
        alpha = zeros(1, length(s));
        for idx=1:size(pos,2)
            tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alpha(idx));    
            sim_q = [sim_q, tmp_q];
        end
        % post-clean action
        clear s sv sa
        for idx=1:5
            jplanner(idx) = LspbTrajPlanner([tmp_q(idx), g_stowed_pos(idx)], g_jvmax(idx), g_jamax(idx));
        end
        tf_postclean = max([jplanner(1).tf, jplanner(2).tf, jplanner(3).tf, jplanner(4).tf, jplanner(5).tf]);
        for idx=1:5
            jplanner(idx) = LspbTrajPlanner([tmp_q(idx), g_stowed_pos(idx)], g_jvmax(idx), g_jamax(idx), tf_postclean);
            [s(idx,:), sv(idx,:), sa(idx,:)] = jplanner(idx).GenerateTraj(dt);
        end
        sim_q = [sim_q, s];
        for idx=1:size(sim_q,2)
            pose_tmp = rbt.FKSolve(sim_q(:,idx));
            sim_pos = [sim_pos, pose_tmp.t];
        end
    case clean_task(2)
        %% wipe the table
        pos1 = [0.8, 0.4, 0.7]; pos2 = [-0.8, 0.4, 0.7]; pos3 = [-0.8, 1.0, 0.7]; pos4 = [0.8, 1.0, 0.7];
        radius = 0.04;
        tf = 60; dt = 0.01;
        via_pos = CalcRectanglePath2([pos1', pos2', pos3', pos4'], 16, 'm');
        cpath = ArcTransPathPlanner(via_pos, radius);
        planner = LspbTrajPlanner([0, cpath.distance], tf, 0.5, 2, 'limitvel');
        [s, sv, sa] = planner.GenerateTraj(dt);
        [pos, vel, acc] = cpath.GenerateTraj(s, sv, sa);
        ik_option = 'q3first0';
        alpha = zeros(1, length(s));
    case clean_task(3)
        %% wipe the washbasin 
        center = [0; 0.5; 0.5]; radius = 0.3; n_vec = [0;0;1];
        circlepath = ArcPathPlanner(center, n_vec, radius, 'circle');
        tf = 10; dt = 0.01;
        planner = LspbTrajPlanner([0, circlepath.theta], tf, 1, 2, 'limitvel');
        [alpha, alpv, alpa] = planner.GenerateTraj(dt);
        [pos, vel, acc] = circlepath.GenerateTraj(alpha, alpv, alpa);
        ik_option = 'q3firstn';
    case clean_task(4)
        %% wipe the washbasin
        dt = 0.01;
        npts = [15, 10, 8, 6];
        center = [0, 0, 0, 0; 0.5, 0.5, 0.5, 0.5; 0.8-0, 0.8-0.1, 0.8-0.2, 0.8-0.3];
        radius = [0.5, 0.4, 0.3, 0.2];
        via_pos = [];
        for idx=1:length(npts)
            theta = linspace(0, 2*pi, npts(idx)+1);
            tmp_pos = center(:,idx)+[radius(idx)*cos(theta); radius(idx)*sin(theta); zeros(1,npts(idx)+1)];
            via_pos = [via_pos, tmp_pos];
        end
        pos = [];
        % pre-clean action
        q = q0;
        pos0 = rbt.FKSolve(q).t;
        line_length = norm(via_pos(:,1)-pos0);
        uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
        [up, uv, ~] = uplanner.GenerateTraj(dt);
        pos_tmp = pos0+up.*(via_pos(:,1)-pos0)/line_length;
        pos = [pos, pos_tmp];
        % clean washbasin action
        planner = CubicBSplinePlanner(via_pos, 'approximation', 60);
        uplanner = LspbTrajPlanner([planner.uknot_vec(1),planner.uknot_vec(end)],2,1,planner.uknot_vec(end));
        for t=planner.uknot_vec(1):dt:planner.uknot_vec(end)
            [u,du,ddu] = uplanner.GenerateMotion(t);
            [p,v,a] = planner.GenerateMotion(u,du,ddu);
            pos = [pos, p];
        end
        % post-clean action
        posn = rbt.FKSolve(g_stowed_pos).t;
        line_length = norm(posn-via_pos(:,end));
        uplanner = LspbTrajPlanner([0,line_length],g_cvmax,g_camax);
        [up, uv, ~] = uplanner.GenerateTraj(dt);
        pos_tmp = via_pos(:,end)+up.*(posn-via_pos(:,end))/line_length;
        pos = [pos, pos_tmp];
        % robot inverse kinematics
        ik_option = 'q3firstn';
        alpha = zeros(size(pos,2),1);
        sim_pos = [];
        for idx=1:size(pos,2)
            tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alpha(idx));
            sim_q = [sim_q, tmp_q];
            sim_pos = [sim_pos, rbt.FKSolve(tmp_q).t];
        end
    case clean_task(5)
        %% wipe the toilet 
        ik_option = 'q3firstn';
    otherwise
        error('CleanRobot can not accomplish the task');
end


%% trajectory plan
% sample_time = 0.001;
% tf = 20;
% pos = [];
% alpha = [];
switch task    
    case clean_task(4)%sphere
%         dt = 0.01;
%         step = 1*pi/180;
%         origin = [0; 0.5; 0.5];
%         radius = 0.3;
%         pos = [0; origin(2)+radius; origin(3)];
%         interp_phi = [0:-15:-90]*pi/180;
%         for idx=1:length(interp_phi)-1
%             pos_z = pos(3,end);
%             tmp_alpha = 0:step:2*pi;
%             alpha = [alpha, tmp_alpha];
%             r = sqrt(radius^2-(pos_z-origin(3))^2);
%             tmp_pos = [-sin(tmp_alpha)*r; origin(2)+cos(tmp_alpha)*r; pos_z*ones(1,length(tmp_alpha))];
%             pos = [pos, tmp_pos];
% 
%             phi = interp_phi(idx):-step:interp_phi(idx+1);
%             tmp_pos = [zeros(1,length(phi)); 0.5+0.3*cos(phi); 0.5+0.3*sin(phi)];
%             pos = [pos, tmp_pos];
%             alpha = [alpha, zeros(1,length(phi))];
%         end
%         pos = pos(:,2:end);
    case clean_task(5)%ellipsoid
%         dt = 0.01;
%         step = 1*pi/180;
%         origin = [0; 0.5; 1];
%         a = 0.4; b = 0.5; c = 0.3;
%         pos = [0; origin(2)+b; origin(3)];
%         interp_phi = [0: -15: -90]*pi/180;
%         for idx=1:length(interp_phi)-1
%             pos_z = pos(3,end);
%             tmp_alpha = 0:step:2*pi;
%             alpha = [alpha, tmp_alpha];
%             rs_value = sqrt(abs(1-(pos_z-origin(3))^2/c^2));
%             a_new = a*rs_value; b_new = b*rs_value;
%             tmp_pos = [-a_new*sin(tmp_alpha); b_new*cos(tmp_alpha)+origin(2); pos_z*ones(1,length(tmp_alpha))];
%             pos = [pos, tmp_pos];
%             
%             phi = interp_phi(idx):-step:interp_phi(idx+1);
%             tmp_pos = [zeros(1,length(phi)); origin(2)+b*cos(phi); origin(3)+c*sin(phi)];
%             pos = [pos, tmp_pos];
%             alpha = [alpha, zeros(1,length(phi))];
%         end
%         pos = pos(:,2:end);      
    otherwise         
end

t = [0:dt:dt*(size(sim_q,2)-1)]';
%% simulation with simscape and plot
figure
plot2(sim_pos', 'r-');
if strcmp(task, 'mirror')
    hold on; plot2([pos1', pos2', pos3', pos4', pos1']', '--'); hold off;
elseif strcmp(task, 'sphere')
    hold on; plot2(pos', '--'); plot3(via_pos(1,:), via_pos(2,:), via_pos(3,:), 'o'); hold off;
end
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

figure
plot(t,sim_q(1,:),'-', t, sim_q(2,:), '--', t, sim_q(3,:), '-.', t, sim_q(4,:), ':', t, sim_q(5,:), '-');
grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5');

if show_power
    sim('simulink/x_project_g3.slx');
    
    time = 0:dt:tf;
    figure
    plot(time,dq(:,1), '-', time, dq(:,2), '--', time, dq(:,3), '-.', time, dq(:,4), ':', time, dq(:,5), '-');
    grid on
    title('joint velocity');
    legend('dq1', 'dq2', 'dq3', 'dq4', 'dq5');
    
    figure
    plot(time,tau(:,1), '-', time, tau(:,2), '--', time, tau(:,3), '-.', time, tau(:,4), ':', time, tau(:,5), '-');
    grid on; title('joint torque'); legend('tau1', 'tau2', 'tau3', 'tau4', 'tau5');
    
    figure
    power = dq.*tau;
    for idx=1:5
        max_dq(idx) = max(dq(:,idx));
        max_tau(idx) = max(tau(:,idx));
        max_power(idx) = max(power(:,idx));
    end
    plot(time,power(:,1), '-', time, power(:,2), '--', time, power(:,3), '-.', time, power(:,4), ':', time, power(:,5), '-');
    grid on; title('joint actuator power'); legend('power1', 'power2', 'power3', 'power4', 'power5');
    disp(['max_q = ', num2str(max_dq)]);    
    disp(['max_tau = ', num2str(max_tau)]);   
    disp(['max_power = ', num2str(max_power)]);
end

