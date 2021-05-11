clear
close all
clc

addpath('classes');
addpath('tools');

rbt = CleanRobot;
global g_jvmax g_jamax g_cvmax g_camax g_stowed_pos
g_jvmax = [pi, 0.15, 0.8*pi, 0.5, 0.8*pi];
g_jamax = [2*pi, 0.3, 1.6*pi, 1, 1.6*pi];
g_cvmax = 0.4; g_camax = 0.8;
g_stowed_pos = [0;0;0;0;0];
g_cycle_time = 0.001;
%% task setting and trajectory plan
clean_task = {'mirror', 'table', 'sphere', 'ellipsoid'};
task = 'mirror';
show_power = 0;
q0 = [0.2,0.8,0,0,0]';
sim_q = []; sim_pos = [];
switch task
    case clean_task(1)
        %% wipe the mirror
        dt = 0.01;
        pos1 = [0.7, 0.8, 1]; pos2 = [-0.7, 0.8, 1]; pos3 = [-0.7, 0.8, 2.4]; pos4 = [0.7, 0.8, 2.4];
        via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 's');
        [sim_pos, sim_q] = CleanMirror(rbt,via_pos,q0,dt);
        
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
        dt = 0.01;
        npts = [15, 10, 8, 6];
        center = [0, 0, 0, 0; 0.7, 0.7, 0.7, 0.7; 0.6-0, 0.6-0.1, 0.6-0.2, 0.6-0.3];
        radius = [0.4, 0.3, 0.2, 0.1];
        via_pos = [];
        for idx=1:length(npts)
            theta = linspace(0, 2*pi, npts(idx)+1);
            tmp_pos = center(:,idx)+[radius(idx)*cos(theta); radius(idx)*sin(theta); zeros(1,npts(idx)+1)];
            via_pos = [via_pos, tmp_pos];
        end
        pos = []; alph = [];
        % pre-clean action
        q = q0;
        pos0 = rbt.FKSolve(q).t;
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
        posn = center(:,1);
        line_length = norm(posn-via_pos(:,end));
        uplanner = LspbTrajPlanner([0,line_length],g_cvmax,g_camax);
        [up, uv, ~] = uplanner.GenerateTraj(dt);
        pos_tmp = via_pos(:,end)+up.*(posn-via_pos(:,end))/line_length;
        pos = [pos, pos_tmp];
        max_alph = alph(end);
        alph = [alph; ones(size(pos_tmp,2) ,1)*max_alph];
        % robot inverse kinematics
        ik_option = 'q3firstn';
        sim_pos = []; pre_q = q;
        for idx=1:size(pos,2)
            tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alph(idx), pre_q);
            sim_q = [sim_q, tmp_q]; pre_q = tmp_q;
            sim_pos = [sim_pos, rbt.FKSolve(tmp_q).t];
        end
    case clean_task(4)
        %% wipe the toilet 
        dt = 0.01;
        npts = [15, 12, 10, 8, 6, 4];
        center = [0,0,0,0,0,0; 0.7, 0.7, 0.7, 0.7, 0.7, 0.7;...
                0.6-0, 0.6-0.06, 0.6-0.12, 0.6-0.18, 0.6-0.24, 0.6-0.3];
        elli_params = [0.6, 0.5, 0.4, 0.3, 0.2, 0.1; 0.5, 0.4, 0.3, 0.2, 0.1, 0.05];
        via_pos = [];
        for idx=1:length(npts)
            theta = linspace(0,2*pi,npts(idx)+1);
            tmp_pos = center(:,idx)+[elli_params(1,idx)*cos(theta); elli_params(2,idx)*sin(theta); zeros(1,npts(idx)+1)];
            via_pos = [via_pos, tmp_pos];
        end
        pos = []; alph = [];
        % pre-clean action
        q = q0;
        pos0 = rbt.FKSolve(q).t;
        line_length = norm(via_pos(:,1)-pos0);
        uplanner = LspbTrajPlanner([0,line_length], g_cvmax, g_camax);
        [up, ~, ~] = uplanner.GenerateTraj(dt);
        pos_tmp = pos0+up.*(via_pos(:,1)-pos0)/line_length;
        pos = [pos, pos_tmp];
        alph = [alph; zeros(size(pos,2),1)];
        % clean toilet action
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
        posn = center(:,1);
        line_length = norm(posn-via_pos(:,end));
        uplanner = LspbTrajPlanner([0,line_length],g_cvmax,g_camax);
        [up,~,~] = uplanner.GenerateTraj(dt);
        pos_tmp = via_pos(:,end)+up.*(posn-via_pos(:,end))/line_length;
        pos = [pos, pos_tmp];
        max_alph = alph(end);
        alph = [alph; ones(size(pos_tmp,2),1)*max_alph];
        % robot inverse kinematics        
        ik_option = 'q3firstn';
        sim_pos = []; pre_q = q;
        for idx=1:size(pos,2)
            tmp_q = rbt.IKSolve(pos(:,idx), ik_option, alph(idx), pre_q);
            sim_q = [sim_q, tmp_q]; pre_q = tmp_q;
            sim_pos = [sim_pos, rbt.FKSolve(tmp_q).t];
        end
        
    otherwise
        error('CleanRobot can not accomplish the task');
end

%% simulation with simscape and plot
t = [0:dt:dt*(size(sim_q,2)-1)]';
figure
plot2(sim_pos', 'r-');
if strcmp(task, 'mirror')
    hold on; plot2([pos1', pos2', pos3', pos4', pos1']', '--'); hold off;
elseif strcmp(task, 'sphere') || strcmp(task, 'ellipsoid')
    hold on; plot2(pos', '--'); plot3(via_pos(1,:), via_pos(2,:), via_pos(3,:), 'o'); hold off;
end
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

figure
yyaxis left; plot(t,sim_q(1,:),'-', t, sim_q(2,:), '--', t, sim_q(3,:), '-.', t, sim_q(4,:), ':');
yyaxis right; plot(t, sim_q(5,:), '-');
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

