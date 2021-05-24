clear
close all
clc

addpath('classes');
addpath('tools');

rbt = CleanRobot;
global g_jvmax g_jamax g_cvmax g_camax g_stowed_pos g_cycle_time
g_jvmax = [pi, 0.15, 0.8*pi, 0.5, 0.8*pi]*0.5;
g_jamax = [2*pi, 0.3, 1.6*pi, 1, 1.6*pi]*0.5;
g_cvmax = 0.15; g_camax = 0.3;
g_stowed_pos = [0;0.3;0;0;0];
g_cycle_time = 0.001;
%% task setting and trajectory plan
clean_task = {'mirror', 'table', 'sphere', 'ellipsoid'};
task = 'ellipsoid';
show_power = 0;
q0 = [0.2,0.8,0.7,0.2,0.5]';
switch task
    case clean_task(1)
        %% wipe the mirror
        mirror_type = 'rectangle';
        dt = 0.01;
        if strcmp(mirror_type, 'rectangle')
            pos1 = [0.4, 0.7, 0.84]; pos2 = [-0.4, 0.7, 0.84]; pos3 = [-0.4, 0.7, 1.12]; pos4 = [0.4, 0.7, 1.12];
            via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 's');
            [sim_pos, sim_q, sim_qd] = CleanRectMirror(rbt,via_pos,q0,dt);
        elseif strcmp(mirror_type, 'circle')
            center = [0,0.7,1]';
            radius = 0.6;
            norm_vec = [0.03;2;0.0];
            interval = 0.1;
            [circle_pos, sim_pos, sim_q] = CleanCircleMirror(rbt, center, radius, norm_vec, interval, q0, dt);
        end
        
    case clean_task(2)
        %% wipe the table
        pos1 = [0.2, 0.8, 0.8]; pos2 = [-0.2, 0.8, 0.8]; pos3 = [-0.2, 1.2, 0.8]; pos4 = [0.2, 1.2, 0.8];
        dt = 0.01;
        via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 'm');
        [sim_pos, sim_q, sim_qd] = CleanRectMirror(rbt,via_pos,q0,dt);

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
        [sim_pos, sim_q, pos] = CleanSurface(rbt,npts,center(:,1),via_pos,q0,dt);
        
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
        [sim_pos, sim_q, pos] = CleanSurface(rbt,npts,center(:,1),via_pos,q0,dt);
        
    otherwise
        error('CleanRobot can not accomplish the task');
end

%% simulation with simscape and plot
t = [0:dt:dt*(size(sim_q,2)-1)]';
figure
plot2(sim_pos', 'r-');
if strcmp(task, 'mirror')
    if strcmp(mirror_type, 'rectangle')
        hold on; plot2([pos1', pos2', pos3', pos4', pos1']', '--'); hold off;
    elseif strcmp(mirror_type, 'circle')
        hold on; plot2(circle_pos', '--'); hold off;
    end
elseif strcmp(task, 'sphere') || strcmp(task, 'ellipsoid')
    hold on; plot2(pos', '--'); plot3(via_pos(1,:), via_pos(2,:), via_pos(3,:), 'o'); hold off;
end
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

figure
yyaxis left; plot(t,sim_q(1,:),'-', t, sim_q(2,:), '--', t, sim_q(3,:), '-.', t, sim_q(4,:), ':');
yyaxis right; plot(t, sim_q(5,:), '-');
grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5');

% figure
% plot(t,sim_qd(1,:),'-', t, sim_qd(2,:), '--', t, sim_qd(3,:), '-.', t, sim_qd(4,:), ':', t, sim_qd(5,:), '-');
% grid on; title('joint velocity'); legend('dq1', 'dq2', 'dq3', 'dq4', 'dq5');

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

