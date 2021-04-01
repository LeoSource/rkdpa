clear
close all
clc

addpath('classes');
addpath('tools');
% TO DO: optimize the sphere and ellipsoid path planner like ArcPathPlanner
rbt = CleanRobot;
%% task setting and trajectory plan
clean_task = {'mirror', 'table', 'circle', 'sphere', 'ellipsoid'};
task = 'mirror';
interp_pos = [];
switch task
    case clean_task(1)
        % wipe the mirror
        pos1 = [0.7, 0.8, 1]; pos2 = [-0.7, 0.8, 1]; pos3 = [-0.7, 0.8, 2.4]; pos4 = [0.7, 0.8, 2.4];
        radius = 0.04;
        tf = 60; dt = 0.01;
        via_pos = CalcRectanglePath2([pos1', pos2', pos3', pos4'], 15, 's');
        cpath = ArcTransPathPlanner(via_pos, radius);
        planner = LspbTrajPlanner([0, cpath.distance], tf, 0.5, 2, 'limitvel');
        [s, sv, sa] = planner.GenerateTraj(dt);
        [pos, vel, acc] = cpath.GenerateTraj(s, sv, sa);
        ik_option = 'q2first';
        alpha = zeros(1, length(s));
    case clean_task(2)
        % wipe the table
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
        % wipe the washbasin 
        center = [0; 0.5; 0.5]; radius = 0.3; n_vec = [0;0;1];
        circlepath = ArcPathPlanner(center, n_vec, radius, 'circle');
        tf = 10; dt = 0.01;
        planner = LspbTrajPlanner([0, circlepath.theta], tf, 1, 2, 'limitvel');
        [alpha, alpv, alpa] = planner.GenerateTraj(dt);
        [pos, vel, acc] = circlepath.GenerateTraj(alpha, alpv, alpa);
        ik_option = 'q3firstn';
    case clean_task(4)
        %wipe the washbasin 
        ik_option = 'q3firstn';
    case clean_task(5)
        %wipe the washbasin 
        ik_option = 'q3firstn';
    otherwise
        error('CleanRobot can not accomplish the task');
end


%% trajectory plan
sample_time = 0.01;
% tf = 20;
% pos = [];
% alpha = [];
switch task    
    case clean_task(4)%sphere
        dt = 0.01;
        step = 1*pi/180;
        origin = [0; 0.5; 0.5];
        radius = 0.3;
        pos = [0; origin(2)+radius; origin(3)];
        interp_phi = [0:-15:-90]*pi/180;
        for idx=1:length(interp_phi)-1
            pos_z = pos(3,end);
            tmp_alpha = 0:step:2*pi;
            alpha = [alpha, tmp_alpha];
            r = sqrt(radius^2-(pos_z-origin(3))^2);
            tmp_pos = [-sin(tmp_alpha)*r; origin(2)+cos(tmp_alpha)*r; pos_z*ones(1,length(tmp_alpha))];
            pos = [pos, tmp_pos];

            phi = interp_phi(idx):-step:interp_phi(idx+1);
            tmp_pos = [zeros(1,length(phi)); 0.5+0.3*cos(phi); 0.5+0.3*sin(phi)];
            pos = [pos, tmp_pos];
            alpha = [alpha, zeros(1,length(phi))];
        end
        pos = pos(:,2:end);
    case clean_task(5)%ellipsoid
        dt = 0.01;
        step = 1*pi/180;
        origin = [0; 0.5; 1];
        a = 0.4; b = 0.5; c = 0.3;
        pos = [0; origin(2)+b; origin(3)];
        interp_phi = [0: -15: -90]*pi/180;
        for idx=1:length(interp_phi)-1
            pos_z = pos(3,end);
            tmp_alpha = 0:step:2*pi;
            alpha = [alpha, tmp_alpha];
            rs_value = sqrt(abs(1-(pos_z-origin(3))^2/c^2));
            a_new = a*rs_value; b_new = b*rs_value;
            tmp_pos = [-a_new*sin(tmp_alpha); b_new*cos(tmp_alpha)+origin(2); pos_z*ones(1,length(tmp_alpha))];
            pos = [pos, tmp_pos];
            
            phi = interp_phi(idx):-step:interp_phi(idx+1);
            tmp_pos = [zeros(1,length(phi)); origin(2)+b*cos(phi); origin(3)+c*sin(phi)];
            pos = [pos, tmp_pos];
            alpha = [alpha, zeros(1,length(phi))];
        end
        pos = pos(:,2:end);      
    otherwise         
end

sim_q = [];
sim_pos = [];
pos = pos';
for idx=1:size(pos,1)    
    tmp_q = rbt.IKSolve(pos(idx,:), ik_option, alpha(idx));    
    sim_q = [sim_q; tmp_q];
    tmp_pose = rbt.FKSolve(tmp_q);
    sim_pos = [sim_pos; tmp_pose.t'];
end
pos_err = pos-sim_pos;
t = [0:sample_time:sample_time*(size(sim_q,1)-1)]';
%% simulation with simscape and plot
sim('x_project_g3.slx');

figure(1)
plot2(pos, '--');
grid on
hold on
plot2(sim_pos, 'r');
legend('cmd\_pos', 'sim\_pos');
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
hold off

figure(2)
plot(t,sim_q(:,1),'-', t, sim_q(:,2), '--', t, sim_q(:,3), '-.', t, sim_q(:,4), ':', t, sim_q(:,5), '-');
grid on
title('joint position');
legend('q1', 'q2', 'q3', 'q4', 'q5');

figure
plot(pos_err);
grid on
title('cartesian position error');
legend('x', 'y', 'z');

time = 0:dt:tf;
figure
plot(time,dq(:,1), '-', time, dq(:,2), '--', time, dq(:,3), '-.', time, dq(:,4), ':', time, dq(:,5), '-');
grid on
title('joint velocity');
legend('dq1', 'dq2', 'dq3', 'dq4', 'dq5');

figure
plot(time,tau(:,1), '-', time, tau(:,2), '--', time, tau(:,3), '-.', time, tau(:,4), ':', time, tau(:,5), '-');
grid on
title('joint torque');
legend('tau1', 'tau2', 'tau3', 'tau4', 'tau5');

figure
power = dq.*tau;
for idx=1:5
    max_dq(idx) = max(dq(:,idx));
    max_tau(idx) = max(tau(:,idx));
    max_power(idx) = max(power(:,idx));
end
plot(time,power(:,1), '-', time, power(:,2), '--', time, power(:,3), '-.', time, power(:,4), ':', time, power(:,5), '-');
grid on
title('joint actuator power');
legend('power1', 'power2', 'power3', 'power4', 'power5');
disp(['max_q = ', num2str(max_dq)]);    
disp(['max_tau = ', num2str(max_tau)]);   
disp(['max_power = ', num2str(max_power)]);


