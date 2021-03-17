clear
close all
clc

addpath('classes');
addpath('tools');

rbt = CleanRobot;
test_mode = 'ctrajcircle';
switch test_mode
    case 'dhmodel'
%% validation for robot model by simscape
%%there is lag in Simulink_PS Converter block because of the filter
sample_time = 0.001;
tf = 10;
t = [0:sample_time:tf]';
q(:,1) = pi/2*sin(2*pi*0.5*t);
q(:,2) = 0.5*sin(2*pi*0.5*t);
q(:,3) = pi/2*sin(2*pi*0.5*t);
q(:,4) =  0.5*sin(2*pi*0.5*t);
q(:,5) = pi/2*sin(2*pi*0.5*t);
q_offset = [0, 0.75, 0, 0, 0];
cart_pos = [];
for idx=1:size(t,1)
    q_in = q(idx,:) + q_offset;
    tmp_frame = rbt.FKSolve(q_in);
    end_rot = [tmp_frame.n, tmp_frame.o, tmp_frame.a];
    cart_pos = [cart_pos, tmp_frame.t];
end
sim('test_model');
sim_pos(:,1) = sim_pos_x;
sim_pos(:,2) = sim_pos_y;
sim_pos(:,3) = sim_pos_z;
%figure
figure(1)
subplot(2,1,1)
plot(t,sim_pos_x ,t, cart_pos(1,:),'--');
xlabel('time(s)');
ylabel('pos_x(m)');
legend('simscape model', 'mdh model');
grid on
subplot(2,1,2)
plot(t, sim_pos_x - cart_pos(1,:)');
legend('position error');

figure(2)
subplot(2,1,1)
plot(t,sim_pos_y ,t, cart_pos(2,:),'--');
xlabel('time(s)');
ylabel('pos_y(m)');
legend('simscape model', 'mdh model');
grid on
subplot(2,1,2)
plot(t, sim_pos_y - cart_pos(2,:)');
legend('position error');

figure(3)
subplot(2,1,1)
plot(t,sim_pos_z ,t, cart_pos(3,:),'--');
xlabel('time(s)');
ylabel('pos_z(m)');
legend('simscape model', 'mdh model');
grid on
subplot(2,1,2)
plot(t, sim_pos_z - cart_pos(3,:)');
legend('position error');


    case  'workspace'
%% validation for the workspace of cleanrobot    
rbt.PlotWorkspace;

    case 'jtrajpoly'
%% joint trajectory plan using polynomial trajectory
pos = [0,10, 5];
planner = PolyTrajPlanner(pos, [0, 1.2, 2], 3);
planner.PlotAVP(0.01);

    case 'jtrajlspb'
%% joint trajectory plan using lspb 
planner = LspbTrajPlanner([20,10], 2, 15, 10, 'limitvel');
planner.PlotAVP(0.01);

    case 'ctrajline'
%% cartesian line trajectory plan using lspb
pos_initial = [0, 0, 0];
pos_goal = [3, 4, 5];
line_length = norm(pos_goal-pos_initial);
planner = LspbTrajPlanner([0, line_length], 2, 5, 5, 'limitvel');
[p, v, ~] = planner.GenerateTraj(0.01);
pos = pos_initial+p'.*(pos_goal-pos_initial)/line_length;
vel = v'.*(pos_goal-pos_initial)/line_length;
plot2(pos);
grid on
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

figure
time = 0:0.01:2;
subplot(3,1,1)
plot(time, pos(:,1));
xlabel('x\_position');
subplot(3,1,2)
plot(time, pos(:,2));
xlabel('y\_position');
subplot(3,1,3)
plot(time, pos(:,3));
xlabel('z\_position');

figure
subplot(3,1,1)
plot(time, vel(:,1));
xlabel('x\_velocity');
subplot(3,1,2)
plot(time, vel(:,2));
xlabel('y\_velocity');
subplot(3,1,3)
plot(time, vel(:,3));
xlabel('z\_velocity');


    case 'ctrajcircle'
%% cartesian circle trajectory plan using lspb
pos1 = [0, -2, 0];
pos2 = [1, 0, 1];
pos3 = [0, 3, 3];
arcpath = ArcPathPlanner(pos1, pos2, pos3);
planner = LspbTrajPlanner([0, arcpath.theta], 2, 2, 2, 'limitvel');
[alp, alpv, alpa] = planner.GenerateTraj(0.01);
[pos, vel, acc] = arcpath.GenerateTraj(alp, alpv, alpa);

plot3([pos1(1); pos2(1); pos3(1)], [pos1(2); pos2(2); pos3(2)], [pos1(3); pos2(3); pos3(3)], 'ro');
grid on
xlabel('x'); ylabel('y'); zlabel('z');
hold on
plot3(pos(1,:), pos(2,:), pos(3,:), 'b-');

figure
time=0:0.01:2;
subplot(3,1,1)
plot(time, pos(1,:));
xlabel('x\_position');
subplot(3,1,2)
plot(time, pos(2,:));
xlabel('y\_position');
subplot(3,1,3)
plot(time, pos(3,:));
xlabel('z\_position');

figure
subplot(3,1,1)
plot(time, vel(1,:));
xlabel('x\_velocity');
subplot(3,1,2)
plot(time, vel(2,:));
xlabel('y\_velocity');
subplot(3,1,3)
plot(time,vel(3,:));
xlabel('z\_velocity');

end
