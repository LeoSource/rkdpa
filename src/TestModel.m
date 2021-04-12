clear
close all
clc

addpath('classes');
addpath('tools');

rbt = CleanRobot;
g_cycle_time = 0.001;
test_mode = 'jtrajpoly';
switch test_mode
    case 'dhmodel'
%% validation for robot model by simscape
tf = 10;
t = [0:g_cycle_time:tf]';
for idx=1:rbt.nlinks-1
    q0 = rand; a = rand; b = rand;
    q(:, idx) = q0+a*sin(t)+b*sin(2*t);
    qd(:, idx) = a*cos(t)+2*b*cos(2*t);
    qdd(:, idx) = -a*sin(t)-4*b*sin(2*t);
end
cart_pos = [];
for idx=1:size(t,1)
    q_in = [q(idx, 1:3), 0, q(idx, 4)];
    tmp_frame = rbt.FKSolve(q_in);
    end_rot = [tmp_frame.n, tmp_frame.o, tmp_frame.a];
    cart_pos = [cart_pos, tmp_frame.t];
end
sim('simulink/test_model');
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
q = [3, -2, -5, 0, 6, 12, 3];
% t = [0, 5, 7, 8, 10, 15, 18];
t = 18;
planner = PolyTrajPlanner(q, t, 3);
% planner.PlotAVP(0.01);

planner1 = CubicSplinePlanner(q, t, 'clamped', [0, 0]);
% planner1.SetTimeOptimizedStyle('middle');
% planner1.SetSmoothWeight(0.7);
planner1.SetSmoothTolerance(1.9);
planner1.PlotAVP(0.001);

    case 'jtrajlspb'
%% joint trajectory plan using lspb 
% planner = LspbTrajPlanner([20,10], 2, 16, 10, 'limitvel');
% planner.PlotAVP(0.01);

trajplanner = DoubleSVelTrajPlanner([0, 10],[0, 0], 5, 10, 30);
trajplanner.SetPhaseDuration(5, 1/3, 1/5);
trajplanner.PlotMotion(0.001, 'pvaj');

% [p, v, a] = planner.GenerateMotion(2)
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

    case 'ctrajarc'
%% cartesian arc path trajectory plan using lspb
pos1 = [0, -2, 0];
pos2 = [1, 0, 1];
pos3 = [0, 3, 3];
arcpath = ArcPathPlanner(pos1, pos2, pos3, 'arc');
planner = LspbTrajPlanner([0, arcpath.theta], 2, 2, 2, 'limitvel');
[alp, alpv, alpa] = planner.GenerateTraj(0.01);
[pos, vel, acc] = arcpath.GenerateTraj(alp, alpv, alpa);

f1 = plot3([pos1(1); pos2(1); pos3(1)], [pos1(2); pos2(2); pos3(2)], [pos1(3); pos2(3); pos3(3)], 'ro');
grid on
xlabel('x'); ylabel('y'); zlabel('z');
hold on
f2 = plot3(pos(1,:), pos(2,:), pos(3,:), 'b--');
% comparison test with Cpp
pos_cpp = load('./data/ctrajarc_pos.csv');
np = length(pos_cpp)/3;
pos_cpp = reshape(pos_cpp, 3, np);
f3 = plot3(pos_cpp(1,:), pos_cpp(2,:), pos_cpp(3,:), 'r-');
legend([f2, f3], 'matlab\_data', 'cpp\_data');

arcpath.PlotTraj(alp, alpv, alpa, 2, 0.01);

    case 'ctrajcircle'
%% cartesian circle path trajectory plan using lspb
center = [1;2;3]; n_vec = [1;0;0]; radius = 0.5;
circlepath = ArcPathPlanner(center, n_vec, radius, 'circle');
planner = LspbTrajPlanner([0, circlepath.theta], 2, 4, 2, 'limitvel');
[alp, alpv, alpa] = planner.GenerateTraj(0.01);
[pos, vel, acc] = circlepath.GenerateTraj(alp, alpv, alpa);

plot3(pos(1,:), pos(2,:), pos(3,:));
grid on
xlabel('x'); ylabel('y'); zlabel('z');
circlepath.PlotTraj(alp, alpv, alpa, 2, 0.01);

% comparison test with Cpp
[alp, alpv, alpa] = planner.GenerateMotion(2);
[pos, vel, acc] = circlepath.GenerateMotion(alp, alpv, alpa)

    case 'ctrajarctrans'
%% cartesian trajectory for points using arc to transmit between 2 line paths
% pos1 = [0.7, 0.8, 1]; pos2 = [-0.7, 0.8, 1]; pos3 = [-0.7, 0.8, 2.4]; pos4 = [0.7, 0.8, 2.4];
pos1 = [0.8, 0.2, 0.7]; pos2 = [-0.8, 0.2, 0.7]; pos3 = [-0.8, 0.8, 0.7]; pos4 = [0.8, 0.8, 0.7];
radius = 0.04;
tf = 60; dt = 0.01;
% via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 0.1, [-1,0,1]);
% via_pos = CalcRectanglePath1([pos1', pos2', pos3', pos4'], 0.1, 's');
via_pos = CalcRectanglePath2([pos1', pos2', pos3', pos4'], 16, 'm');
cpath = ArcTransPathPlanner(via_pos, radius);
planner = LspbTrajPlanner([0, cpath.distance], tf, 0.5, 2, 'limitvel');
[s, sv, sa] = planner.GenerateTraj(dt);
[pos, vel, acc] = cpath.GenerateTraj(s, sv, sa);

plot3(via_pos(1,:), via_pos(2,:), via_pos(3,:), 'ro');
grid on
xlabel('x'); ylabel('y'); zlabel('z');
hold on
plot3(pos(1,:), pos(2,:), pos(3,:), 'b-');
cpath.PlotTraj(s, sv, sa, tf, dt);

% comparison test with Cpp
[s, sv, sa] = planner.GenerateMotion(54);
[pos, vel, acc] = cpath.GenerateMotion(s, sv, sa)

    case 'ctrajpoly'
%% cartesian trajectory plan using polynomial trajectory
% it is very important to define the time sequence for every via position
pos1 = [0.8, 0.2, 0.7]; pos2 = [-0.77, 0.2, 0.7]; pos3 = [-0.77, 0.8, 0.7]; pos4 = [0.8, 0.8, 0.7];
via_pos = CalcRectanglePath1([pos1', pos2', pos3', pos4'], 0.1, 'm');
for idx=1:3
    planner(idx) = PolyTrajPlanner(via_pos(idx,:), 2, 3);
    [pos(idx,:), ~, ~] = planner(idx).GenerateTraj(0.01);
end
% planner(1).PlotAVP(0.01);
figure
plot3(via_pos(1,:), via_pos(2,:), via_pos(3,:), 'ro');
grid on
xlabel('x'); ylabel('y'); zlabel('z');
hold on
plot3(pos(1,:), pos(2,:), pos(3,:));

    case 'ctrajbspline'
%% cartesian trajectory plan using B-spline
%pos1 = [0.7, 0.8, 1]; pos2 = [-0.7, 0.8, 1]; pos3 = [-0.7, 0.8, 2.4]; pos4 = [0.7, 0.8, 2.4];
pos1 = rand(2,1)*1; pos2 = rand(2,1)*2; pos3 = rand(2,1)*3; 
pos4 = rand(2,1)*4; pos5 = rand(2,1)*5; pos6 = rand(2,1)*6;
via_pos = [pos1, pos2, pos3, pos4, pos5, pos6];
%via_pos = CalcRectanglePath1([pos1', pos2', pos3', pos4'], 0.1, 's');
knots_vec = [0,0,0,0,1/3,2/3,1,1,1,1];%clamped table
% knots_vec = 0:0.1:1;
pos = [];
for u=0:0.001:0.999
    [p, coef] = BSpline(via_pos(:,1:6),knots_vec, u);
    pos = [pos ,p];
end
figure
% plot3(via_pos(1,1:6),via_pos(2,1:6),via_pos(3,1:6), 'ro');
plot(via_pos(1,:), via_pos(2,:), 'ro');
grid on
xlabel('x'); ylabel('y'); zlabel('z');
hold on
plot(pos(1,:), pos(2,:));
% plot3(pos(1,:), pos(2,:), pos(3,:));

    case 'jacobian'
%% test for jacobian
mdh_table = [      0,   0,   0,       0,    0,   0
                pi/2,   0,   0,       0,    1,   0 
                    0,    0,   0,   pi/2,    0,   pi/2
                pi/2,    0,   0,   pi/2,   1,   0
                    0,    0,   0,    pi/2,   0,   0
                    0,   0.2,  0,   -pi/6-pi/2, 0, 0];
test_rbt = SerialLink(mdh_table, 'modified', 'name', 'test_rbt');
q = rand(1,5);  test_q = [q, 0];
dq = rand(5,1); test_dq = [dq; 0];
spatial_vel = rbt.CalcJaco(q)*dq
test_jaco = test_rbt.jacob0(test_q);
test_vel = test_jaco*test_dq
jaco_err = norm(rbt.CalcJaco(q)-test_jaco(:,1:5))

q= [0.3,0.98,-0.91,0.91,0.56];
jaco = rbt.CalcJaco(q)
    case 'redudantsolve'
%% redundant solve for robot work
% % pos1 = [0.7, 0.8, 1]; pos2 = [-0.7, 0.8, 1]; pos3 = [-0.7, 0.8, 2.4]; pos4 = [0.7, 0.8, 2.4];
% pos1 = [0.8, 0.2, 0.7]; pos2 = [-0.77, 0.2, 0.7]; pos3 = [-0.77, 0.8, 0.7]; pos4 = [0.8, 0.8, 0.7];
% radius = 0.04;
% tf = 60; dt = 0.01;
% % via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 0.1, [-1,0,1]);
% via_pos = CalcRectanglePath1([pos1', pos2', pos3', pos4'], 0.1, 'm');
% cpath = ArcTransPathPlanner(via_pos, radius);
% planner = LspbTrajPlanner([0, cpath.distance], tf, 0.5, 2, 'limitvel');
% [s, sv, sa] = planner.GenerateTraj(dt);
% [pos, vel, acc] = cpath.GenerateTraj(s, sv, sa);

    case 'mirrortask'
%%  comparison with cpp
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

sim_q = [];
pos = pos';
for idx=1:size(pos,1)    
    tmp_q = rbt.IKSolve(pos(idx,:), ik_option, alpha(idx));    
    sim_q = [sim_q; tmp_q];
end
t = 0:dt:tf;
tt = 0:g_cycle_time:tf;

q_cpp = load('./data/mirrortask_jpos.csv');
q_cpp = reshape(q_cpp, rbt.nlinks, length(tt));
for idx=1:rbt.nlinks
    figure(idx)
    plot(t, sim_q(:,idx), 'b--', tt, q_cpp(idx, :), 'r-');
    xlabel('time'); ylabel(['q', num2str(idx)]);
    legend('matlab\_data', 'cpp\_data');
end


end





