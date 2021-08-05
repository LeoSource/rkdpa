clear
close all
clc

addpath('classes');
addpath('tools');
%% define robot model with MDH method
global g_jvmax g_jamax g_cvmax g_camax g_stowed_pos g_cycle_time
g_jvmax = [pi/4, pi/4, pi/4, pi/4, pi/4, pi/4];
g_jamax = [pi/2, pi/2, pi/2, pi/2, pi/2, pi/2];
g_cvmax = [0.15, 0.15]; g_camax = [0.3, 0.3];
g_stowed_pos = [0;0;0;0;-pi/2;0];
g_cycle_time = 0.005;

d1 = 0.048; a2 = 0.51; a3 = 0.51;
d4 = 0.11; d5 = 0.08662; d6 = 0.035;
mdh_table = [0, d1, 0, 0, 0, 0;...
                    0, 0, 0, -pi/2, 0, -pi/2;...
                    0, 0, a2, 0, 0, pi/2;...
                    0, d4, a3, 0, 0, -pi/2;...
                    0, d5, 0, -pi/2, 0, 0;...
                    0, d6, 0, pi/2, 0, 0];
pose_tool = SE3(rotx(-10), [0,0,0.116]);
qmin = [-pi, -pi/2, -4*pi/3, -pi, -pi, -2*pi]';
qmax = [pi, pi/2, pi/3, pi, pi, 2*pi]';
rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot', 'tool',pose_tool);
simu_mode = 'workspace';
switch simu_mode
    case 'workspace'
%% plot workspace
nump = 5000;
tmp_q = rand(1,6,nump);
q = qmin'+tmp_q.*(qmax'-qmin');
for idx=1:nump
    pos(:,idx) = rbt.fkine(q(:,:,idx)).t;
end

figure
plot3(pos(1,:), pos(2,:), pos(3,:), 'r.', 'MarkerSize', 3);
grid on
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
    
    case 'mirror'
%% simulation all task
compare_cpp = 0;
compare_plan = 1;
dt = 0.01;
%%%%% joint plan for grasp&interim%%%%%
q0 = [0,0,0,0,-pi/2,0]';
q1 = [0,145,-240,90,-90,0]'*pi/180;
q2 = [0,-35,50,-100,-90,0]'*pi/180;
taskplanner = TaskTrajPlanner(rbt,q0,compare_plan);
taskplanner.AddTraj([q1,q2], 'joint', 1);
%%%%% mirror task pose %%%%%
pos1 = [0.5,0,1]'; pos2 = [0.65,0,1]'; pos3 = [0.65,0,0.6]';
rpy1 = [0,pi/2,pi/2]'; rpy2 = [0,pi/2,pi/2]'; rpy3 = [0,pi/2,pi/2]';
via_pos1 = [[pos1;rpy1],[pos2;rpy2],[pos3;rpy3]];
taskplanner.AddTraj(via_pos1, 'cartesian', 0);
%%%%% table task pose %%%%%
q1 = [0,-35,50,-105,-90,0]'*pi/180;
pos1 = [0.8,0,0.23]'; pos2 = [0.8,0.4,0.23]'; pos3 = [0.4,0.4,0.23]'; pos4 = [0.4,0,0.23]';
rpy1 = [0,pi/6,-pi]'; rpy2 = [0,0,-5*pi/6]'; rpy3 = [0,-pi/6,-pi]'; rpy4 = [0,0,-7*pi/6]';
via_pos2 = [[pos1;rpy1], [pos2;rpy2], [pos3;rpy3], [pos4;rpy4]];
taskplanner.AddTraj(q1, 'joint', 1);
taskplanner.AddTraj(via_pos2, 'cartesian', 1);
%%%%% toilet task pose %%%%%
q1 = [0,-35,50,-105,-90,0]'*pi/180;
q2 = [0.35,0.52,0.52,-1.1,-1.4,0.52]';
taskplanner.AddTraj([q1,q2], 'joint', 1);
pos1 = [0.5297,0.2516,-0.4929]'; pos2 = [0.5208,0.2905,-0.5424]'; pos3 = [0.6039,0.4115,-0.544]';
pos4 = [0.7013,0.3362,-0.5544]'; pos5 = [0.6396,0.2582,-0.567]';
rpy1 = [-106,0.3,-175]'*pi/180; rpy2 = [-104.7,-0.8,-170]'*pi/180; rpy3 = [-114.8,3.7,-168]'*pi/180;
rpy4 = [-113,8.6,-177.7]'*pi/180; rpy5 = [-109.2,4.4,-179.4]'*pi/180;
via_pos3 = [[pos1;rpy1], [pos2;rpy2], [pos3;rpy3], [pos4;rpy4], [pos5;rpy5]];
taskplanner.AddTraj(via_pos3, 'bspline', 'interpolation');


[cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
%% plot and compare with cpp data
figure
plot2(cpos(1:3,:)', 'r--'); hold on;plot2(cpos_sim', 'k');
plot2([via_pos1(1:3,:), via_pos2(1:3,:)]', 'bo'); axis square vis3d;
PlotRPY(cpos, 60); hold off;
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

t = 0:dt:dt*(size(jpos,2)-1);
figure
plot(t,jpos(1,:),'-', t, jpos(2,:), '--', t, jpos(3,:), '-.', t, jpos(4,:), ':', t, jpos(5,:), '-', t,jpos(6,:),'k');
grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');

if compare_cpp
    q_cpp = load('./data/mirrortask_jpos1.csv');
    q_cpp = reshape(q_cpp, rbt.n, []);
    tt = g_cycle_time*[0:size(q_cpp,2)-1];
    for idx=1:rbt.n
        figure
        plot(t, jpos(idx,:), 'b--', tt, q_cpp(idx,:), 'r-');
        xlabel('time'); ylabel(['q', num2str(idx)]); grid on;
        legend('matlab\_data', 'cpp\_data');
    end
end





end