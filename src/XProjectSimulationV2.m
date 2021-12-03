clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
%% define robot model with MDH method
global g_jvmax g_jamax g_cvmax g_camax g_stowed_pos g_cycle_time
g_jvmax = [pi/8, pi/8, 0.8*pi/4, pi/4, pi/4, pi/4];
g_jamax = [pi/4, pi/4, pi/2, pi/4, pi/4, pi/4];
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
% pose_tool = SE3(rotx(-10), [0,0,0.116]);
tool_toiletlid = SE3(rotx(0), [0,-0.035,0.23]);
qmin = [-pi, -pi/2, -4*pi/3, -pi, -pi, -2*pi]';
qmax = [pi, pi/2, pi/3, pi, pi, 2*pi]';
rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot', 'tool',tool_toiletlid);
rbt.qlim(:,1) = qmin; rbt.qlim(:,2) = qmax;
simu_mode = 'toilet_lid';
switch simu_mode
    case 'workspace'
%% plot workspace
joint_plot = 0;
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
    
    case 'common'
%% simulation all task
joint_plot = 1;
compare_cpp = 1;
compare_plan = 1;
dt = 0.01;

q0 =  [-38,-27,18,-7,-82,-82]'*pi/180;
taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                            g_cvmax,g_camax,compare_plan);
vision_pos = [0.649857,  0.612921, 0.662765, 0.745594, 0.839413, 0.876876, 0.811265, 0.732626, 0.7157;
                    -0.106503, -0.00835975, 0.0869856, 0.112016, 0.0893813, -0.0304303, -0.126356, -0.142894, -0.09246;
                    -0.533577, -0.542969, -0.53932, -0.526466, -0.506219, -0.491052, -0.50203, -0.512606, -0.6222];
% via_posrpy = CalcViapos(vision_pos(:,2:end), 'toilet');
via_posrpy = PlanToiletInlierPath(vision_pos,10*pi/180,'left');
% pos1 = [0.5297,0.2516,-0.4929]'; pos2 = [0.5208,0.2905,-0.5424]'; pos3 = [0.6039,0.4115,-0.544]';
% pos4 = [0.7013,0.3362,-0.5544]'; pos5 = [0.6396,0.2582,-0.567]';
% rpy1 = [-106,0.3,-175]'*pi/180; rpy2 = [-104.7,-0.8,-170]'*pi/180; rpy3 = [-114.8,3.7,-168]'*pi/180;
% rpy4 = [-113,8.6,-177.7]'*pi/180; rpy5 = [-109.2,4.4,-179.4]'*pi/180;
% via_posrpy = [[pos1;rpy1], [pos2;rpy2], [pos3;rpy3], [pos4;rpy4], [pos5;rpy5]];
%%%%%simple test%%%%%
% posrpy1 = [0,0,0,0,0,0]';
% pos2 = [0.5,0.5,0.5]'; rpy2 = tr2rpy(rotx(90), 'xyz')'; posrpy2 = [pos2;rpy2];
% pos3 = [1,1,1]'; rpy3 = tr2rpy(roty(90), 'xyz')'; posrpy3 = [pos3;rpy3];
% via_posrpy = [posrpy1, posrpy2, posrpy3];
% taskplanner.AddTraj(via_posrpy, 'bspline', 'interpolation');
taskplanner.AddTraj(via_posrpy, 'cartesian', 0);

[cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
% [cpos,cve,cacc] = taskplanner.GenerateCartTraj(dt);

figure
plot2(cpos(1:3,:)', 'r--'); hold on;%plot2(cpos_sim', 'k');
plot2(via_posrpy(1:3,:)', 'bo'); axis equal;%axis square vis3d;
PlotRPY(cpos, 60); hold off;
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

    case 'toilet_lid'
%% simulate toilet lifting
joint_plot = 1;
compare_cpp = 0;
compare_plan = 1;
dt = 0.01;
toilet_lid = 'close';
% q0 = [0,-35, 50, -100, -90, 0]'*pi/180;
% q0 = [-60,50,40,-50,-20,-90]'*pi/180;
if strcmp(toilet_lid,'open')
    q0 = [-40,65,40,-5,55,-180]'*pi/180;
elseif strcmp(toilet_lid,'close')
    q0 = [0,-35,20,65,-90,0]'*pi/180;
end
taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                            g_cvmax,g_camax,compare_plan);
toilet_lid = 'close';
if strcmp(toilet_lid,'open')
    a = [0.89, 0.178, -0.3627]'; b = [0.87426, -0.19926, -0.36788]'; c = [0.5006, -0.1645, -0.3838]';
    vision_pos = [a,b,c];
    via_posrpy = PlanToiletlidPath(vision_pos, 110*pi/180, (-0-100)*pi/180, 0*pi/180, 0.05);
elseif strcmp(toilet_lid,'close')
    a = [0.87, 0.178, -0.3627]'; b = [0.85426, -0.19926, -0.36788]'; c = [0.9232, -0.079565, 0.066379]';
    vision_pos = [a,b,c];
    via_posrpy = PlanToiletlidPath(vision_pos, -pi/3, 0*pi/180, 150*pi/180, 0.05);
    tmp_jpos = [0,-35,20,0,-70,150]'*pi/180;
    taskplanner.AddTraj(tmp_jpos, 'joint', 1);
    taskplanner.AddTraj(via_posrpy(:,1), 'cartesian', 0);
    taskplanner.AddTraj(via_posrpy(:,2:end), 'arc', 0);
    taskplanner.AddTraj(fliplr(via_posrpy(:,2:end)), 'arc', 0);
end

if compare_plan
    [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
else
    [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
end

figure
plot2(cpos(1:3,:)', 'r--'); hold on;
plot2(via_posrpy(1:3,:)', 'bo'); plot2(vision_pos','r*'); axis equal;
PlotRPY(cpos, 50);
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

    case 'mirror'
%% simulate scrape mirror 
joint_plot = 1;
compare_cpp = 0;
compare_plan = 1;
dt = 0.01;
q0 = [0, -35, 50, -100, -90, 0]'*pi/180;
taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                            g_cvmax,g_camax,compare_plan);
rectplanner = QuadranglePlanner;
p1 = [0.8,-0.2,0.45]'; p2 = [0.8,0.4,0.45]'; p3 = [0.8,0.4,0.81]'; p4 = [0.8,-0.2,0.81]';
vision_pos = [p1,p2,p3,p4];
% via_posrpy = CalcMirrorPath(vision_pos, 0.15, 10*pi/180, 50*pi/180);
% via_posrpy = CalcMirrorPath_Normal(vision_pos, 0.15, 60*pi/180, 0.08, 'right');
via_posrpy = rectplanner.PlanMirror(vision_pos);
taskplanner.AddTraj(via_posrpy, 'cartesian', 0);

if compare_plan
    [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
else
    [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
end

figure
plot2(cpos(1:3,:)', 'r--');hold on;
plot2(via_posrpy(1:3,:)', 'bo'); plot2(vision_pos', 'r*'); axis equal;
PlotRPY(cpos, 60);
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

    case 'table'
%% simulate rectangle table or ground zones
joint_plot = 1;
compare_cpp = 0;
compare_plan = 1;
dt = 0.01;
q0 = [0, -35, 50, -100, -90, 0]'*pi/180;
taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                            g_cvmax,g_camax,compare_plan);
rectplanner = QuadranglePlanner;
p1 = [0.45,-0.25,-0.75]'; p2 = [0.45,0.25,-0.75]'; p3 = [0.8,0.25,-0.75]'; p4 = [0.8,-0.25,-0.75]';
vision_pos = [p1,p2,p3,p4];
via_posrpy = rectplanner.PlanGround(vision_pos);
taskplanner.AddTraj(via_posrpy, 'cartesian', 0);

if compare_plan
    [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
else
    [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
end

figure
plot2(cpos(1:3,:)', 'r--');hold on;
plot2(via_posrpy(1:3,:)', 'bo'); plot2(vision_pos', 'r*'); axis equal;
PlotRPY(cpos, 100);
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

    case 'fric_test'
%% simulate friction test trajectory
joint_plot = 0;
compare_cpp = 0;
compare_plan = 0;
dt = 0.01;
q0 = [0,45,45,0,-10,0]'*pi/180';
taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                g_cvmax,g_camax,compare_plan);
det_q = {[0,0,0,0,10,0]'*pi/180,[0,0,0,0,20,0]'*pi/180,[0,0,0,0,30,0]'*pi/180,...
        [0,0,0,0,40,0]'*pi/180,[0,0,0,0,50,0]'*pi/180,[0,0,0,0,60,0]'*pi/180,...
        [0,0,0,0,70,0]'*pi/180,[0,0,0,0,80,0]'*pi/180,[0,0,0,0,90,0]'*pi/180};
q00 = [0,45,45,0,0,0]'*pi/180';
jpos = []; jvel = [];
vel_scale = [0.05,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0];
for idx=1:length(vel_scale)
    if idx < 9
        detq_pos = det_q{idx};
        detq_neg = -det_q{idx+1};
    else
        detq_pos = det_q{9};
        detq_neg = -det_q{9};
    end
    q1 = q00+detq_pos;
    q2 = q00+detq_neg;
    taskplanner.Reset(q0);
    taskplanner.AddTraj([q1,q2],'joint',1,vel_scale(idx),1);
    [jpos_tmp,jvel_tmp,~] = taskplanner.GenerateJointTraj(dt);
    jpos = [jpos,jpos_tmp]; jvel = [jvel,jvel_tmp];
    q0 = jpos(:,end);
end
plot(jpos(5,:)); grid on; hold on;
plot(jvel(5,:)); hold off;


end

%% plot for each joint position
if joint_plot
    t = 0:dt:dt*(size(jpos,2)-1);
    figure
    plot(t,jpos(1,:),'-', t, jpos(2,:), '--', t, jpos(3,:), '-.', t, jpos(4,:), ':', t, jpos(5,:), '-', t,jpos(6,:),'k');
    grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');

    if compare_cpp
        q_cpp = load('./data/mirrortask_jpos1.csv');
        q_cpp = reshape(q_cpp, 6, []);
        tt = g_cycle_time*[0:size(q_cpp,2)-1];
        for idx=1:rbt.n
            figure
            plot(t, jpos(idx,:), 'b--', tt, q_cpp(idx,:), 'r-');
            xlabel('time'); ylabel(['q', num2str(idx)]); grid on;
            legend('matlab\_data', 'cpp\_data');
        end
    end
end
