clear
close all
clc

addpath('classes');
addpath(genpath('tools'));

global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
g_jvmax = [pi/8, pi/8, 0.8*pi/4, pi/4, pi/4, pi/4];
g_jamax = [pi/4, pi/4, pi/2, pi/4, pi/4, pi/4];
g_cvmax = [0.4, 0.6]; g_camax = [0.8, 1.2];
g_cycle_time = 0.005;

rbt = CreateRobot();
q0 = deg2rad([0,0,0,0,-90,0]');
compare_plan = false;
taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                            g_cvmax,g_camax,compare_plan);
pos_rpy1 = [0.8,0,0.23,-pi,-pi/6,0]';
pos_rpy2 = [0.3,0.2,0.23,-pi,-pi/6,0]';
pos_rpy3 = [0.8,0,0.1,-pi,-pi/6,0]';
pos_rpy4 = [0.2,0.6,0.23,-pi,-pi/6,0]';
via_posrpy = [pos_rpy1,pos_rpy2,pos_rpy3,pos_rpy4];
taskplanner.AddTraj(via_posrpy,'cartesian',true);

cpos=[]; cvel=[]; cacc=[];
jpos=[]; jvel=[]; jacc=[];
while ~taskplanner.task_completed
    [cp,cv,ca,jp,jv,ja] = taskplanner.GenerateBothMotion();
    cpos = [cpos,cp]; cvel = [cvel,cv]; cacc = [cacc,ca];
    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
end

figure
plot2(cpos(1:3,:)', 'r--'); hold on;%plot2(cpos_sim', 'k');
plot2(via_posrpy(1:3,:)', 'bo'); axis equal;%axis square vis3d;
%     PlotRPY(cpos, 60); hold off;
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');




%% robot description
function rbt = CreateRobot()
    d1 = 0.048; a2 = 0.51; a3 = 0.51;
    d4 = 0.11; d5 = 0.08662; d6 = 0.035;
    mdh_table = [0, d1, 0, 0, 0, 0;...
                        0, 0, 0, -pi/2, 0, -pi/2;...
                        0, 0, a2, 0, 0, pi/2;...
                        0, d4, a3, 0, 0, -pi/2;...
                        0, d5, 0, -pi/2, 0, 0;...
                        0, d6, 0, pi/2, 0, 0];
    % pose_tool = SE3(rotx(-10), [0,0,0.116]);
%     tool_toiletlid = SE3(rotx(0), [0,-0.035,0.23]);
    tool_toiletlid = SE3(rotx(0), [0,0,0]);
    qmin = [-pi, -pi/2, -4*pi/3, -pi, -pi, -2*pi]';
    qmax = [pi, pi/2, pi/3, pi, pi, 2*pi]';
    rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot', 'tool',tool_toiletlid);
    rbt.qlim(:,1) = qmin; rbt.qlim(:,2) = qmax;
end