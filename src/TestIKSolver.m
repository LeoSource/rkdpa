clear
close all
clc

addpath('classes');
addpath('tools');

%% tests for iksolver
% to do: random position to test
rbt = CleanRobot;

cmd_pos = [rand, rand, rand+1]';
% cmd_pos = [0.1, 0.8, 1.7]';
q = rbt.IKSolve(cmd_pos, 'q2first', 0);
pos = rbt.FKSolve(q).t;
disp(['pos_err = ', num2str(norm(cmd_pos-pos))]);

cmd_pos = [0, 0.8, rand]';
q2 = rbt.IKSolve(cmd_pos, 'q3first0', 0);
pos2 = rbt.FKSolve(q2).t;
disp(['pos_err = ', num2str(norm(cmd_pos-pos2))]);

circle_params.origin = [rand; rand; rand];
circle_params.radius = 0.3;
circle_params.alpha = 45*pi/180;
cmd_pos = [-sin(circle_params.alpha)*circle_params.radius, circle_params.origin(2)+cos(circle_params.alpha)*circle_params.radius, circle_params.origin(3)]
q3 = rbt.IKSolve(cmd_pos, 'q3firstn', circle_params.alpha);
pos3 = rbt.FKSolve(q3).t;


frame = SE3(transl(cmd_pos)*trotx(30)*trotz(20));
[q4, qerr, exitflag] = rbt.IKSolveCon(frame, rand(1,5));
pos4 = rbt.FKSolve(q4).t
