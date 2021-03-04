clear
close all
clc

%% tests for iksolver
% to do: random position to test
rbt = CleanRobot;

q = rbt.IKSolve([0.499770431588613, 0.7, 1.6], 'q2first', 0);
pos = rbt.FKSolve(q).t;


q2 = rbt.IKSolve([0, 0.8, 0.5], 'q3first', 0);
pos2 = rbt.FKSolve(q2).t;

circle_params.origin = [0; 0.5; 0.5];
circle_params.radius = 0.3;
circle_params.alpha = 20*pi/180;
cmd_pos = [-sin(circle_params.alpha)*circle_params.radius, circle_params.origin(2)+cos(circle_params.alpha)*circle_params.radius, circle_params.origin(3)];
q3 = rbt.IKSolve(cmd_pos, 'q3first', circle_params.alpha);
pos3 = rbt.FKSolve(q3).t;


frame = SE3(transl(0.6, 0.8, 1.5));
[q4, qerr, exitflag] = rbt.IKSolveCon(frame, q2);
pos4 = rbt.FKSolve(q4).t
