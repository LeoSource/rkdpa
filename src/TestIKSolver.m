clear
close all
clc

%% tests for iksolver
% to do: random position to test
rbt = CleanRobot;

q = rbt.IKSolve([0.499770431588613, 0.7, 1.6], 'vertical', 0);
pos = rbt.FKSolve(q).t;


q2 = rbt.IKSolve([-0.2, 0.7, 1.60], 'horizontal', 0);
pos = rbt.FKSolve(q2).t;

circle_params.origin = [0; 0.5; 0.5];
circle_params.radius = 0.3;
circle_params.alpha = 10*pi/180;
cmd_pos = [-sin(circle_params.alpha)*circle_params.radius, circle_params.origin(2)+cos(circle_params.alpha)*circle_params.radius, circle_params.origin(3)]
q3 = rbt.IKSolve([-sin(circle_params.alpha)*circle_params.radius, circle_params.origin(2)+cos(circle_params.alpha), circle_params.origin(3)], 'circle', circle_params);
pos = rbt.FKSolve(q3).t
