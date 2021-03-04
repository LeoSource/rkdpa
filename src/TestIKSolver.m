% clear
% close all
% clc

%% tests for iksolver
% to do: random position to test
rbt = CleanRobot;

q = rbt.IKSolve([0.499770431588613, 0.7, 1.6], 'vertical', 0);
pos = rbt.FKSolve(q).t


q2 = rbt.IKSolve([-0.2, 0.7, 1.60], 'horizontal', 0);
pos = rbt.FKSolve(q2).t


