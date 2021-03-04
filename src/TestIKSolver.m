clear
close all
clc

%% tests for iksolver
% to do: random position to test
rbt = CleanRobot;

q = rbt.IKSolve([-0.7, 0.7, 1.8], 'vertical', 0);
pos = rbt.FKSolve(q).t


q2 = rbt.IKSolve([-0.6, 0.5, 0.8], 'horizontal', 0);
pos = rbt.FKSolve(q2).t


