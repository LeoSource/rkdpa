clear
close all
clc

addpath('classes');
addpath('tools');

d1 = 0.048; a2 = 0.51; a3 = 0.51;
d4 = 0.11; d5 = 0.08662; d6 = 0.035;
mdh_table = [0, d1, 0, 0, 0, 0;...
                    0, 0, 0, -pi/2, 0, -pi/2;...
                    0, 0, a2, 0, 0, pi/2;...
                    0, d4, a3, 0, 0, -pi/2;...
                    0, d5, 0, -pi/2, 0, 0;...
                    0, d6, 0, pi/2, 0, 0];
rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot');
q = rand(6,1);
rbt.fkine(q)