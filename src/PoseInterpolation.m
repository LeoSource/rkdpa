clear
close all
clc

addpath('classes');
addpath(genpath('tools'));

p1 = [1,2,3]';
p2 = [2,3,1]';
p3 = [-2,5,-1]';
p4 = [5,-1,0]';
pmat = [p1,p2,p3,p4];

pi = [];
for u=0:0.01:1
    pi = [pi,quad_pos(pmat,u)];
end
plot2(pmat','ro'); grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
hold on; plot2(pi','b-');