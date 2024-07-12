clear
close all
clc

addpath('classes');
addpath(genpath('tools'));

%% interpolate position in Cartesian sapce
clear
close all
clc

p1 = [1,2,3]';
p2 = [2,3,1]';
p3 = [-2,5,-1]';
p4 = [5,-1,0]';
pmat = [p1,p2,p3,p4];

pquad = [];
for u=0:0.01:1
    pquad = [pquad,quad_pos(pmat,u)];
end
plannerx = PolyTrajPlanner(pmat(1,:),[0,1,2,3],[0,0],3);
plannery = PolyTrajPlanner(pmat(2,:),[0,1,2,3],[0,0],3);
plannerz = PolyTrajPlanner(pmat(3,:),[0,1,2,3],[0,0],3);
ppoly3(1,:) = plannerx.GenerateTraj(0.01);
ppoly3(2,:) = plannery.GenerateTraj(0.01);
ppoly3(3,:) = plannerz.GenerateTraj(0.01);

plot2(pmat','ro'); grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
hold on; plot2(pquad','b-'); plot2(ppoly3','g');


%% interpolate pose in Cartesian space
clear
close all
clc

p1 = [1,2,3]';
p2 = [2,3,1]';
p3 = [-2,5,-1]';
p4 = [5,-1,0]';
pmat = [p1,p2,p3,p4];
q1 = dcm2quat(rpy2r([10,32,1]));
rpy1 = tr2rpy(quat2dcm(q1),'xyz')';
q2 = dcm2quat(rpy2r([-2,40,20]));
rpy2 = tr2rpy(quat2dcm(q2),'xyz')';
q3 = dcm2quat(rpy2r([40,30,-30]));
rpy3 = tr2rpy(quat2dcm(q3),'xyz')';
q4 = dcm2quat(rpy2r([2,6,0]));
rpy4 = tr2rpy(quat2dcm(q4),'xyz')';
qmat = [q1;q2;q3;q4]';
rpymat = [rpy1,rpy2,rpy3,rpy4];
prmat = [pmat;rpymat];
pqmat = [pmat;qmat];

pq_quad = []; posrpy = [];
for u=0:0.01:1
    pq = quad_pose(pqmat,u);
    pq_quad = [pq_quad,pq];
    pos = pq(1:3);
    rpy = tr2rpy(quat2dcm(pq(4:7)'),'xyz');
    posrpy = [posrpy,[pos;rpy']];
end

figure; PlotRPY(posrpy,1,0.5); xlabel('X'); ylabel('Y'); zlabel('Z');
 hold on; plot2(posrpy(1:3,:)','k'); plot2(pmat','ro'); PlotRPY(prmat,1,1.5);


