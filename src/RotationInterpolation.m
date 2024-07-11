clear
close all
clc

addpath('classes');
addpath(genpath('tools'));

r1 = rpy2r([30,21,35]);
r2 = rpy2r([20,40,30]);
[theta1,dir1] = tr2angvec(r1'*r2);
[theta2,dir2] = tr2angvec(r2*r1');
%%%dir1 and dir2 are the same vector, the diference between them is
%%%dir1 is descriped in r1 coordinate system, while dir2 is described in
%%%world coordinate system
q1 = UnitQuaternion(r1);
q2 = UnitQuaternion(r2);
np = 500;
the = linspace(0,theta2,np+1);
for idx=0:np
    qtmp = q1.interp(q2,idx/np);
    qtmp1 = quaternion(q2.double())*conj(quaternion(q1.double()));
    qtmp1 = qtmp1.^(idx/np);
    qtmp1 = qtmp1*quaternion(q1.double());
    qi1(:,idx+1) = qtmp.double();
    [th,v] = tr2angvec(qtmp.R());
    avi1(:,idx+1) = th*v;
    
    [th,v] = tr2angvec(angvec2r(the(idx+1),dir2)*r1);
    avi2(:,idx+1) = th*v;
end
figure(1); subplot(3,1,1); plot(avi1(1,:)); hold on; plot(avi2(1,:));
subplot(3,1,2); plot(avi1(2,:)); hold on; plot(avi2(2,:));
subplot(3,1,3); plot(avi1(3,:)); hold on; plot(avi2(3,:));
%%%slerp in quaternion is the same with interpolate in axis-angle


r0 = rpy2r([0,30,21]);
r3 = rpy2r([10,23,41]);
rf = rpy2r([21,23,76]);
q0 = UnitQuaternion(r0);
q3 = UnitQuaternion(r3);
qf = UnitQuaternion(rf);
quat = [q0,q1,q2,q3,qf];
for idx=1:length(quat)
    qmat(:,idx) = quat(idx).double();
end
for idx=0:np
    qtmp = quat_squad(qmat,idx/np);
    qu(:,idx+1) = qtmp;
    [th,v] = tr2angvec(quat2dcm(qtmp));
    avu(:,idx+1) = th*v;
end
figure(2); subplot(2,2,1); plot(qu(1,:));
subplot(2,2,2); plot(qu(2,:));
subplot(2,2,3); plot(qu(3,:));
subplot(2,2,4); plot(qu(4,:));
figure(3); subplot(3,1,1); plot(avu(1,:));
subplot(3,1,2); plot(avu(2,:));
subplot(3,1,3); plot(avu(3,:));

