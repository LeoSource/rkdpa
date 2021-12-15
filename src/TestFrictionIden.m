clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
dt = 0.005;
%% load joint test data
nj = 6;
td = load('./data/test_data_1214_155413.csv');
jpos_idx = 1; jvel_idx = 2; jtor_idx = 3;
jpos = td(:,1:jpos_idx*nj);
jvel = td(:,nj+1:jvel_idx*nj);
jtor = td(:,2*nj+1:jtor_idx*nj);
t = 0:dt:dt*(size(td,1)-1);
%% process joint velocity and torque
fs = 200;
N = 2;
fc_vel = 1;
fc_tau = 2;
fc_acc = 0.5;
[Bvel,Avel] = butter(N,fc_vel/(fs/2));
[Btau,Atau] = butter(N,fc_tau/(fs/2));
[Bacc,Aacc] = butter(N,fc_acc/(fs/2));
joint_idx = 5;
vel_filter = filtfilt(Bvel, Avel, jvel(:,joint_idx));
tau_filter = filtfilt(Btau, Atau, jtor(:,joint_idx));
acc = diff(jvel(:,joint_idx))/dt;
acc_filter = filtfilt(Bacc, Aacc, acc);
acc_filter = [0;acc_filter];
figure
plot(t,jvel(:,joint_idx),'r'); grid on; hold on;
plot(t,vel_filter,'k'); plot(t,acc_filter,'b'); hold off;
title('joint velocity')
figure
plot(t,jtor(:,joint_idx),'r'); grid on; hold on;
plot(t,tau_filter,'k'); hold off;
title('joint torque')
%% joint friction parameters identification
% seg_idx = {[5.28,18.25],[30.52,42.99],[65.07,72.88],[81.11,93.2],[111.8,116.9],[129,136.4],...
%             [154.7,159.4],[168.6,174.5],[192.5,196.1],[205,209.6],[229,232.8],[241.9,246.4],...
%             [262.1,265.4],[274,278],[293.6,296.4],[305.8,309.3],[328.4,330.9],[340.6,344.2],...
%             [364,366.2],[376.2,379],[400.8,402.8],[412.6,415.3]};
seg_idx = {[8.615,13.08],[23.07,27.03],[45.07,47.15],[57.42,60.83],[78.23,79.88],[86.83,89.44],...
            [104.6,106.8],[114.2,116],[139.4,141],[151,152.1],[171.8,173.3],[182.2,183.7],...
            [201.4,203],[209.8,211.4],[229,230.4],[237.2,238.6],[256,257],[263.8,265.4],...
            [280.3,281.7],[288.8,290.5],[313.6,315],[321,322.6]};
for idx=1:length(seg_idx)
    seg_idx{idx} = round(seg_idx{idx}/dt);
end
tau_Iden = []; jvel_iden = []; reg_fric = [];
for idx=1:length(seg_idx)
    vel = vel_filter(seg_idx{idx}(1):seg_idx{idx}(2));
    vel = mean(vel);
    reg_mat = [sign(vel), vel];
    reg_fric = [reg_fric; reg_mat];
    jvel_iden = [jvel_iden;vel];
    tau_Iden = [tau_Iden;mean(tau_filter(seg_idx{idx}(1):seg_idx{idx}(2)))];
end
fric_params = pinv(reg_fric)*tau_Iden;
time_tmp = datevec(now);
time_stamp = time_tmp(4)*100+time_tmp(5);
file_name = ['gravity/joint5_friction_parameters',num2str(time_stamp),'.txt'];
dlmwrite(file_name,fric_params,'precision',12);
%% joint friction parameters polyfit
vel_poly = sort(jvel_iden);
tau_poly = sort(tau_Iden);
figure
plot(vel_poly,tau_poly,'o'); grid on; hold on;
vv = min(vel_poly):0.01:max(vel_poly);
plot(vv,fric_params(1)*sign(vv)+fric_params(2)*vv,'k');
xlabel('vel(rad/s)'); ylabel('tau(Nm)');
%% joint friction parameters validation
tau_val = [];
for idx=1:length(jvel(:,joint_idx))
    v = jvel(idx,joint_idx);
    tau_tmp = fric_params(1)*SignVel(v)+fric_params(2)*v;
    tau_val = [tau_val;tau_tmp];
end
figure
plot(t,jtor(:,joint_idx),'r'); grid on; hold on;
plot(t,tau_val,'k');

