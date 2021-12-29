clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
dt = 0.005;
%% load joint test data
nj = 6;
td = load('./data/test_data_1229_145039.csv');
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
% seg_idx = {[8.4,21.8],[28.96,42.56],[77.23,88.09],[99.29,114.7],[133.5,141.4],[148.9,157.6],...
%             [177.8,184.4],[192.1,197.5],[218.5,221.8],[236.4,240.3],[264,267.4],[277.6,279.4],...
%             [299.1,302.4],[312,314],[334.6,336.4],[346.8,349.3],[372.1,374.9],[384.6,386.2],...
%             [406,408.2],[418.2,421],[441.8,442.8],[455.6,457.3]};
seg_idx = {[1.5,3.2],[6.3,14.9],[19.9,24.6],[28.9,35.3],[40,43],[47.8,52],...
            [56.2,59.8],[63.8,68.5],[72.9,76.9],[81.2,86.2],[91,95],[99.2,103.5],...
            [108.8,112.8],[117.7,121.7],[126.5,130.5],[134.9,138.7],[143.7,147.3],...
            [151.7,155.2],[160.1,163],[167.8,171],[176,178.6],[183.9,186]};
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
time_stamp = [num2str(time_tmp(2)),num2str(time_tmp(3)),num2str(time_tmp(4)),num2str(time_tmp(5))];
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

