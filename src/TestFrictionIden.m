clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
dt = 0.005;
%% load joint test data
nj = 6;
td = load('./data/joint1_friction_1108_155706.csv');
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
joint_idx = 1;
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
seg_idx = {[1638,14690],[18390,30790],[35130,40780],[43870,49140],[52760,54780],[56850,59540],...
            [61760,62900],[64070,65320],[67190,67950],[69140,69560],[71340,71800],[72680,73180],...
            [75020,75320],[76270,76530],[78250,78440],[79440,79560],[81310,81460],[82320,82470]};
% seg_idx = {[1011,8076],[9510,16610],[18820,21930],[23250,26410],[28200,29450],[30600,31880],...
%             [33760,34480],[35850,36550],[38370,38760],[39830,40180],[42000,42210],[43310,43590],...
%             [45430,45740],[46650,47030],[48740,49020],[49830,50130],[52160,52390],[53170,53410],...
%             [54980,55150],[55880,56080],[57880,58010],[58800,58930]};
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

