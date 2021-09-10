clear
close all
clc

addpath('classes');
addpath('tools');
dt = 0.005;
%% analyse test data
nj = 6;
td = load('./data/test_data_0908_135411.csv');
jpos_idx = 1; jvel_idx = 2; jtor_idx = 3;
jpos = td(:,1:jpos_idx*nj); jvel = td(:,nj+1:jvel_idx*nj); jtor = td(:,2*nj+1:jtor_idx*nj);
t = 0:dt:dt*(size(td,1)-1);

% td1 = load('./data/test_data_0902_1000.csv');
% t1 = 0:dt:dt*(size(td1,1)-1);
% plot(t,td(:,10), 'k--', t1,td1(:,10), 'r-.'); grid on;
% legend('filtered', 'original');

fs = 200;
N= 5;
fc = 10;
[Btorque, Atorque] = butter(N, fc/(fs/2));
tor = filtfilt(Btorque, Atorque, jtor(:,4));
tor1 = filter(Btorque, Atorque, jtor(:,4));

% figure
% plot(t, td(:,5), t, td(:,6), t,td(:,7), t,td(:,8), t,td(:,9), t,td(:,10));
% grid on
figure
plot(t, jtor(:,4), t, tor); grid on
figure
plot(t,tor, t, tor1); grid on;