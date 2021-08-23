clear
close all
clc

addpath('classes');
addpath('tools');
dt = 0.005;
%% analyse test data
td = csvread('./data/test_data_0818_1720.csv');
t = 0:dt:dt*(size(td,1)-1);

figure
plot(t, td(:,1), t, td(:,3));
figure
plot(t, td(:,2), t, td(:,4));
figure
plot(t, td(:,5), t, td(:,6), t,td(:,7), t,td(:,8), t,td(:,9), t,td(:,10));
grid on