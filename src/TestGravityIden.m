clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
addpath('gravity')
dt = 0.005;
%% load joint test data
nj = 6;
td = load('./data/test_data_1106_105529.csv');
jpos_idx = 1; jvel_idx = 2; jtor_idx = 3;
jpos = td(:,1:jpos_idx*nj);
jvel = td(:,nj+1:jvel_idx*nj);
jtor = td(:,2*nj+1:jtor_idx*nj);
t = 0:dt:dt*(size(td,1)-1);
%% plot joint data
plot_jdata = 0;
if plot_jdata
    jpos_plot = [1,2,3,4,5,6];
    jvel_plot = [5];
    jtor_plot = [5];
    figure;
    for idx=jpos_plot
        plot(t,jpos(:,idx),'DisplayName',['jpos',num2str(idx)]); grid on;
        xlabel('time(s)'); ylabel('position(rad)'); hold on;
    end
    hold off; legend;
    figure;
    for idx=jvel_plot
        plot(t,jvel(:,idx),'DisplayName',['jvel',num2str(idx)]); grid on;
        xlabel('time(s)'); ylabel('velocity(rad/s)'); hold on;
    end
    hold off; legend;
    figure;
    for idx=jtor_plot
        plot(t,jtor(:,idx),'DisplayName',['jtor',num2str(idx)]); grid on;
        xlabel('time(s)'); ylabel('torque(Nm)'); hold on;
    end
    hold off; legend;
end
%% robot gravity identification
grav_iden = RobotGravityIden;
grav_iden.Identificatino(jpos,jtor);
