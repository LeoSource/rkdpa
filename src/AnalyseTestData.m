clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
dt = 0.005;
%% analyse test data
nj = 6;
td = load('./data/test_data_1215_174740.csv');
jpos_idx = 1; jvel_idx = 2; jtor_idx = 3;
jpos = td(:,1:jpos_idx*nj);
jvel = td(:,nj+1:jvel_idx*nj);
jtor = td(:,2*nj+1:jtor_idx*nj);
t = 0:dt:dt*(size(td,1)-1);

analysis_mode = 'filter';
switch analysis_mode
    case 'common'
%% plot joint position, velocity and torque
jpos_plot = [1,2,3,4,5,6];
jvel_plot = [1,2,3,4,5,6];
jtor_plot = [1,2,3,4,5,6];
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

    case 'torque'
%% analyse joint torque
for idx=1:6
    figure
    plot(t,jvel(:,idx), t,jtor(:,idx)); grid on;
    xlabel('time(s)'); ylabel('torque(Nm)');
    title(['joint',num2str(idx)]);
end

    case 'filter'
%% filter compared
fs = 200;
N= 2;
fc = 10;
for idx=1:6
    [Btorque, Atorque] = butter(N, fc/(fs/2));
    tor = filtfilt(Btorque, Atorque, jtor(:,idx));
%     tor1 = filter(Btorque, Atorque, jtor(:,idx));
    tau_filter = Biquad('LOWPASS',fc/fs,0.707,0);
    for nidx=1:length(jtor(:,idx))
        tor1(nidx) = tau_filter.Filter(jtor(nidx,idx));
    end

    figure
    plot(t, jtor(:,idx),'r', t, tor,'b', t,tor1,'k'); grid on;
%     plot(t, tor,'b', t,tor1,'k'); grid on;
    xlabel('time(s)'); ylabel('torque(Nm)');
    title(['joint',num2str(idx)]);
end

    case 'collision'
fs = 200;
fc = 10;
for idx=1:6
    tau_filter = Biquad('LOWPASS',fc/fs,0.707,0);
    for nidx=1:length(jtor(:,idx))
        tor(nidx) = tau_filter.Filter(jtor(nidx,idx));
    end
    figure
    title(['joint',num2str(idx)]);
    xlabel('time(s)');
    yyaxis left
    plot(t,jtor(:,idx),'r', t,tor,'b-', t,jvel(:,idx),'k-'); grid on;
    ylabel('joint torque(Nm)');
    yyaxis right
    plot(t,tor'-jvel(:,idx));
    ylabel('joint torque bias(Nm)');
end

end

