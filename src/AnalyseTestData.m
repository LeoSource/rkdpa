clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
dt = 0.005;
%% analyse test data
nj = 6;
td = load('./data/test_data_0101_121609.csv');
jpos_idx = 1; jvel_idx = 2; jtor_idx = 3;
jpos = td(:,1:jpos_idx*nj);
jvel = td(:,nj+1:jvel_idx*nj);
jtor = td(:,2*nj+1:jtor_idx*nj);
t = 0:dt:dt*(size(td,1)-1);

analysis_mode = 'dynamic_ground';
switch analysis_mode
    case 'common'
%% plot joint position, velocity and torque
jpos_plot = [1,2,3,4,5,6];
jvel_plot = [1,2,3];
jtor_plot = [1,2,3];
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
%% collision detector
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

    case 'tau_compare'
%% analyse joint torque during clean task
rbtdef = CreateRobot();
rbtdyn = RobotDynamics(rbtdef);
rbtdyn.LoadParams('gravity/gravity_parameters.txt','gravity/friction_parameters.txt');
fs = 200;
fc = 10;
for idx=1:6
    tau_filter = Biquad('LOWPASS', fc/fs, 0.707, 0);
    for nidx=1:length(jtor(:,idx))
        tor(idx,nidx) = tau_filter.Filter(jtor(nidx,idx));
    end
end
for nidx=1:length(jtor(:,1))
    tau_iden(:,nidx) = rbtdyn.GenerateIdenTorque(jpos(nidx,:),jvel(nidx,:));
end
for idx=1:6
    figure
    plot(t,tau_iden(idx,:), t,tor(idx,:)); grid on;
    title(['joint',num2str(idx)]);
    xlabel('time(s)'); ylabel('joint torque(Nm)');
end

    case 'acceleration'
%% differentiator for acceleration
JointDiff = JointDifferentiator(1, 0.005);
% for nidx=1:size(jpos,1)
%     [jvel_diff(:,nidx),jacc_diff(:,nidx)] = JointDiff.ProcessPosition(jpos(nidx,:));
% end
for nidx=1:size(jpos,1)
    jacc_diff(:,nidx) = JointDiff.ProcessVelocity(jvel(nidx,:));
end
for jidx=1:6
    figure
    plot(t,jacc_diff(jidx,:)); grid on; title(['joint',num2str(jidx)]);
end
% for jidx=1:6
%     figure
%     plot(t,jvel(:,jidx)-jvel_diff(jidx,:)'); grid on; title(['joint',num2str(jidx)]);
% end
% for jidx=1:6
%     figure
%     plot(t,jvel(:,jidx),'r', t,jvel_diff(jidx,:),'k'); grid on;
%     title(['joint',num2str(jidx)]);
% end

    case 'dynamic_ground'
%% dynamic analysis during cleaning ground
rbtdef = CreateRobot();
rbtdef.tool = SE3(rotx(0), [0,0,0.51]);
rbtdyn = RobotDynamics(rbtdef);
rbtdyn.LoadParams('gravity/gravity_parameters.txt','gravity/friction_parameters.txt');
load('jointdata.mat');
force_ext = [0,0,-30,0,0,0]';
for idx=1:size(jpos,2)
    tau_iden(:,idx) = rbtdyn.GenerateIdenTorque(jpos(:,idx),jvel(:,idx));
    jaco = rbtdef.jacob0(jpos(:,idx));
    tau_ext(:,idx) = jaco'*force_ext;
    tau_test(:,idx) = tau_iden(:,idx)-tau_ext(:,idx);
    tau_test_up(:,idx) = tau_test(:,idx)+15*ones(6,1);
    tau_test_down(:,idx) = tau_test(:,idx)-15*ones(6,1);
end

pnts = 1:size(jpos,2);
for jidx=1:6
    if jidx<=3
        tau_rated = 52;
    else
        tau_rated = 10;
    end
    figure
    plot(tau_test(jidx,:),'k-'); grid on; hold on;
%     plot(pnts, ones(size(pnts))*tau_rated,'r');
    if jidx==2
        plot(pnts, -ones(size(pnts))*tau_rated,'r');
        plot(tau_test_up(jidx,:),'b--');
        plot(tau_test_down(jidx,:),'b--');
    end
    ylabel('tau(Nm)'); title(['joint',num2str(jidx)]);
end

end


%% robot description
function rbt = CreateRobot()
    d1 = 0.048; a2 = 0.51; a3 = 0.51;
    d4 = 0.11; d5 = 0.08662; d6 = 0.035;
    mdh_table = [0, d1, 0, 0, 0, 0;...
                        0, 0, 0, -pi/2, 0, -pi/2;...
                        0, 0, a2, 0, 0, pi/2;...
                        0, d4, a3, 0, 0, -pi/2;...
                        0, d5, 0, -pi/2, 0, 0;...
                        0, d6, 0, pi/2, 0, 0];
    % pose_tool = SE3(rotx(-10), [0,0,0.116]);
    tool_toiletlid = SE3(rotx(0), [0,-0.035,0.23]);
    qmin = [-pi, -pi/2, -4*pi/3, -pi, -pi, -2*pi]';
    qmax = [pi, pi/2, pi/3, pi, pi, 2*pi]';
    rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot', 'tool',tool_toiletlid);
    rbt.qlim(:,1) = qmin; rbt.qlim(:,2) = qmax;
end