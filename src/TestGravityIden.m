clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
addpath('gravity')
dt = 0.005;
%% robot gravity identification
grav_regressor = RobotGravityIden;

reg_grav = []; tau_grav = []; jpos = [];
jpos5 = []; tau5 = [];
for idx=1:16
    data_file = ['./gravity/joint5_data',num2str(idx),'.mat'];
    jdata = load(data_file);
    jpos5 = [jpos5; jdata.jpos_iden];
    tau5 = [tau5; jdata.tor_iden];
    clear jdata
end
tau_grav = [tau_grav; tau5];
jpos = [jpos; jpos5];
for idx=1:length(tau5)
    reg_tmp = grav_regressor.CalcRegressorJoint5(jpos5(idx,:));
    reg_grav = [reg_grav; reg_tmp];
end

jpos4 = []; tau4 = [];
for idx=1:9
    data_file = ['./gravity/joint4_data',num2str(idx),'.mat'];
    jdata = load(data_file);
    jpos4 = [jpos4; jdata.jpos_iden];
    tau4 = [tau4; jdata.tor_iden];
    clear jdata
end
tau_grav = [tau_grav; tau4];
jpos = [jpos; jpos4];
for idx=1:length(tau4)
    reg_tmp = grav_regressor.CalcRegressorJoint4(jpos4(idx,:));
    reg_grav = [reg_grav; reg_tmp];
end

jpos3 = []; tau3 = [];
for idx=1:4
    data_file = ['./gravity/joint3_data',num2str(idx),'.mat'];
    jdata = load(data_file);
    jpos3 = [jpos3; jdata.jpos_iden];
    tau3 = [tau3; jdata.tor_iden];
    clear jdata
end
tau_grav = [tau_grav; tau3];
jpos = [jpos; jpos3];
for idx=1:length(tau3)
    reg_tmp = grav_regressor.CalcRegressorJoint3(jpos3(idx,:));
    reg_grav = [reg_grav; reg_tmp];
end

jpos2 = []; tau2 = [];
for idx=1:6
    data_file = ['./gravity/joint2_data',num2str(idx),'.mat'];
    jdata = load(data_file);
    jpos2 = [jpos2; jdata.jpos_iden];
    tau2 = [tau2; jdata.tor_iden];
    clear jdata
end
tau_grav = [tau_grav; tau2];
jpos = [jpos; jpos2];
for idx=1:length(tau2)
    reg_tmp = grav_regressor.CalcRegressorJoint2(jpos2(idx,:));
    reg_grav = [reg_grav; reg_tmp];
end

grav_regressor.barycenter_params = pinv(reg_grav)*tau_grav;

tau_val = [];
for idx=1:length(tau3)
    tau_tmp = grav_regressor.CalcGravJoint3(jpos3(idx,:));
    tau_val = [tau_val; tau_tmp];
end
plot(tau3,'r'); grid on; hold on; plot(tau_val, 'k');
legend('identification', 'validation');


%% load joint test data
nj = 6;
td = load('./data./joint2/test_data_1106_112629.csv');
jpos_idx = 1; jvel_idx = 2; jtor_idx = 3;
jpos = td(:,1:jpos_idx*nj);
jvel = td(:,nj+1:jvel_idx*nj);
jtor = td(:,2*nj+1:jtor_idx*nj);
t = 0:dt:dt*(size(td,1)-1);

%% process joint position and torque
test_joint_idx = 2;
jpos_iden = [];
record_value = [-0.7, 0.7];
eps = 4e-4;
start_idx = find(jpos(:,test_joint_idx)>record_value(1)-0.5*eps & jpos(:,test_joint_idx)<record_value(1)+0.5*eps);
stop_idx = find(jpos(:,test_joint_idx)>record_value(2)-0.5*eps & jpos(:,test_joint_idx)<record_value(2)+0.5*eps);
if length(start_idx) ~=2  || length(stop_idx) ~=2
    error('rescop1');
end
start_idx(2) = start_idx(2)+1;
if abs(stop_idx(1)-start_idx(1)) ~= abs(stop_idx(2)-start_idx(2))
    error('rescope2');
end
sparse_value = 5;
jpos_iden = [jpos_iden; jpos(start_idx(1):stop_idx(1),:)];
jpos_iden = jpos_iden(1:sparse_value:end,:);
tor1 = jtor(start_idx(1):stop_idx(1),test_joint_idx);
tor2_tmp = jtor(stop_idx(2):start_idx(2),test_joint_idx);
tor2 = fliplr(tor2_tmp');
tor2 = tor2';
tor_grav = 0.5*(tor1+tor2);
tor_iden = tor_grav(1:sparse_value:end);

fs = 200;
N= 2;
fc = 5;
[Btorque, Atorque] = butter(N, fc/(fs/2));
tor = filtfilt(Btorque, Atorque, jtor(:,test_joint_idx));

figure
plot(t, jtor(:,test_joint_idx), t, tor); grid on;
figure
plot(tor1); grid on; hold on; plot(tor2); hold off;
save('./gravity/joint2_data6.mat', 'jpos_iden', 'tor_iden');

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



