clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
dt = 0.005;
simu_mode = 'friction';
switch simu_mode
    case 'gravity'
        GravityIden(dt);
    case 'friction'
        FrictionIden(dt);
end

%% robot gravity identification
function GravityIden(dt)
    [jpos,jvel,jtor,t] = LoadTestFile('./data/test_data_1222_171054.csv',dt);
    % PlotJointData(jpos,jvel,jtor,[1,2,3,4,5,6],[2,3,4,5],[2,3,4,5],t);
    rbtdef = CreateRobot();
    grav_iden = RobotDynamics(rbtdef);
    grav_iden.GravityIden(jpos,jtor);
    %%save gravity identification parameters%%
    time_tmp = datevec(now);
    time_stamp = [num2str(time_tmp(2)),num2str(time_tmp(3)),num2str(time_tmp(4)),num2str(time_tmp(5))];
    file_name = ['gravity/gravity_parameters_',time_stamp,'.txt'];
    dlmwrite(file_name,grav_iden.barycenter_params,'precision',12);
end

%% robot friction identification
function FrictionIden(dt)
    [~,jvel1,jtor1,~] = LoadTestFile('./data/test_data_1222_174238.csv',dt);
    [~,jvel2,jtor2,~] = LoadTestFile('./data/test_data_1222_175312.csv',dt);
    rbtdef = CreateRobot();
    rbtdyn = RobotDynamics(rbtdef);
    rbtdyn.FrictionIden(jvel1(:,1),jtor1(:,1),jvel2(:,5),jtor2(:,5));
    %%save friction identification parameters%%
    time_tmp = datevec(now);
    time_stamp = [num2str(time_tmp(2)),num2str(time_tmp(3)),num2str(time_tmp(4)),num2str(time_tmp(5))];
    file_name = ['gravity/friction_parameters',time_stamp,'.txt'];
    dlmwrite(file_name,rbtdyn.fric_params,'precision',12);
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

%% plot joint data
function PlotJointData(jpos,jvel,jtau,jpplot,jvplot,jtplot,t)
    figure;
    for idx=jpplot
        plot(t,jpos(:,idx),'DisplayName',['jpos',num2str(idx)]); grid on;
        xlabel('time(s)'); ylabel('position(rad)'); hold on;
    end
    hold off; legend;
    figure;
    for idx=jvplot
        plot(t,jvel(:,idx),'DisplayName',['jvel',num2str(idx)]); grid on;
        xlabel('time(s)'); ylabel('velocity(rad/s)'); hold on;
    end
    hold off; legend;
    figure;
    for idx=jtplot
        plot(t,jtau(:,idx),'DisplayName',['jtor',num2str(idx)]); grid on;
        xlabel('time(s)'); ylabel('torque(Nm)'); hold on;
    end
    hold off; legend;
end