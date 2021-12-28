clear all
close all
clc

addpath('classes');
addpath(genpath('tools'));

global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
g_jvmax = [pi/8, pi/8, 0.8*pi/4, pi/4, pi/4, pi/4];
g_jamax = [pi/4, pi/4, pi/2, pi/4, pi/4, pi/4];
g_cvmax = [0.15, 0.15]; g_camax = [0.3, 0.3];
g_cycle_time = 0.005;

func_map = containers.Map;
simu_name = {'workspace','common','toilet_lid','mirror','table','friction','gravity'};
simu_func = {@PlotWorkspace,...
                    @PlanCommon,...
                    @PlanToiletlid,...
                    @PlanMirror,...
                    @PlanTable,...
                    @GenerateFricIdenTraj,...
                    @GenerateGravIdenTraj};
for idx=1:length(simu_name)
    func_map(simu_name{idx}) = simu_func{idx};
end

rbt = CreateRobot();
dt = 0.005;
test_name = 'friction';
if strcmp(test_name, 'workspace')
    PlotWorkspace(rbt)
else
    func = func_map(test_name);
    [jpos,joint_plot,compare_cpp] = func(rbt,dt);
    
    if joint_plot
        PlotJointPosition(jpos,compare_cpp,dt)
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

%% plot workspace
function PlotWorkspace(rbt)
    nump = 5000;
    tmp_q = rand(1,6,nump);
    qmin = rbt.qlim(:,1);
    qmax = rbt.qlim(:,2);
    q = qmin'+tmp_q.*(qmax'-qmin');
    for idx=1:nump
        pos(:,idx) = rbt.fkine(q(:,:,idx)).t;
    end

    figure
    plot3(pos(1,:), pos(2,:), pos(3,:), 'r.', 'MarkerSize', 3);
    grid on
    xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
end

%% plot joint position
function PlotJointPosition(jpos,compare_cpp,dt)
    t = 0:dt:dt*(size(jpos,2)-1);
    figure
    plot(t,jpos(1,:),'-', t, jpos(2,:), '--', t, jpos(3,:), '-.', t, jpos(4,:), ':', t, jpos(5,:), '-', t,jpos(6,:),'k');
    grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');

    if compare_cpp
        q_cpp = load('./data/mirrortask_jpos1.csv');
        q_cpp = reshape(q_cpp, 6, []);
        tt = g_cycle_time*[0:size(q_cpp,2)-1];
        for idx=1:rbt.n
            figure
            plot(t, jpos(idx,:), 'b--', tt, q_cpp(idx,:), 'r-');
            xlabel('time'); ylabel(['q', num2str(idx)]); grid on;
            legend('matlab\_data', 'cpp\_data');
        end
    end
end

%% simulation all task
function [output_pos,joint_plot,compare_cpp] = PlanCommon(rbt, dt)
    global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
    joint_plot = 1;
    compare_cpp = 1;
    compare_plan = 1;

    q0 =  [-38,-27,18,-7,-82,-82]'*pi/180;
    taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                                g_cvmax,g_camax,compare_plan);
    vision_pos = [0.649857,  0.612921, 0.662765, 0.745594, 0.839413, 0.876876, 0.811265, 0.732626, 0.7157;
                        -0.106503, -0.00835975, 0.0869856, 0.112016, 0.0893813, -0.0304303, -0.126356, -0.142894, -0.09246;
                        -0.533577, -0.542969, -0.53932, -0.526466, -0.506219, -0.491052, -0.50203, -0.512606, -0.6222];
    % via_posrpy = CalcViapos(vision_pos(:,2:end), 'toilet');
    via_posrpy = PlanToiletInlierPath(vision_pos,10*pi/180,'left');
    % pos1 = [0.5297,0.2516,-0.4929]'; pos2 = [0.5208,0.2905,-0.5424]'; pos3 = [0.6039,0.4115,-0.544]';
    % pos4 = [0.7013,0.3362,-0.5544]'; pos5 = [0.6396,0.2582,-0.567]';
    % rpy1 = [-106,0.3,-175]'*pi/180; rpy2 = [-104.7,-0.8,-170]'*pi/180; rpy3 = [-114.8,3.7,-168]'*pi/180;
    % rpy4 = [-113,8.6,-177.7]'*pi/180; rpy5 = [-109.2,4.4,-179.4]'*pi/180;
    % via_posrpy = [[pos1;rpy1], [pos2;rpy2], [pos3;rpy3], [pos4;rpy4], [pos5;rpy5]];
    %%%%%simple test%%%%%
    % posrpy1 = [0,0,0,0,0,0]';
    % pos2 = [0.5,0.5,0.5]'; rpy2 = tr2rpy(rotx(90), 'xyz')'; posrpy2 = [pos2;rpy2];
    % pos3 = [1,1,1]'; rpy3 = tr2rpy(roty(90), 'xyz')'; posrpy3 = [pos3;rpy3];
    % via_posrpy = [posrpy1, posrpy2, posrpy3];
    % taskplanner.AddTraj(via_posrpy, 'bspline', 'interpolation');
    taskplanner.AddTraj(via_posrpy, 'cartesian', 0);

    [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
    % [cpos,cve,cacc] = taskplanner.GenerateCartTraj(dt);
    output_pos = jpos;
    
    figure
    plot2(cpos(1:3,:)', 'r--'); hold on;%plot2(cpos_sim', 'k');
    plot2(via_posrpy(1:3,:)', 'bo'); axis equal;%axis square vis3d;
    PlotRPY(cpos, 60); hold off;
    grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
end

%% simulate toilet lifting
function [output_pos,joint_plot,compare_cpp] = PlanToiletlid(rbt, dt)
    global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
    joint_plot = 1;
    compare_cpp = 0;
    compare_plan = 1;

    q0 = [0,-35,20,65,-90,0]'*pi/180;
    taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                                g_cvmax,g_camax,compare_plan);
    toilet_lid = 'open';
    if strcmp(toilet_lid,'open')
        a = [0.89, 0.178, -0.3627]'; b = [0.87426, -0.19926, -0.36788]'; c = [0.5006, -0.1645, -0.3838]';
        vision_pos = [a,b,c];
        via_posrpy = PlanToiletlidPath(vision_pos, 110*pi/180, -110*pi/180, 90*pi/180, 0.05);
        tmp_jpos = [-40,65,40,-35,-90,0]'*pi/180;
    elseif strcmp(toilet_lid,'close')
        a = [0.87, 0.178, -0.3627]'; b = [0.85426, -0.19926, -0.36788]'; c = [0.9232, -0.079565, 0.066379]';
        vision_pos = [a,b,c];
        % for toilet lid
    %     via_posrpy = PlanToiletlidPath(vision_pos, -pi/3, 0*pi/180, 150*pi/180, 0.05);
    %     tmp_jpos = [0,-35,20,0,-70,150]'*pi/180;
        % for toilet seat
        via_posrpy = PlanToiletlidPath(vision_pos, -pi/3, -100*pi/180, 20*pi/180, 0.05);
        tmp_jpos = [-40,65,40,-35,-90,0]'*pi/180;
    end
    taskplanner.AddTraj(tmp_jpos, 'joint', 1);
    taskplanner.AddTraj(via_posrpy(:,1), 'cartesian', 0);
    taskplanner.AddTraj(via_posrpy(:,2:end), 'arc', 0);
    % taskplanner.AddTraj(fliplr(via_posrpy(:,2:end)), 'arc', 0);

    if compare_plan
        [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
        output_pos = jpos;
    else
        [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
    end

    figure
    plot2(cpos(1:3,:)', 'r--'); hold on;
    plot2(via_posrpy(1:3,:)', 'bo'); plot2(vision_pos','r*'); axis equal;
    PlotRPY(cpos, 50);
    grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
end

%% simulate scrape mirror 
function [output_pos,joint_plot,compare_cpp] = PlanMirror(rbt, dt)
    global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
    joint_plot = 1;
    compare_cpp = 0;
    compare_plan = 1;
    rbt.tool = SE3(rotx(-30), [0,0.058,0.398]);
    q0 = [0, -35, 50, -100, -90, 0]'*pi/180;
    taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                                g_cvmax,g_camax,compare_plan);
    rectplanner = QuadranglePlanner;
    p1 = [0.8,-0.2,0.45]'; p2 = [0.8,0.4,0.45]'; p3 = [0.8,0.4,0.81]'; p4 = [0.8,-0.2,0.81]';
    vision_pos = [p1,p2,p3,p4];
    % via_posrpy = CalcMirrorPath(vision_pos, 0.15, 10*pi/180, 50*pi/180);
    % via_posrpy = CalcMirrorPath_Normal(vision_pos, 0.15, 60*pi/180, 0.08, 'right');
    via_posrpy = rectplanner.PlanMirror(vision_pos, 1);
    taskplanner.AddTraj(via_posrpy, 'cartesian', 0);

    if compare_plan
        [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
        output_pos = jpos;
    else
        [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
    end

    figure
    plot2(cpos(1:3,:)', 'r--');hold on;
    plot2(via_posrpy(1:3,:)', 'bo'); plot2(vision_pos', 'r*'); axis equal;
    PlotRPY(cpos, 60);
    grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
end

%% simulate rectangle table or ground zones
function [output_pos,joint_plot,compare_cpp] = PlanTable(rbt, dt)
    global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
    joint_plot = 1;
    compare_cpp = 0;
    compare_plan = 1;
    % q0 = [0,-35,40,-35,-90,0]'*pi/180;
    q0 = [-10,20,50,-35,-90,0]'*pi/180;
    rbt.tool = SE3(rotx(0), [0,0,0.51]);
    taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                                g_cvmax,g_camax,compare_plan);
    rectplanner = QuadranglePlanner;
    % table zone
    % p1 = [0.379,0.018,-0.164]'; p2 = [0.407,0.5057,-0.158]';
    % p3 = [0.852,0.4808,-0.1438]'; p4 = [0.8264,-0.0066,-0.148375]';
    % ground zone A1
    p1 = [0.50024,0.01394,-0.80907]'; p2 = [0.55821,0.37747,-0.79605]';
    p3 = [1.04993,0.35174,-0.78043]'; p4 = [1.04299,0.02213,-0.796]';
    % ground zone A2
    % p1 = [0.42578,-0.28383,-0.81447]'; p2 = [0.54249,0.46099,-0.79074]';
    % p3 = [0.75257,0.46584,-0.78597]'; p4 = [0.72554,-0.27564,-0.79367]';
    % ground zone A3
    % p1 = [0.35768,-0.57864,-0.81025]'; p2 = [0.43221,-0.25252,-0.80335]';
    % p3 = [0.83252,-0.28358,-0.79444]'; p4 = [0.80586,-0.61339,-0.79536]';
    % ground zone A4
    % p1 = [0.42027,-0.2954,-0.80776]'; p2 = [0.51165,0.42612,-0.79377]';
    % p3 = [0.94606,0.4236,-0.77803]'; p4 = [0.67835,-0.30654,-0.79918]';
    vision_pos = [p1,p2,p3,p4];
    via_posrpy = rectplanner.PlanGround(vision_pos);
    % via_posrpy = rectplanner.PlanTable(vision_pos);
    taskplanner.AddTraj(via_posrpy, 'cartesian', 0);

    if compare_plan
        [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
        output_pos = jpos;
    else
        [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
    end

    figure
    plot2(cpos(1:3,:)', 'r--');hold on;
    plot2(via_posrpy(1:3,:)', 'bo'); plot2(vision_pos', 'r*'); axis equal;
    PlotRPY(cpos, 100);
    grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
end

%% simulate friction identification trajectory
function [output_pos,joint_plot,compare_cpp] = GenerateFricIdenTraj(rbt, dt)
    global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
    joint_plot = 0;
    compare_cpp = 0;
    compare_plan = 0;
    q0 = [0,45,45,0,-10,0]'*pi/180';
    taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                    g_cvmax,g_camax,compare_plan);
    joint_idx = 5;
    if joint_idx==1
        proj = [1,0,0,0,0,0]';
        q00 = deg2rad([0,-35,20,65,-90,0]');
    elseif joint_idx==5
        proj = [0,0,0,0,1,0]';
        q00 = deg2rad([0,0,0,-90,0,0]');
    end
    det_q = deg2rad([10,20,30,50,70,90,110,120,130,140,150]);
    vel_scale = [0.05,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0];
    taskplanner.Reset(q00);
    for idx=1:length(vel_scale)
        detq_pos = proj*det_q(idx);
        if idx==11
            detq_neg = -proj*det_q(idx);
        else
            detq_neg = -proj*det_q(idx+1);
        end
        q1 = q00+detq_pos;
        q2 = q00+detq_neg;
        taskplanner.AddTraj([q1,q2],'joint',1,vel_scale(idx),1);
    end
    [jpos,jvel,jacc] = taskplanner.GenerateJointTraj(dt);
    plot(rad2deg(jpos(joint_idx,:))); grid on; hold on;
    plot(rad2deg(jvel(joint_idx,:)));
    plot(rad2deg(jacc(joint_idx,:)));
    hold off;
    output_pos = jpos;
    
    zero_acc_idx = find(~jacc(joint_idx,:));
    seg_idx = [zero_acc_idx(1)];
    for nidx=1:length(zero_acc_idx)-1
        if zero_acc_idx(nidx+1)-zero_acc_idx(nidx)~=1
            seg_idx = [seg_idx,zero_acc_idx(nidx),zero_acc_idx(nidx+1)];
        end
    end
    seg_idx = [seg_idx,zero_acc_idx(end)];
    start_idx = seg_idx(1:2:end);
    stop_idx = seg_idx(2:2:end);
    count_scale = 0.7;
    for idx=1:length(start_idx)
        len = stop_idx(idx)-start_idx(idx);
        start_idx(idx) = start_idx(idx)+round(len*0.5*(1-count_scale));
        stop_idx(idx) = stop_idx(idx)-round(len*0.5*(1-count_scale));
    end
    disp('starting index for identifying joint friction is'); disp(start_idx');
    disp('ending index for identifying joint friction is'); disp(stop_idx');
end

%% simulate gravity identification trajectory
function [output_pos,joint_plot,compare_cpp] = GenerateGravIdenTraj(rbt, dt)
    global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
    joint_plot = 0;
    compare_cpp = 0;
    compare_plan = 0;
    q0 = deg2rad([0,-60,-160,-90,-90,45]');
    qf = deg2rad([0,60,20,90,90,-45]');
    taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                    g_cvmax,g_camax,compare_plan);
    taskplanner.AddTraj([qf,q0],'joint',1,0.05,1);
    [jpos,jvel,~] = taskplanner.GenerateJointTraj(dt);
    t = 0:dt:dt*(size(jpos,2)-1);
    zero_vel_idx = find(~jvel(2,:));
    drop_time = 5;
    start_idx(1) = drop_time/dt;
    stop_idx(1) = zero_vel_idx(2)-(start_idx(1)-1);
    start_idx(2) = zero_vel_idx(2)+(start_idx(1)-1);
    stop_idx(2) = start_idx(2)+(stop_idx(1)-start_idx(1));
    time_slice{1} = dt*[start_idx(1),stop_idx(1)];
    time_slice{2} = dt*[start_idx(2),stop_idx(2)];
    disp('starting and ending time for processing gravity joint data is');
    disp(time_slice{1});
    disp(time_slice{2});
    figure
    plot(t,jpos(1,:),'-', t, jpos(2,:), '--', t, jpos(3,:), '-.', t, jpos(4,:), ':', t, jpos(5,:), '-', t,jpos(6,:),'k');
    grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
    figure
    plot(t,jvel(1,:),'-', t, jvel(2,:), '--', t, jvel(3,:), '-.', t, jvel(4,:), ':', t, jvel(5,:), '-', t,jvel(6,:),'k');
    grid on; title('joint velocity'); legend('qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6');
    output_pos = jpos;
end


