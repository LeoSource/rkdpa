clear
close all
clc

addpath('classes');
addpath(genpath('tools'));

global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
g_jvmax = [pi/8, pi/8, 0.8*pi/4, pi/4, pi/4, pi/4];
g_jamax = [pi/4, pi/4, pi/2, pi/4, pi/4, pi/4];
g_cvmax = [0.4, 0.6]; g_camax = [0.8, 1.2];
g_cycle_time = 0.005;

func_map = containers.Map;
simu_name = {'workspace','common','toilet_lid','toilet','table','friction','gravity','dynamics'};
simu_func = {@PlotWorkspace,...
                    @PlanCommon,...
                    @PlanToiletlid,...
                    @PlanToilet,...
                    @PlanTable,...
                    @GenerateFricIdenTraj,...
                    @GenerateGravIdenTraj,...
                    @GenerateDynIdenTraj};
for idx=1:length(simu_name)
    func_map(simu_name{idx}) = simu_func{idx};
end

rbt = CreateRobot();
dt = 0.01;
test_name = 'common';
if strcmp(test_name, 'workspace')
    PlotWorkspace(rbt)
else
    func = func_map(test_name);
    [jpos,joint_plot,compare_cpp] = func(rbt,dt);
    
    if joint_plot
        PlotJointPosition(jpos,compare_cpp,dt)
    end
end
t = 0:dt:dt*(size(jpos,2)-1);

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
%     tool_toiletlid = SE3(rotx(0), [0,-0.035,0.23]);
    tool_toiletlid = SE3(rotx(0), [0,0,0]);
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
        global g_cycle_time
        tt = g_cycle_time*[0:size(q_cpp,2)-1];
        for idx=1:6
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
    compare_cpp = 0;
    compare_plan = 0;

%     q0 =  deg2rad([0, -35, 50, -100, -90, 0]');
%     taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
%                                                 g_cvmax,g_camax,compare_plan);
%     
%     qtable = deg2rad([-5,-2,-12,9,-93,-3]');
%     q00 = zeros(6,1);
%     q1 = deg2rad([90,-30,20,-90,-110,-90]');
%     q2 = deg2rad([45,30,20,-20,-90,90]');
%     q3 = deg2rad([0,-50,-30,0,90,0]');
%     q4 = deg2rad([-90,-30,20,-90,-110,-90]');
%     q5 = deg2rad([-45,30,20,-20,-90,-90]');
%     q6 = deg2rad([0,-50,-30,0,90,0]');
%     qfatigue = [q00,q1,q2,q3,q4,q5,q6,q00];
%     qstandby = deg2rad([0,-35,20,65,-90,0]');
%     
%     taskplanner.AddTraj([qtable,qfatigue,qstandby], 'joint', 1);
    q0 = deg2rad([0,0,0,0,-90,0]');
    taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                                g_cvmax,g_camax,compare_plan);
    pos_rpy1 = [0.8,0,0.23,-pi,-pi/6,0]';
    pos_rpy2 = [0.3,0.2,0.23,-pi,-pi/6,0]';
    pos_rpy3 = [0.8,0,0.1,-pi,-pi/6,0]';
    pos_rpy4 = [0.2,0.6,0.23,-pi,-pi/6,0]';
    via_posrpy = [pos_rpy1,pos_rpy2,pos_rpy3,pos_rpy4];
    taskplanner.AddTraj(via_posrpy,'cartesian',true);

    [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
    % [cpos,cve,cacc] = taskplanner.GenerateCartTraj(dt);
%     [jpos,~,~] = taskplanner.GenerateJointTraj(dt);
    output_pos = jpos;
    
    figure
    plot2(cpos(1:3,:)', 'r--'); hold on;%plot2(cpos_sim', 'k');
    plot2(via_posrpy(1:3,:)', 'bo'); axis equal;%axis square vis3d;
%     PlotRPY(cpos, 60); hold off;
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

%% simulate cleaning toilet
function [output_pos,joint_plot,compare_cpp] = PlanToilet(rbt,dt)
        global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
        joint_plot = 1;
        compare_cpp = 0;
        compare_plan = 1;
        q0 = deg2rad([20,30,40,-70,-90,0]');
        taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                                    g_cvmax,g_camax,compare_plan);
        p1 = [ 0.523063,0.0671301,-0.451756]'; p2 = [0.532622,0.189069,-0.449019]'; p3 = [0.628787,0.256844,-0.445255]';
        p4 = [0.770462,0.208325,-0.438582]'; p5 = [  0.845802,0.142217,-0.433995]'; p6 = [0.800494,0.0301943,-0.449299]';
        p7 = [0.722686,-0.061485,-0.441482]'; p8 = [0.627474,-0.0225126,-0.447813]';
        p9 = [0.714925,0.0960708,-0.585581]'; p10 = [ 0.714925,0.0960708,-0.585581]'; p11 = [0.714925,0.0960708,-0.585581]';
        p12 = [ 0.714925,0.0960708,-0.585581]'; p13 = [0.714925,0.0960708,-0.585581]'; p14 = [0.714925,0.0960708,-0.585581]';
        p15 = [0.750471,0.115512,-0.559841]'; p16 = [0.854383,0.0735621,-0.556036]';
        p19 = [0.548193,0.454853,-0.66386]';
        vision_pos = [p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16];
%         figure
%         plot2(vision_pos','r*'); axis equal;
%         grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
        
        via_posrpy = PlanToiletInlierPath(vision_pos(:,1:8),deg2rad(20),'left');
        taskplanner.AddTraj(via_posrpy(:,1),'cartesian',0);
        slant_angle = deg2rad(10);
        taskplanner.SetPlanningScene(vision_pos(:,1:8),slant_angle);
        taskplanner.AddTraj([p1,p2,p3,p4],'bspline',1);
        if compare_plan
            [cpos,~,~,jpos,~,~,cpos_sim] = taskplanner.GenerateBothTraj(dt);
            output_pos = jpos;
        else
            [cpos,~,~] = taskplanner.GenerateCartTraj(dt);
        end
        
        taskplanner.Reset(jpos(:,end));
        taskplanner.AddTraj([p4,p5,p6,p7],'bspline',1);
        if compare_plan
            [cpos_tmp,~,~,jpos_tmp,~,~,cpos_sim_tmp] = taskplanner.GenerateBothTraj(dt);
            output_pos = [output_pos,jpos_tmp];
            cpos = [cpos,cpos_tmp]; jpos = [jpos,jpos_tmp]; cpos_sim = [cpos_sim,cpos_sim_tmp];
        else
            [cpos,~,~] = taskplanner.GenerateCartTraj(dt);
            cpos = [cpos,cpos_tmp];
        end
        
%         taskplanner.Reset(jpos(:,end));
%         taskplanner.AddTraj([p7,p8,p9,p10],'bspline',1);
%         if compare_plan
%             [cpos_tmp,~,~,jpos_tmp,~,~,cpos_sim_tmp] = taskplanner.GenerateBothTraj(dt);
%             output_pos = [output_pos,jpos_tmp];
%             cpos = [cpos,cpos_tmp]; jpos = [jpos,jpos_tmp]; cpos_sim = [cpos_sim,cpos_sim_tmp];
%         else
%             [cpos,~,~] = taskplanner.GenerateCartTraj(dt);
%             cpos = [cpos,cpos_tmp];
%         end
%         
%         taskplanner.Reset(jpos(:,end));
%         taskplanner.AddTraj([p10,p11,p12,p13],'bspline',1);
%         if compare_plan
%             [cpos_tmp,~,~,jpos_tmp,~,~,cpos_sim_tmp] = taskplanner.GenerateBothTraj(dt);
%             output_pos = [output_pos,jpos_tmp];
%             cpos = [cpos,cpos_tmp]; jpos = [jpos,jpos_tmp]; cpos_sim = [cpos_sim,cpos_sim_tmp];
%         else
%             [cpos,~,~] = taskplanner.GenerateCartTraj(dt);
%             cpos = [cpos,cpos_tmp];
%         end
%         
%         taskplanner.Reset(jpos(:,end));
%         taskplanner.AddTraj([p13,p14,p15,p16],'bspline',1);
%         if compare_plan
%             [cpos_tmp,~,~,jpos_tmp,~,~,cpos_sim_tmp] = taskplanner.GenerateBothTraj(dt);
%             output_pos = [output_pos,jpos_tmp];
%             cpos = [cpos,cpos_tmp]; jpos = [jpos,jpos_tmp]; cpos_sim = [cpos_sim,cpos_sim_tmp];
%         else
%             [cpos,~,~] = taskplanner.GenerateCartTraj(dt);
%             cpos = [cpos,cpos_tmp];
%         end

        figure
        plot2(cpos(1:3,:)', 'r--'); hold on;
        plot2(via_posrpy(1:3,:)', 'bo'); plot2(vision_pos', 'r*'); axis equal;
        PlotRPY(cpos,50);
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
    t = 0:dt:dt*(length(jpos(joint_idx,:))-1);
    plot(t,rad2deg(jpos(joint_idx,:))); grid on; hold on;
    plot(t,rad2deg(jvel(joint_idx,:)));
    plot(t,rad2deg(jacc(joint_idx,:)));
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
    
    [jpos1,jvel1,jtor1,t1] = LoadTestFile('./data/test_data_1229_145039.csv',0.005);
    JointDiff1 = JointDifferentiator(1, 0.005);
    JointDiff2 = JointDifferentiator(2, 0.005);
    JointDiff3 = JointDifferentiator(3, 0.005);
    for nidx=1:size(jvel1,1)
        jacc_diff1(:,nidx) = JointDiff1.ProcessVelocity(jvel1(nidx,:));
        jacc_diff2(:,nidx) = JointDiff2.ProcessVelocity(jvel1(nidx,:));
        jacc_diff3(:,nidx) = JointDiff3.ProcessVelocity(jvel1(nidx,:));
    end
    figure
    plot(t,rad2deg(jacc(joint_idx,:)),'r', t1,rad2deg(jacc_diff1(joint_idx,:)),'g',...
            t1,rad2deg(jacc_diff2(joint_idx,:)),'b', t1,rad2deg(jacc_diff3(joint_idx,:)),'m');
    grid on; legend('acc\_cmd','acc\_order1','acc\_order2','acc\_order3');
    % subplot(2,1,1)
    % plot(t1,rad2deg(jvel1(:,joint_idx)),'r', t,rad2deg(jvel(joint_idx,:)),'k'); grid on;
    % subplot(2,1,2)
    % plot(t1,rad2deg(ja),'r', t,rad2deg(jacc(joint_idx,:)),'k'); grid on;
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

%% simulate robot dynamics identification trajectory
function [output_pos,joint_plot,compare_cpp] = GenerateDynIdenTraj(rbt,dt)
    global g_jvmax g_jamax g_cvmax g_camax g_cycle_time
    joint_plot = 1;
    compare_cpp = 1;
    compare_plan = 0;
    q0 = zeros(6,1);
    taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                    g_cvmax,g_camax,compare_plan);
    traj_params_half1hz = load('./data/traj_0.5hz.csv');
    traj_params_1hz = load('./data/traj_1hz.csv');
    dyniden_planner = DynIdenTrajPlanner(traj_params_half1hz,traj_params_1hz);
    dyniden_planner.AddTraj(10,8);
    taskplanner.AddDynIdenTraj(dyniden_planner);
    
    [jpos,~,~] = taskplanner.GenerateJointTraj(dt);
    output_pos = jpos;

end
