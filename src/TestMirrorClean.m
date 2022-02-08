clear all
close all
clc

addpath('classes');
addpath(genpath('tools'));

g_jvmax = [pi/8, pi/8, 0.8*pi/4, pi/4, pi/4, pi/4];
g_jamax = [pi/4, pi/4, pi/2, pi/4, pi/4, pi/4];
g_cvmax = [0.15, 0.3]; g_camax = [0.3, 0.5];
g_cycle_time = 0.005;

rbt = CreateRobot();
dt = 0.01;
q0 = [0, -35, 50, -100, -90, 0]'*pi/180;
compare_plan = 0;
taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
                                            g_cvmax,g_camax,compare_plan);
mirror_planner = MirrorCleanPlanner;
test_name = 'mirror_circle';
switch test_name
    case 'mirror_rectangle'
        p1 = [0.8,-0.2,0.45]'; p2 = [0.8,0.4,0.45]'; p3 = [0.8,0.4,0.81]'; p4 = [0.8,-0.2,0.81]';
        vision_pos = [p1,p2,p3,p4];
        mirror_planner.SetCleanParams(vision_pos,'eRectangle');
        [~,via_posrpy,~] = mirror_planner.PlanCleanPath();
        taskplanner.AddTraj(via_posrpy, 'cartesian', 0);
        
        if compare_plan
            [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
        else
            [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
        end
        figure
        plot2([vision_pos,p1]', 'r--'); hold on; axis equal;
        plot2(cpos(1:3,:)', 'k--'); plot2(via_posrpy(1:3,:)', 'bo'); plot2(vision_pos', 'r*');
        PlotRPY(cpos, 60);
        grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
    case 'mirror_circle'
        r = 0.4; origin = [0.8,0,0.8]'; rot_circle = [0,0,-1;-1,0,0;0,1,0];
        p1 = [origin(1),origin(2)-r,origin(3)]'; p2 = [origin(1), origin(2), origin(3)-r]';
        p3 = [origin(1),origin(2)+r,origin(3)]'; p4 = [origin(1),origin(2),origin(3)+r]';
        vision_pos = [p1,p2,p3,p4];
        mirror_planner.SetCleanParams(vision_pos,'eCircle');
        [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = mirror_planner.PlanCleanPath();
        % plan for up zone
        taskplanner.AddTraj(via_posrpy_up(:,1),'cartesian',0);
        taskplanner.AddTraj(via_posrpy_up(:,2:4),'arc',0);
        taskplanner.AddTraj(via_posrpy_up(:,5),'cartesian',0);
        % plan for middle zone
        taskplanner.AddTraj(via_posrpy_middle(:,1), 'cartesian', 0);
        taskplanner.AddTraj(via_posrpy_middle(:,2:4),'arc',0);
        taskplanner.AddTraj(via_posrpy_middle(:,5:end-4),'cartesian',0);
        taskplanner.AddTraj(via_posrpy_middle(:,end-3:end-1),'arc',0);
        taskplanner.AddTraj(via_posrpy_middle(:,end),'cartesian',0);
        % plan for down zone
        taskplanner.AddTraj(via_posrpy_down(:,1),'cartesian',0);
        taskplanner.AddTraj(via_posrpy_down(:,2:4),'arc',0);
        taskplanner.AddTraj(via_posrpy_down(:,5),'cartesian',0);
        if compare_plan
            [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
        else
            [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
        end
        theta = 0:deg2rad(2):deg2rad(360);
        circle_mirror(1,:) = r*cos(theta);
        circle_mirror(2,:) = r*sin(theta);
        circle_mirror(3,:) = zeros(size(circle_mirror(2,:)));
        for idx=1:size(circle_mirror,2)
            circle_mirror(:,idx) = origin+rot_circle*circle_mirror(:,idx);
        end
        figure
        plot2(circle_mirror', 'r--'); hold on; axis equal;
        plot2([via_posrpy_up(1:3,:),via_posrpy_middle(1:3,:),via_posrpy_down(1:3,:)]', 'bo');
        plot2(vision_pos', 'r*'); plot2(cpos(1:3,:)', 'k--');
        PlotRPY(cpos,60);
        grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
    case 'mirror_ellipse'
        a = 0.5; b = 0.3; origin = [0.8,0,0.8]'; rot_ellipse = [0,0,-1;-1,0,0;0,1,0];
        p1 = [origin(1),origin(2)-b,origin(3)]'; p2 = [origin(1),origin(2),origin(3)-a]';
        p3 = [origin(1),origin(2)+b,origin(3)]'; p4 = [origin(1),origin(2),origin(3)+a]';
        vision_pos = [p1,p2,p3,p4];
        mirror_planner.SetCleanParams(vision_pos,'eEllipse');
        [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = mirror_planner.PlanCleanPath();
        % plan for up zone
        taskplanner.AddTraj(via_posrpy_up(:,1), 'cartesian', 0);
        taskplanner.AddTraj(via_posrpy_up(:,2:4), 'arc', 0);
        taskplanner.AddTraj(via_posrpy_up(:,5), 'cartesian', 0);
        % plan for middle zone
        taskplanner.AddTraj(via_posrpy_middle, 'cartesian', 0);
        % plan for down zone
        taskplanner.AddTraj(via_posrpy_down(:,1), 'cartesian', 0);
        taskplanner.AddTraj(via_posrpy_down(:,2:4), 'arc', 0);
        taskplanner.AddTraj(via_posrpy_down(:,5), 'cartesian', 0);
        if compare_plan
            [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
        else
            [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
        end
        theta = 0:deg2rad(2):deg2rad(360);
        ellipse_mirror(1,:) = b*cos(theta);
        ellipse_mirror(2,:) = a*sin(theta);
        ellipse_mirror(3,:) = zeros(size(ellipse_mirror(2,:)));
        for idx=1:size(ellipse_mirror,2)
            ellipse_mirror(:,idx) = origin+rot_ellipse*ellipse_mirror(:,idx);
        end
        figure
        plot2(ellipse_mirror','r--'); hold on; axis equal;
        plot2([via_posrpy_up(1:3,:),via_posrpy_middle(1:3,:),via_posrpy_down(1:3,:)]', 'bo');
        plot2(vision_pos', 'r*'); plot2(cpos(1:3,:)', 'k--');
        PlotRPY(cpos, 60);
        grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
    case 'mirror_runway'
        p1 = [0.8,-0.2,0.45]'; p2 = [0.8,0.4,0.45]'; p3 = [0.8,0.4,0.81]'; p4 = [0.8,-0.2,0.81]';
        vision_pos = [p1,p2,p3,p4];
        mirror_planner.SetCleanParams(vision_pos,'eRunway');
        [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = mirror_planner.PlanCleanPath();
         % plan for up zone
        taskplanner.AddTraj(via_posrpy_up(:,1), 'cartesian', 0);
        taskplanner.AddTraj(via_posrpy_up(:,2:4), 'arc', 0);
        taskplanner.AddTraj(via_posrpy_up(:,5), 'cartesian', 0);
        % plan for middle zone
        taskplanner.AddTraj(via_posrpy_middle, 'cartesian', 0);
        % plan for down zone
        taskplanner.AddTraj(via_posrpy_down(:,1), 'cartesian', 0);
        taskplanner.AddTraj(via_posrpy_down(:,2:4), 'arc', 0);
        taskplanner.AddTraj(via_posrpy_down(:,5), 'cartesian', 0);
        if compare_plan
            [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
        else
            [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
        end
        rot_runway = [0,0,-1;0,1,0;1,0,0];
        origin_up = 0.5*(p3+p4);
        radius_up = 0.5*norm(p3-p4);
        th_up = deg2rad(-90):deg2rad(2):deg2rad(90);
        arc_up(1,:) = radius_up*cos(th_up);
        arc_up(2,:) = radius_up*sin(th_up);
        arc_up(3,:) = zeros(size(arc_up(2,:)));
        for idx=1:size(arc_up,2)
            arc_up(:,idx) = origin_up+rot_runway*arc_up(:,idx);
        end
        origin_down = 0.5*(p1+p2);
        radius_down = 0.5*norm(p1-p2);
        th_down = deg2rad(90):deg2rad(2):deg2rad(270);
        arc_down(1,:) = radius_down*cos(th_down);
        arc_down(2,:) = radius_down*sin(th_down);
        arc_down(3,:) = zeros(size(arc_down(2,:)));
        for idx=1:size(arc_down,2)
            arc_down(:,idx) = origin_down+rot_runway*arc_down(:,idx);
        end
        figure
        plot2([p1,p4]','r--'); hold on; axis equal; plot2([p2,p3]','r--');
        plot2(arc_up','r--'); plot2(arc_down','r--');
        plot2([via_posrpy_up(1:3,:),via_posrpy_middle(1:3,:),via_posrpy_down(1:3,:)]', 'bo');
        plot2(vision_pos', 'r*');
        plot2(cpos(1:3,:)', 'k--');
        PlotRPY(cpos, 60);
        grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
    case 'mirror_octagon'
        p1 = [0.8,-0.2,0.45]'; p2 = [0.8,0.1,0.3]'; p3 = [0.8,0.4,0.45]'; p4 = [0.8,0.4,0.81]'; p5 = [0.8,-0.2,0.81]';
        vision_pos = [p1,p2,p3,p4,p5];
        mirror_planner.SetCleanParams(vision_pos,'eOctagon');
        [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = mirror_planner.PlanCleanPath();
        % plan for middle zone
%         taskplanner.AddTraj(via_posrpy_middle, 'cartesian', 0);
        % plan for down zone
        taskplanner.AddTraj(via_posrpy_down, 'cartesian', 0);
        if compare_plan
            [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = taskplanner.GenerateBothTraj(dt);
        else
            [cpos,cvel,cacc] = taskplanner.GenerateCartTraj(dt);
        end
        figure
        plot2([vision_pos,p1]', 'r--'); hold on; axis equal;
        plot2(cpos(1:3,:)', 'k--'); plot2(vision_pos', 'r*');
        plot2([via_posrpy_middle(1:3,:),via_posrpy_down(1:3,:)]', 'bo');
        PlotRPY(cpos, 60);
        grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
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
%     pose_tool = SE3(rotx(-10), [0,0,0.116]);
%     tool_toiletlid = SE3(rotx(0), [0,-0.035,0.23]);
    tool_mirror = SE3(rotx(-30), [0,0.058,0.398]);
    qmin = [-pi, -pi/2, -4*pi/3, -pi, -pi, -2*pi]';
    qmax = [pi, pi/2, pi/3, pi, pi, 2*pi]';
    rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot', 'tool',tool_mirror);
    rbt.qlim(:,1) = qmin; rbt.qlim(:,2) = qmax;
end

