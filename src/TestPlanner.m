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

test_case = 'cubic-spline';
switch test_case
    case 'lspb'
        %% test for lspb planner
%         lspbplanner = LspbPlanner(-10,3,4,3,[-2,3]);
        lspbplanner = LspbPlanner(0,0.5,0.5,[],[0.5,0]);
        lspbplanner.PlotAVP(g_cycle_time);
        compare_cpp = true;
        if compare_cpp
            [p,v,a] = lspbplanner.GenerateTraj(g_cycle_time);
            q_cpp = load('F:/0_project/rkdpl/mirrortask_jpos1.csv');
            q_cpp = q_cpp';
            t = 0:g_cycle_time:g_cycle_time*(length(p)-1);
            t_cpp = 0:g_cycle_time:g_cycle_time*(size(q_cpp,2)-1);
            figure
            subplot(3,1,1); plot(t, p, t_cpp,q_cpp(1,:)); grid on; ylabel('position');     
            subplot(3,1,2); plot(t, v, t_cpp,q_cpp(2,:)); grid on; ylabel('velocity');
            subplot(3,1,3); plot(t, a, t_cpp,q_cpp(3,:)); grid on; ylabel('acceleration');
        end

    case 'line'
        %% test for line planner
%         lineplanner = LinePlanner([0,0,0]',[],0.5,0.5,[[0.1,0,0]',[0,0,0.1]'],...
%                             [0,0,0]',[0,0,0]',0.5,0.5,[0,0]);
        lineplanner = LinePlanner([0,0,0]', [1,1,0]', 0.5,0.5,[0,0],...
                                [0,0,0]',[0,0,0]',0.5,0.5,[0,0]);
        cpos=[]; cvel=[]; cacc=[];
        for t=0:g_cycle_time:4
            [p,pv,pa,r,rv,ra] = lineplanner.GenerateMotion(t);
            cpos = [cpos,[p;r]]; cvel = [cvel,[pv;rv]]; cacc = [cacc,[pa;ra]];
        end
        figure
        plot2(cpos(1:3,:)', 'r--'); hold on;
        grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
        
        
    case 'cubic-spline'
        %% test for cubic spline planner
        q = [0, 0.5, 0.8, 1.1, 1.6, 1.9, 2.2, 2.5, 3];
        t = [0, 0.4, 0.8, 1.05, 1.4, 1.8, 2.4, 2.6, 3];
        dt = 0.005;
        planner1 = PolyTrajPlanner(q,t,[0,0],3);
        [p1,v1,a1] = planner1.GenerateTraj(dt);
        planner2 = CubicSplinePlanner(q,t,'clamped',[0,0]);
        [p2,v2,a2] = planner2.GenerateTraj(dt);
        planner3 = CubicSplinePlanner(q,t,'natural');
        [p3,v3,a3] = planner3.GenerateTraj(dt);
        planner4 = CubicSplinePlanner(q,t,'cyclic');
        [p4,v4,a4] = planner4.GenerateTraj(dt);
        
        tt = 0:dt:t(end);
        figure(1); subplot(3,1,1); scatter(t,q); hold on;
        h{1} = plot(tt,p1,'b',tt,p2,'r',tt,p3,'g',tt,p4,'k'); grid on; ylabel('pos');
        subplot(3,1,2); h{2} = plot(tt,v1,'b',tt,v2,'r',tt,v3,'g',tt,v4,'k'); grid on; ylabel('vel');
        subplot(3,1,3); h{3} = plot(tt,a1,'b',tt,a2,'r',tt,a3,'g',tt,a4,'k'); grid on; ylabel('acc');
        legend('poly','clamped','natural','cyclic');
        
        planner5 = CubicSplinePlanner(q,t,'clamped',[0,0]);
        planner5.SetTimeOptimizedStyle('gentle');
        [p5,v5,a5] = planner5.GenerateTraj(dt);
        planner6 = CubicSplinePlanner(q,t,'clamped',[0,0]);
        planner6.SetTimeOptimizedStyle('fast');
        [p6,v6,a6] = planner6.GenerateTraj(dt);
        planner7 = CubicSplinePlanner(q,t,'clamped',[0,0]);
        planner7.SetTimeOptimizedStyle('middle');
        [p7,v7,a7] = planner7.GenerateTraj(dt);
        planner8 = CubicSplinePlanner(q,t,'clamped',[0,0]);
        planner8.SetTimeOptimizedConstrtaints(1.5,10);
        [p8,v8,a8] = planner8.GenerateTraj(dt);
        t8 = 0:dt:(length(p8)-1)*dt;
        
        figure(2); subplot(3,1,1); scatter(t,q,'b'); hold on; scatter(planner5.duration,planner5.pos,'r');
        scatter(planner6.duration,planner6.pos,'g'); scatter(planner7.duration,planner7.pos,'k');
        scatter(planner8.duration,planner8.pos,'m');
        plot(tt,p2,'b',tt,p5,'r',tt,p6,'g',tt,p7,'k',t8,p8,'m'); grid on; ylabel('pos');
        subplot(3,1,2); h{2} = plot(tt,v2,'b',tt,v5,'r',tt,v6,'g',tt,v7,'k',t8,v8,'m'); grid on; ylabel('vel');
        subplot(3,1,3); h{3} = plot(tt,a2,'b',tt,a5,'r',tt,a6,'g',tt,a7,'k',t8,a8,'m'); grid on; ylabel('acc');
        legend('clamped','gentle','fast','middle','con-v-a');
        
        
    case 'joint'
        %% test for joint planner
        q0 = deg2rad([0,0,0,0,-90,0]');
        qf = deg2rad([-30,55,0,-60,-80,0]');
        jplanner = JointPlanner(q0,true,g_cycle_time);
        jplanner.AddJntPos(qf,g_jvmax,g_jamax);
        
        jpos = []; jvel = []; jacc = [];
        time_idx = 0;
        while ~jplanner.plan_completed
            [jp,jv,ja] = jplanner.GenerateMotion();
            jpos = [jpos,jp]; jvel = [jvel,jv]; jacc = [jacc,ja];
            if time_idx==230
                jplanner.Stop(jp,jv,g_jvmax,g_jamax);
            end
            time_idx = time_idx+1;
        end
        
        
    case 'cartesian'
        %% test for cartesian planner
        pos0 = [0,0,0]'; rpy0 = tr2rpy(rotx(0));
        cplanner = CartesianPlanner([pos0;rpy0'],false,g_cycle_time);
        pos1 = [2,3,2]'; rpy1 = tr2rpy(rotx(0));
        pos2 = [-1,1,0]';
        via_posrpy = [[pos1;rpy1'],[pos2;rpy1']];
        cplanner.AddPosRPY(via_posrpy,[0.5,0.5],[0.5,0.5]);
        
        cpos=[]; cvel=[]; cacc=[];
        time_idx = 0;
        while ~cplanner.plan_completed
            [p,pv,pa,r,rv,ra] = cplanner.GenerateMotion();
            cpos = [cpos,[p;r]]; cvel = [cvel,[pv;rv]]; cacc = [cacc,[pa;ra]];
            if time_idx==1000
                cplanner.Stop([p;r],[pv;rv],[0.5,0.5],[0.5,0.5]);
            end
            time_idx = time_idx+1;
        end
        
        figure
        plot2(cpos(1:3,:)', 'r--'); hold on;
        plot2(via_posrpy(1:3,:)', 'bo'); axis equal;
        grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
        
        
        
    case 'taskplanner-joint'
        %% test for task planner with only joint trajectory
        rbt = CreateRobot();
        q0 = deg2rad([0,0,0,0,-90,0]');
        compare_plan = false;
        taskplanner = TaskTrajPlanner([],q0,g_cycle_time,g_jvmax,g_jamax,...
                                                    g_cvmax,g_camax,compare_plan);
        qf = deg2rad([-30,55,0,-60,-80,0]');
        taskplanner.AddTraj(qf,'joint',true);
         jpos = []; jvel = []; jacc = [];
        time_idx = 0;
        while ~taskplanner.task_completed
            [jp,jv,ja] = taskplanner.GenerateJointMotion();
            jpos = [jpos,jp]; jvel = [jvel,jv]; jacc = [jacc,ja];
            if time_idx==230
                taskplanner.Stop(jp,jv);
            end
            time_idx = time_idx+1;
        end       
                                                
        
    case 'taskplanner'
        %% test for task planner
        rbt = CreateRobot();
        q0 = deg2rad([0,0,0,0,-90,0]');
        % q0 = deg2rad([-40,65,40,-35,-90,0]');
        compare_plan = false;
        taskplanner = TaskTrajPlanner([],rbt.fkine(q0),g_cycle_time,g_jvmax,g_jamax,...
                                                    g_cvmax,g_camax,compare_plan);
%         taskplanner = TaskTrajPlanner(rbt,q0,g_cycle_time,g_jvmax,g_jamax,...
%                                                     g_cvmax,g_camax,compare_plan);
%         taskplanner.AddTraj(zeros(6,1),'joint',true);
        % test for cartesian planner
        pos_rpy1 = [0.8,0,0.23,-pi,-pi/6,0]';
        pos_rpy2 = [0.3,0.2,0.23,-pi,-pi/6,0]';
        pos_rpy3 = [0.8,0,0.1,-pi,-pi/6,0]';
        pos_rpy4 = [0.2,0.6,0.23,-pi,-pi/6,0]';
        via_posrpy = [pos_rpy1,pos_rpy2,pos_rpy3,pos_rpy4];
        taskplanner.AddTraj(via_posrpy,'cartesian',true);

        % test for joint and arc planner
%         a = [0.89, 0.178, -0.3627]'; b = [0.87426, -0.19926, -0.36788]'; c = [0.5006, -0.1645, -0.3838]';
%         vision_pos = [a,b,c];
%         via_posrpy = PlanToiletlidPath(vision_pos, 110*pi/180, -110*pi/180, 90*pi/180, 0.05);
%         tmp_jpos = [-40,65,40,-35,-90,0]'*pi/180;
%         taskplanner.AddTraj(tmp_jpos, 'joint', true);
%         taskplanner.AddTraj(via_posrpy(:,1), 'cartesian', false);
%         taskplanner.AddTraj(via_posrpy(:,2:end), 'arc', false);

        cpos=[]; cvel=[]; cacc=[];
        jpos=[]; jvel=[]; jacc=[];
        time_idx = 0;
        while ~taskplanner.task_completed
%             [cp,cv,ca,jp,jv,ja] = taskplanner.GenerateBothMotion();
            [cp,cv,ca] = taskplanner.GenerateCartMotion();
            cpos = [cpos,cp]; cvel = [cvel,cv]; cacc = [cacc,ca];
%             jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
            if time_idx==1200
                taskplanner.Stop(cp,cv);
            end
            time_idx = time_idx+1;
        end

        figure
        plot2(cpos(1:3,:)', 'r--'); hold on;%plot2(cpos_sim', 'k');
        plot2(via_posrpy(1:3,:)', 'bo'); axis equal;%axis square vis3d;
        %     PlotRPY(cpos, 60); hold off;
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
    % pose_tool = SE3(rotx(-10), [0,0,0.116]);
%     tool_toiletlid = SE3(rotx(0), [0,-0.035,0.23]);
    tool_toiletlid = SE3(rotx(0), [0,0,0]);
    qmin = [-pi, -pi/2, -4*pi/3, -pi, -pi, -2*pi]';
    qmax = [pi, pi/2, pi/3, pi, pi, 2*pi]';
    rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot', 'tool',tool_toiletlid);
    rbt.qlim(:,1) = qmin; rbt.qlim(:,2) = qmax;
end