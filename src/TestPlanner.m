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

test_case = 'joint';
switch test_case
    case 'lspb'
        %% test for lspb planner
%         lspbplanner = LspbPlanner(-10,3,4,3,[-2,3]);
        lspbplanner = LspbPlanner(0,0.5,0.5,[],[0.5,0]);
        lspbplanner.PlotAVP(g_cycle_time);
        

    case 'line'
        %% test for line planner
        lineplanner = LinePlanner([0,0,0]',[],0.5,0.5,[[0.1,0,0]',[0,0,0.1]'],...
                            [0,0,0]',[0,0,0]',0.5,0.5,[0,0]);
        cpos=[]; cvel=[]; cacc=[];
        for t=0:g_cycle_time:4
            [p,pv,pa,r,rv,ra] = lineplanner.GenerateMotion(t);
            cpos = [cpos,[p;r]]; cvel = [cvel,[pv;rv]]; cacc = [cacc,[pa;ra]];
        end
        figure
        plot2(cpos(1:3,:)', 'r--'); hold on;
        grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
        
        
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