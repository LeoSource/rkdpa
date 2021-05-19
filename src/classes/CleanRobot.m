%   build clean robot model
%   it is not a universal serial arm robot because of the iksolver
%   the iksolver is specific under work situation
%   workspace display which depends on dh_table and joint limit 
%   Author:
%   liao zhixiang, zhixiangleo@163.com

classdef CleanRobot < handle
    
    properties
        arm
        tool
    end
    properties
        mdh
        nlinks
        links
        tool_pose
        qmin
        qmax

        gain_pos
        gain_rpy
        gain_opt
        q_ik
        ts
    end
        
    methods
        %% Class Constructor
        function obj = CleanRobot()
            % mdh parameters: theta d a alpha type offset
            l1 = 0.111; l2 = 0.0935; l3 = 0.066;% model refinement
            h = 0.3; w = 0.518;
            mdh_table = [      0,   0,   0,       0,    0,   0
                                    pi/2,   0,   0,       0,    1,   h 
                                        0,    l2,   -l1,   pi/2,    0,   pi/2
                                    pi/2,    0,   0,   pi/2,   1,   w
                                        0,    0,   -l3,    pi/2,   0,   0];
            qlimit = [-pi/2, pi/2; 0, 1; -pi/2 ,pi/2; 0, 0.506; -2*pi, 2*pi];            
            obj.arm = SerialLink(mdh_table,'modified','name','CleanRobot');
            for idx=1:size(mdh_table,1)
                obj.arm.links(idx).qlim = qlimit(idx,:);
            end            
            obj.tool = [0, 0.2*cos(-pi/6), 0.2*sin(-pi/6)]';
            % without robotics toolbox
            obj.mdh = mdh_table;
            obj.nlinks = size(mdh_table, 1);
            for idx=1:obj.nlinks
                obj.links{idx} = MDHLink(mdh_table(idx,:));
            end
            obj.qmax = qlimit(:,2);
            obj.qmin = qlimit(:,1);
            obj.tool_pose = eye(4);
            obj.tool_pose(1:3, 1:3) = rotx(-30);
            obj.tool_pose(1:3,end) = [0, 0.2*cosd(-30), 0.2*sind(-30)]';
            obj.gain_pos = diag([500, 500, 500]);
            obj.gain_rpy = diag([500, 500, 500, 100, 100]);
            obj.gain_opt = diag([5, 5, 5, 5, 5]);
        end
        
        %% Forward Kinematics Using Robotics Toolbox
        function pose = FKSolve(obj, q)
            pose = obj.arm.fkine(q);
            pose.t = pose.t+tr2rt(pose)*obj.tool;
        end

        function  tool_pose = FKSolveTool(obj, q)
            pose = obj.transform(q, 1, obj.nlinks);
            tool_pose = pose*obj.tool_pose;
        end

        function pose = transform(obj, q, s_idx, e_idx)
            pose = eye(4);
            for idx=s_idx:e_idx
                obj.links{idx}.Transform(q(idx));
                pose = pose*obj.links{idx}.pose;
            end
        end

        function pose = fksolve(obj, q)
            pose = obj.transform(q, 1, obj.nlinks);
            pose = pose*obj.tool_pose;
        end
        
        %% Inverse Kinematics with Analytical Solution    
        function q = IKSolve(obj, pos, option, alpha, q_in)
            %%to simplify the problem of end-effector's pose: theta5 = alpha-theta1
            q = zeros(5,1);
            ty = obj.tool(2); tz = obj.tool(3);
            h = obj.links{2}.offset; w = obj.links{4}.offset;
            height_limit = obj.arm.qlim(2,:)+tz+obj.links{2}.offset;
            switch option
                case 'q3first0'
                    q = obj.IKJnt3(pos, alpha, 0, q_in);
                case 'q3firstn'
                    q = obj.IKJnt3(pos, alpha, -pi/6, q_in);
                case 'q2first'
                    if pos(3)>height_limit(2)
                        q(2) = obj.arm.qlim(2,2);
                    elseif pos(3)<height_limit(1)
                        q(2) = obj.arm.qlim(2,1);
                    else
                        q(2) = pos(3)-tz - h;
                    end        
                    a = pos(1)+sin(alpha)*ty;
                    b = pos(2)-cos(alpha)*ty;
                    c = obj.arm.d(3)+obj.arm.a(5);
                    q(1) = CalcTransEqua(a, b, c, q_in(1));
                    q(5) = alpha-q(1);

                    s1 = sin(q(1)); c1 = cos(q(1));
                    s5 = sin(q(5)); c5 = cos(q(5));
                    lx = obj.arm.d(3)+obj.arm.a(5); ly = obj.arm.a(3);
                    a = c1*(pos(3)-q(2)-h);
                    b = -pos(2)-s1*s5*ty+c1*ly+s1*lx;
                    c = c1*tz;
                    q(3) = CalcTransEqua(a, b, c, q_in(3));
                    s3 = sin(q(3)); c3 = cos(q(3));
                    tmp_value = pos(2)-ty*(-s1*s5+c1*c3*c5)+c1*s3*tz-ly*c1-lx*s1;
                    q(4) = tmp_value/(c1*c3)-w;                    
                otherwise
                    error('put the right option to inverse kinematics solver');
            end
        end
        
        function q = IKJnt3(obj, pos, alpha, q3, q_in)
            q = zeros(5,1);
            ty = obj.tool(2); tz = obj.tool(3);
            q(3) = q3;
            a = pos(1)+sin(alpha)*ty;
            b = pos(2)-cos(alpha)*ty;
            c = obj.arm.d(3)+obj.arm.a(5);
            q(1) = CalcTransEqua(a, b, c, q_in(1));
            q(5) = alpha-q(1);

            s1 = sin(q(1)); c1 = cos(q(1));
            s3 = sin(q(3)); c3 = cos(q(3));
            s5 = sin(q(5)); c5 = cos(q(5));
            h = obj.links{2}.offset; w = obj.links{4}.offset;
            lx = obj.arm.d(3)+obj.arm.a(5); ly = obj.arm.a(3);
            tmp_value = pos(2)-ty*(-s1*s5+c1*c3*c5)+c1*s3*tz-ly*c1-lx*s1;
            q(4) = tmp_value/(c1*c3) - w;
            q(2) = pos(3)-s3*c5*ty-c3*tz-s3*(q(4)+w) - h;
        end
        
        %% Inverse Kinematics with Numerical Solution(optimization toolbox)
        function [q, qerr, exitflag] = IKSolveCon(obj, pose, q0)
            %%to guarantee that the end-effector's pose is perpendicular to
            %%vertical surface, but it is very hard to get a analytical value
            %%to do: set tolerance, set iteration times
            A = [];
            b = [];
            Aeq = [];
            beq = [];
            lb = obj.arm.qlim(:,1);
            ub = obj.arm.qlim(:,2);
            nonlcon = @(x) obj.PosCond(x, pose.t);
            objfunc = @(x) obj.ObjFunc(x, tr2rt(pose));
            [q, qerr, exitflag] = fmincon(objfunc, q0, A, b, Aeq, beq, lb, ub, nonlcon);           
        end
        
        function objective = ObjFunc(obj, q, cmd_rot) 
            fdb_rot = tr2rt(obj.FKSolve(q));
            objective = norm(cmd_rot-fdb_rot);
        end
        
        function [c, ceq] = PosCond(obj, q, pos) 
            ty = obj.tool(2); tz = obj.tool(3);
            s1 = sin(q(1)); c1 = cos(q(1));
            s3 = sin(q(3)); c3 = cos(q(3));
            s5 = sin(q(5)); c5 = cos(q(5));
            lx = obj.arm.d(3)+obj.arm.a(5); ly = obj.arm.a(3);
            h = obj.links{2}.offset; w = obj.links{4}.offset;
            ceq(1) = -ty*(c1*s5+s1*c3*c5)+s1*s3*tz-s1*c3*(q(4)+w)-ly*s1+lx*c1-pos(1);
            ceq(2) = ty*(-s1*s5+c1*c3*c5)-c1*s3*tz+c1*c3*(q(4)+w)+ly*c1+lx*s1-pos(2);
            ceq(3) = s3*c5*ty+c3*tz+q(2)+h+s3*(q(4)+w)-pos(3);
            c = [];
        end

        %% Inverse Kinematics of Numerical Solution with Jacobian
        function [q, qd] = IKSolvePos(obj, pos_cmd, vel_cmd, q_in)
            pose = obj.FKSolveTool(q_in);
            pos_fdb = pose(1:3, end);
            vel_comp = obj.gain_pos*(pos_cmd-pos_fdb);
            jv = obj.CalcJv(q_in);
            qd = pinv(jv)*(vel_cmd+vel_comp);
            obj.q_ik = obj.q_ik+qd*obj.ts;
            obj.q_ik = LimitNumber(obj.qmin, obj.q_ik, obj.qmax);
            q = obj.q_ik;
        end

        function [q, qd] = IKSolveRPY(obj, pos_cmd, vel_cmd, q_in)
            pose = obj.FKSolveTool(q_in);
            pos_fdb = zeros(5,1);
            pos_fdb(1:3) = pose(1:3, end);
            rpy = Rot2RPY(pose(1:3, 1:3));
            pos_fdb(4:5) = [rpy(1); rpy(3)];
            vel_comp = obj.gain_rpy*(pos_cmd-pos_fdb);
            jaco = obj.CalcOperationJaco(q_in);
            qd = pinv(jaco)*(vel_cmd+vel_comp);
            obj.q_ik = obj.q_ik+qd*obj.ts;
            obj.q_ik = LimitNumber(obj.qmin, obj.q_ik, obj.qmax);
            q = obj.q_ik;
        end

        function [q, qd] = IKSolveRedudant(obj, pos_cmd, vel_cmd, q_in, dir_vec)
            pose = obj.FKSolveTool(q_in);
            pos_fdb = pose(1:3, end);
            vel_comp = obj.gain_pos*(pos_cmd-pos_fdb);

            jaco_wrist = obj.CalcJacoWrist(q_in);
            jv_wrist = jaco_wrist(1:3,:);
            jv_tool = obj.CalcJv(q_in);
            q0 = obj.gain_opt*(jv_tool'-jv_wrist')*dir_vec;

            qd = pinv(jv_tool)*(vel_cmd+vel_comp)+(eye(5)-pinv(jv_tool)*jv_tool)*q0;
            obj.q_ik = obj.q_ik+qd*obj.ts;
            obj.q_ik = LimitNumber(obj.qmin, obj.q_ik, obj.qmax);
            q = obj.q_ik;
        end

        function InitIKSolver(obj, q_in, ts)
            obj.q_ik = q_in;
            obj.ts = ts;
        end
        
        %% Calculate Jacobian Matrix
        function jaco = CalcJacoWrist(obj,q)
            jaco = obj.arm.jacob0(q);
        end

        function jaco = CalcJacoTool(obj, q)
            jaco_end = obj.CalcJacoWrist(q);
            p_mat = eye(6,6);
            pose = obj.FKSolve(q);
            rot = tr2rt(pose);
            r = rot*obj.tool;%projection to the world coordinate system
            p_mat(1:3,4:6) = -skew(r);
            jaco = p_mat*jaco_end;
        end

        function jv = CalcJv(obj, q)
            jaco = obj.CalcJacoTool(q);
            jv = jaco(1:3,:);
        end

        function jw = CalcJw(obj, q)
            jaco = obj.CalcJacoTool(q);
            jw = jaco(4:6,:);
        end

        function jaco = CalcRPYJaco(obj, q)
            pose = obj.FKSolveTool(q);
            rot_0_tool = pose(1:3, 1:3);
            rpy = Rot2RPY(rot_0_tool);
            trans_0_rpy = RPY2JAC(rpy);
            jv_tool = obj.CalcJv(q); jw_tool = obj.CalcJw(q);
            jw_tool = pinv(trans_0_rpy)*jw_tool;
            jaco = [jv_tool; jw_tool];
        end

        function jaco = CalcOperationJaco(obj, q)
            jaco_rpy_tool = obj.CalcRPYJaco(q);
            jaco = [jaco_rpy_tool(1:3,:); jaco_rpy_tool(4,:); jaco_rpy_tool(6,:)];
        end

        %% Validation for Workspace of Cleanrobot
        function PlotWorkspace(obj)
            %to do:simplify the plot code
            num = 10000;
            qlim = obj.arm.qlim;
            q0 = qlim(:,1)';
            qf = qlim(:,2)';
            tmp_q = rand(1, 5, num);
            q = q0+tmp_q.*(qf-q0);
            for idx=1:num
                pos(:,idx) = obj.FKSolve(q(:,:,idx)).t;
            end

            figure(1)
            plot3(pos(1,:), pos(2,:), pos(3,:), 'r.', 'MarkerSize', 3);
            grid on
            title('Cartesian Workspace of CleanRobot');
            xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
            work_range = [min(pos(1,:)), max(pos(1,:)); min(pos(2,:)), max(pos(2,:)); min(pos(3,:)), max(pos(3,:))];
            disp(['work range in X axis:',num2str(work_range(1,:))]);
            disp(['work range in Y axis:',num2str(work_range(2,:))]);    
            disp(['work range in Z axis:',num2str(work_range(3,:))]);    

            figure(2)
            k = convhull(pos(1,:), pos(2,:));
            f1 = plot(pos(1,k), pos(2,k), 'b-', 'LineWidth', 2);
            title('workspace in XOY plane');
            xlabel('X(m)'); ylabel('Y(m)');
            grid on
            hold on
            radius = 1.1; origin = [0, 0];
            alpha = linspace(-0.1,pi+0.1,1000);
            x = origin(1)+radius*cos(alpha); y = origin(2)+radius*sin(alpha);
            f2 = plot(x, y, 'c--', [x(1), 0, x(end)], [y(1), -0.2, y(end)], 'c--', 'LineWidth', 2);
            radius = radius*0.9;
            x = origin(1)+radius*cos(alpha); y = origin(2)+radius*sin(alpha);
            f3 = plot(x, y, 'k-', [x(1), 0, x(end)], [y(1), -0.2, y(end)], 'k-', 'LineWidth', 2);
            f4 = plot(pos(1,:), pos(2,:), 'r.');
            legend([f1, f2(1), f3(1)], 'envelope curve', 'reachable workspace', 'dexterous workspace');
            hold off

            figure(3)
            k = convhull(pos(1,:), pos(3,:));
            plot(pos(1,k), pos(3,k), 'b-', 'LineWidth', 2);
            title('workspace in XOZ plane');
            xlabel('X(m)'); ylabel('Z(m)');
            grid on
            hold on
            a = 1.6; b= 1.2; origin = [0, 1];
            alpha = linspace(0, 2*pi, 1000);
            x = origin(1)+b*cos(alpha); y = origin(2)+a*sin(alpha);
            plot(x, y, 'c--', 'LineWidth', 2);
            a = a*0.8; b = b*0.8; 
            x = origin(1)+b*cos(alpha); y = origin(2)+a*sin(alpha);
            plot(x, y, 'k-', 'LineWidth', 2);
            plot(pos(1,:), pos(3,:), 'r.');
            legend('envelope curve', 'reachable workspace', 'dexterous workspace');
            hold off

            figure(4)
            k = convhull(pos(2,:), pos(3,:));
            f1 = plot(pos(2,k), pos(3,k), 'b-', 'LineWidth', 2);
            title('workspace in YOZ plane');
            xlabel('Y(m)'); ylabel('Z(m)');
            grid on
            hold on
            a = 1.6; b = 1.2; origin = [0, 1];
            alpha = linspace(-0.1,pi+0.1,1000);
            x = origin(1)+b*cos(alpha-pi/2); y = origin(2)+a*sin(alpha-pi/2);
            f2 = plot(x, y, 'c--', [x(1), -0.2, x(end)], [y(1), origin(2), y(end)], 'c--', 'LineWidth', 2);
            a = a*0.8; b = b*0.8;
            x = origin(1)+b*cos(alpha-pi/2); y = origin(2)+a*sin(alpha-pi/2);
            f3 = plot(x, y, 'k-', [x(1), -0.2, x(end)], [y(1), origin(2), y(end)], 'k-', 'LineWidth', 2);
            f4 = plot(pos(2,:), pos(3,:), 'r.');
            legend([f1, f2(1), f3(1)], 'envelope curve', 'reachable workspace',  'dexterous workspace');
            hold off            
            
        end
    end
    
end

