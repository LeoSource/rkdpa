classdef CleanRobot < handle
    
    % build clean robot model
    % surface trajectory plan
    % some tests 
    
    properties(SetAccess = private)
        arm
        tool
    end
        
    methods
        %% constructor
        function obj = CleanRobot()
            % mdh parameters: theta d a alpha sigma offset
            mdh_table = [      0,   0,   0,       0,    0,   0
                                    pi/2,   0,   0,       0,    1,   0 
                                        0,    0,   0,   pi/2,    0,   pi/2
                                    pi/2,    0,   0,   pi/2,   1,   0
                                        0,    0,   0,    pi/2,   0,   0];
            qlimit = [-pi/2, pi/2; 0.5, 1.5; -pi/2 ,pi/2; 0.2, 1; -2*pi, 2*pi];            
            obj.arm = SerialLink(mdh_table,'modified','name','CleanRobot');
            for idx=1:size(mdh_table,1)
                obj.arm.links(idx).qlim = qlimit(idx,:);
            end            
            obj.tool = [0, 0.2*cos(-pi/6), 0.2*sin(-pi/6)]';
        end
        
        %% forward kinematics using robotics toolbox
        function pose = FKSolve(obj, q)
            pose = obj.arm.fkine(q);
            pose.t = pose.t+tr2rt(pose)*obj.tool;
        end
        
        %% inverse kinematics with analytical solution        
        function q = IKSolve(obj, pos, option, alpha)
             %%to simplify the problem of end-effector's pose: theta5 = alpha-theta1
            q = zeros(1,5);
            ty = obj.tool(2); tz = obj.tool(3);
            height_limit = obj.arm.qlim(2,:)+tz;
            switch option
                case 'q3first'
                    q(3) = -pi/6;
                    q(1) = atan((pos(1)+sin(alpha)*ty)/(cos(alpha)*ty-pos(2)));
                    q(5) = alpha-q(1); 
                    tmp_value = pos(2)-ty*(-sin(q(1))*sin(q(5))+cos(q(1))*cos(q(3))*cos(q(5)))+cos(q(1))*sin(q(3))*tz;
                    q(4) = tmp_value/(cos(q(1))*cos(q(3)));
                    q(2) = pos(3)-cos(q(5))*sin(q(3))*ty-cos(q(3))*tz-sin(q(3))*q(4);                               
                case 'q2first'
                    if pos(3)>height_limit(2)
                        q(2) = obj.arm.qlim(2,2);
                    elseif pos(3)<height_limit(1)
                        q(2) = obj.arm.qlim(2,1);
                    else
                        q(2) = pos(3)-tz;
                    end        
                    q(1) = atan((pos(1)+sin(alpha)*ty)/(cos(alpha)*ty-pos(2)));
                    q(5) = alpha-q(1);
                    a = sin(q(1))*(pos(3)-q(2));
                    b = pos(1)+sin(q(5))*cos(q(1))*ty;
                    c = sin(q(1))*tz;
                    if abs(a+c)<1e-5
                        u = (c-a)/(2*b);
                        q(3) = 2*atan(u);
                    else
                        q3_tmp1 = 2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));
                        q3_tmp2 = 2*atan((b-sqrt(b^2+a^2-c^2))/(a+c));
                        if (q3_tmp1>pi/2) || (q3_tmp1<-pi/2)
                            q(3) = q3_tmp2;
                        else
                            q(3) = q3_tmp1;
                        end
                    end
                    tmp_value = pos(2)-ty*(-sin(q(1))*sin(q(5))+cos(q(1))*cos(q(3))*cos(q(5)))+cos(q(1))*sin(q(3))*tz;
                    q(4) = tmp_value/(cos(q(1))*cos(q(3)));                    
                otherwise
                    error('put the right option to inverse kinematics solver');
            end
        end
        
        %% inverse kinematics with numerical solution(optimization toolbox)
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
            ceq(1) = -ty*(c1*s5+s1*c3*c5)+s1*s3*tz-s1*c3*q(4)-pos(1);
            ceq(2) = ty*(-s1*s5+c1*c3*c5)-c1*s3*tz+c1*c3*q(4)-pos(2);
            ceq(3) = s3*c5*ty+c3*tz+q(2)+s3*q(4)-pos(3);
            c = [];
        end
        
        %% jacobian matrix calculation
        function jaco = CalcJaco(obj, q)
            jaco_end = obj.arm.jacob0(q);
            p_mat = eye(6,6);
            pose = obj.FKSolve(q);
            rot = tr2rt(pose);
            r = rot*obj.tool;%projection to the world coordinate system
            p_mat(1:3,4:6) = -skew(r);
            jaco = p_mat*jaco_end;
        end

        function jv = CalcJv(obj, q)
            jaco = obj.CalcJaco(q);
            jv = jaco(1:3,:);
        end

        function jw = CalcJw(obj, q)
            jaco = obj.CalcJaco(q);
            jw = jaco(4:6,:);
        end

        %% validation for the workspace of cleanrobot
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