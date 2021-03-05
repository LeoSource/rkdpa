classdef CleanRobot
    
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
        
        %% inverse kinematics with numerical solution
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
        
    end
    
end