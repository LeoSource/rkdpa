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
        function q = IKSolveCircle(obj, pos, option, circle_params)
            h = circle_params.origin(2);
            r = circle_params.radius;
            alpha = circle_params.alpha;
            ty = obj.tool(2); tz = obj.tool(3);
            switch option
                case 'horizontal'                    
                    q(1) = atan((r-ty)*sin(alpha)/(h+cos(alpha)*(r-ty)));
                    q(2) = pos(3) - tz;
                    q(3) = 0;
                    q(4) = sqrt((sin(alpha)*(r-ty))^2+(h+(r-ty)*cos(alpha))^2);
                    q(5) = alpha - q(1);
                case 'slope'
                    q(1) = atan(sin(alpha)*(r-ty)/(h+cos(alpha)*(r-ty)));
                    q(3) = -pi/6;
                    q(5) = alpha-q(1);
                    tmp_value = h+r*cos(alpha)+ty*sin(q(1))*sin(q(5));
                    q(4) = tmp_value/(cos(q(1))*cos(q(3)))-ty*cos(q(5));
                    q(2) = pos(3)-sin(q(3))*(q(4)+ty*cos(q(5)));
                otherwise
                    error('IKSolver can not slove the circle trajectory');
            end
        end
        
        function q = IKSolve(obj, pos, option, circle_params)
            %%to simplify the problem of end-effector's pose: theta5 = -theta1
            q = zeros(1,5);
            ty = obj.tool(2); tz = obj.tool(3);            
            height_limit = obj.arm.qlim(2,:)+tz;
            switch option
                case 'vertical'
                    if pos(3)>height_limit(2)
                        q(2) = obj.arm.qlim(2,2);
                    elseif pos(3)<height_limit(1)
                        q(2) = obj.arm.qlim(2,1);
                    else
                        q(2) = pos(3)-tz;
                    end        
                    q(1) = atan(pos(1)/(ty-pos(2)));
                    a = sin(q(1))*(pos(3)-q(2));
                    b = pos(1)-sin(q(1))*cos(q(1))*ty;
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
                    tmp_value = pos(2)-ty*((sin(q(1)))^2+(cos(q(1)))^2*cos(q(3)))+cos(q(1))*sin(q(3))*tz;
                    q(4) = tmp_value/(cos(q(1))*cos(q(3)));
                    q(5) = -q(1);
                case 'horizontal'
                    q(3) = -pi/6;
                    q(1) = atan(pos(1)/(ty-pos(2)));
                    tmp_value = pos(2)-ty*((sin(q(1)))^2+(cos(q(1)))^2*cos(q(3)))+cos(q(1))*sin(q(3))*tz;
                    q(4) = tmp_value/(cos(q(1))*cos(q(3)));
                    q(2) = pos(3)-cos(q(1))*sin(q(3))*ty-cos(q(3))*tz-sin(q(3))*q(4);
                    q(5) = -q(1);            
                case 'circle'
                    circle_option = 'slope';
                    q = obj.IKSolveCircle(pos, circle_option, circle_params);
                otherwise
                    error('IKSolver dose not work');
            end       
        end
        
        %% inverse kinematics with numerical solution
        function q = IKSolveOptimize(obj, pos, option, alpha)
            %%to guarantee that the end-effector's pose is perpendicular to
            %%vertical surface, but it is very hard to get a analytical value
            q = zeros(1,5);
            height_limit = obj.arm.qlim(2,:);
            ty = obj.tool(2); tz = obj.tool(3);
            switch option
                case 'vertical'
                    if pos(3)>height_limit(2)
                        q(2) = height_limit(2);
                        q(3) = atan((pos(3)-q(2))/pos(2));%there is problem to calculate q3
                        q(1) = -atan(pos(1)/(pos(2)-ty*cos(q(3))));%there is problem to calculate q1
                        a = sin(q(1))*cos(q(3)); b = cos(q(1)); c= sin(q(1))*sin(q(3))*tz/ty;
                        if abs(a+c)<eps
                            q(5) = 0;
                        else
                            q5_tmp1 = 2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));
                            q5_tmp2 = 2*atan((b-sqrt(b^2+a^2-c^2))/(a+c));
                            if (q5_tmp1>pi/2) || (q5_tmp1<-pi/2)
                                q(5) = q5_tmp2;
                            else
                                q(5) = q5_tmp1;
                            end
                        end
                        tmp_value = pos(3)-q(2)-sin(q(3))*cos(q(5))*ty+cos(q(3))*tz;
                        q(4) = tmp_value/sin(q(3));
                    elseif pos(3)<height_limit(1)
                        q(2) = height_limit(1);
                        q(3) = atan((pos(3)-q(2))/pos(2));
                        q(1) = -atan(pos(1)/(pos(2)-ty*cos(q(3))));
                        a = sin(q(1))*cos(q(3)); b = cos(q(1)); c= sin(q(1))*sin(q(3))*tz/ty;
                        if abs(a+c)<eps
                            q(5) = 0;
                        else
                            q5_tmp1 = 2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));
                            q5_tmp2 = 2*atan((b-sqrt(b^2+a^2-c^2))/(a+c));
                            if (q5_tmp1>pi/2) || (q5_tmp1<-pi/2)
                                q(5) = q5_tmp2;
                            else
                                q(5) = q5_tmp1;
                            end
                        end
                        tmp_value = pos(3)-q(2)-sin(q(3))*cos(q(5))*ty+cos(q(3))*tz;
                        q(4) = tmp_value/sin(q(3));
                    else
                        q(2) = pos(3);
                        q(3) = 0;
                        q(1) = -atan(pos(1)/(pos(2)-ty));
                        if abs(q(1))<eps
                            q(4) = pos(2)-ty;
                        else
                            q(4) = -pos(1)/sin(q(1));
                        end
                        q(5) = -q(1);
                    end        
                case 'horizontal'
                    q(3) = -pi/6;
                    q(1) = -atan(pos(1)/(pos(2)-cos(q(3))*ty));
                    if abs(q(1))<eps
                        q(4) = pos(2)/cos(q(3))-ty;
                    else
                        q(4) = -pos(1)/(sin(q(1))*cos(q(3)));
                    end
                    a = sin(q(1))*cos(q(3)); b = cos(q(1)); c= sin(q(1))*sin(q(3))*tz/ty;
                        if abs(a+c)<eps
                            q(5) = 0;
                        else
                            q5_tmp1 = 2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));
                            q5_tmp2 = 2*atan((b-sqrt(b^2+a^2-c^2))/(a+c));
                            if (q5_tmp1>pi/2) || (q5_tmp1<-pi/2)
                                q(5) = q5_tmp2;
                            else
                                q(5) = q5_tmp1;
                            end
                        end
                    q(2) = pos(3)-sin(q(3))*q(4) - sin(q(3))*cos(q(5))*ty - cos(q(3))*tz;
                case 'circle'
                    circle_option = 'slope';
                    q = IKSolveCircle(pos, circle_option, alpha);
                otherwise
                    error('IKSolver dose not work');
            end           
            
        end
        
        
    end
    
end