classdef LspbTrajPlanner < handle
    % lspb trajectory(trapezoidal velocity profile)    
    % TO DO: trajectory through a sequence of points
    % TO DO: add preassigned acceleration and velocity
    
    properties
        max_vel
        max_acc
        max_jerk
        const_phase
        
        np
        tf
        tc
        pos
    end
    
    methods
        function obj = LspbTrajPlanner(pos, tf, max_vel, max_acc, option)
        % assume that: jmin = -jmax, amin = -amax, vmin = -vmax, t0 = 0
        % generic initial and final values of velocity
        % initial and final accelerations set to zeros
            obj.tf = tf;
            obj.pos = pos;
            obj.np = length(pos);
            obj.max_vel = abs(max_vel)*sign(pos(2)-pos(1));
            obj.max_acc = abs(max_acc)*sign(pos(2)-pos(1));
            obj.max_jerk = 100;
            if strcmp(option, 'limitacc')
                tmp_value = 4*abs(pos(2) - pos(1))/tf^2;
                if abs(max_acc)<tmp_value
                    error('cannot arrive at the goal in the time');
                end
                obj.tc = tf/2-0.5*sqrt((tf^2*obj.max_acc-4*(pos(2)-pos(1)))/obj.max_acc);
            elseif strcmp(option, 'limitvel')
                low_value = abs(pos(2)-pos(1))/tf;
                up_value = 2*low_value;
                if abs(max_vel)>low_value && abs(max_vel)<up_value
                    obj.const_phase = 1;
                    obj.tc = (pos(1)-pos(2)+obj.max_vel*tf)/obj.max_vel;
                    obj.max_acc = (obj.max_vel)^2/(pos(1)-pos(2)+obj.max_vel*tf);
                elseif abs(max_vel)>=up_value
                    obj.const_phase = 0;
                    obj.tc = 0.5*tf;
                    obj.max_vel = 2*(pos(2)-pos(1))/tf;
                    obj.max_acc = obj.max_vel/obj.tc;
                else
                    error('input the right maximum velocity')
                end
            else
                error('input the right option');
            end
        end
        
      
        function [pos, vel, acc] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = [];
            for t = 0:dt:obj.tf
                [p, v, a] = obj.GenerateMotion(t);
                pos = [pos, p];
                vel = [vel, v];
                acc = [acc, a];
            end            
        end       

        function [p, v, a] = GenerateMotion(obj, t)
            if obj.const_phase==1
                if t>=0 && t<=obj.tc
                    p = obj.pos(1)+0.5*obj.max_acc*t^2;
                    v = obj.max_acc*t;
                    a = obj.max_acc;
                elseif t>obj.tc && t<=(obj.tf-obj.tc)
                    p = obj.pos(1)+obj.max_acc*obj.tc*(t-0.5*obj.tc);
                    v = obj.max_acc*obj.tc;
                    a = 0;
                elseif t>(obj.tf-obj.tc) && t<=obj.tf
                    p = obj.pos(2)-0.5*obj.max_acc*(obj.tf-t)^2;
                    v = obj.max_acc*(obj.tf-t);
                    a = -obj.max_acc;
                end
            else
                if t>=0 && t<=obj.tc
                    p = obj.pos(1)+0.5*obj.max_acc*t^2;
                    v = obj.max_acc*t;
                    a = obj.max_acc;
                elseif t>obj.tc && t<=obj.tf
                    p = obj.pos(2)-0.5*obj.max_vel*(obj.tf-t)^2/(obj.tf-obj.tc);
                    v = -obj.max_vel/(obj.tf-obj.tc)*(t-obj.tf);
                    a = -obj.max_acc;
                end
            end
        end
        
        function PlotAVP(obj, dt)
            [q, dq, ddq] = obj.GenerateTraj(dt);
            t = 0:dt:obj.tf;
            subplot(3,1,1);
            plot(t, q); grid on; ylabel('position');
            
            subplot(3,1,2)
            plot(t, dq); grid on; ylabel('velocity');
            
            subplot(3,1,3)
            plot(t, ddq); grid on; ylabel('acceleration');
        end
        
        
    end
end

