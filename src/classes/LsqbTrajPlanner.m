classdef LsqbTrajPlanner < handle
    % lsqb trajectory(trapezoidal velocity profile)    
    % to do: add arbitrary position value
    
    properties
        max_vel
        max_acc
        max_jerk
        
        np
        tf
        tc
        pos
    end
    
    methods
        function obj = LsqbTrajPlanner(pos, tf, max_vel, max_acc, max_jerk)
            obj.tf = tf;
            obj.pos = pos;
            obj.np = length(pos);
            obj.max_vel = max_vel;
            obj.max_acc = abs(max_acc)*sign(pos(2)-pos(1));
            if nargin==5
                obj.max_jerk = max_jerk;
            else
                obj.max_jerk = 100;
            end
            
            tmp_value = 4*abs(pos(2) - pos(1))/tf^2;
            if abs(max_acc)<tmp_value
                error('robot cannot arrive at the goal in the time');
            end
            obj.tc = tf/2-0.5*sqrt((tf^2*obj.max_acc-4*(pos(2)-pos(1)))/obj.max_acc);
        end
        
        
        
        
        function [pos, vel, acc] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = [];
            for t = 0:dt:obj.tf
                if t>=0 && t<=obj.tc
                    q = obj.pos(1)+0.5*obj.max_acc*t^2;
                    v = obj.max_acc*t;
                    a = obj.max_acc;
                elseif t>obj.tc && t<=(obj.tf-obj.tc)
                    q = obj.pos(1)+obj.max_acc*obj.tc*(t-0.5*obj.tc);
                    v = obj.max_acc*obj.tc;
                    a = 0;
                elseif t>(obj.tf-obj.tc) && t<=obj.tf
                    q = obj.pos(2)-0.5*obj.max_acc*(obj.tf-t)^2;
                    v = obj.max_acc*(obj.tf-t);
                    a = -obj.max_acc;
                end
                pos = [pos, q];
                vel = [vel, v];
                acc = [acc, a];
            end
            
        end
        
        function PlotAVP(obj, dt)
            [q, dq, ddq] = obj.GenerateTraj(dt);
            t = 0:dt:obj.tf;
            plot(t, q);
            ylabel('position');
            
            figure
            plot(t, dq);
            ylabel('velocity');
            
            figure
            plot(t, ddq);
            ylabel('acceleration');
        end
        
        
    end
end

