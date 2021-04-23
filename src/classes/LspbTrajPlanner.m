classdef LspbTrajPlanner < handle
    % lspb trajectory(trapezoidal velocity profile)
    % TO DO: trajectory through a sequence of points
    % TO DO: add preassigned acceleration and velocity
    
    properties
        vmax
        amax
        
        np
        tf
        pos
        ta
        dir
        maxvel_reached
    end
    
    methods
        function obj = LspbTrajPlanner(pos, max_vel, max_acc, tf)
            % assume that: jmin = -jmax, amin = -amax, vmin = -vmax, t0 = 0
            % generic initial and final values of velocity
            % initial and final accelerations set to zeros
            obj.pos = pos;
            obj.np = length(pos);
            obj.dir = sign(pos(2)-pos(1));
            h = abs(pos(2)-pos(1));
            if h>max_vel^2/max_acc
                obj.maxvel_reached = 1;
            else
                obj.maxvel_reached = 0;
            end

            if nargin<4
                if obj.maxvel_reached
                    obj.ta = max_vel/max_acc;
                    obj.tf = h/max_vel+max_vel/max_acc;
                    obj.amax = max_acc;
                    obj.vmax = max_vel;
                else
                    obj.ta = sqrt(h/max_acc);
                    obj.tf = 2*obj.ta;
                    obj.vmax = h/obj.ta;
                    obj.amax = max_acc;
                end
            else
                if tf>=h/max_vel+max_vel/max_acc
                    obj.tf = tf;
                    if obj.maxvel_reached
                        obj.amax = max_acc;
                        a = 1; b = -obj.tf*obj.amax; c = h*obj.amax;
                        obj.vmax = (-b-sqrt(b^2-4*a*c))/2/a;
                        obj.ta = obj.vmax/obj.amax;
                    else
                        obj.vmax = 2*h/tf;
                        obj.ta = 0.5*tf;
                        obj.amax = obj.vmax/obj.ta;
                    end
                else
                    error('input the larger time')
                end
            end
            obj.vmax = abs(obj.vmax)*obj.dir;
            obj.amax = abs(obj.amax)*obj.dir;
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
            if obj.maxvel_reached
                if t>=0 && t<obj.ta
                    p = obj.pos(1)+0.5*obj.amax*(t-0)^2;
                    v = obj.amax*(t-0);
                    a = obj.amax;
                elseif t>=obj.ta && t<obj.tf-obj.ta
                    p = obj.pos(1)+obj.amax*obj.ta*(t-0-0.5*obj.ta);
                    v = obj.amax*obj.ta;
                    a = 0;
                elseif t>=obj.tf-obj.ta && t<=obj.tf
                    p = obj.pos(2)-0.5*obj.amax*(obj.tf-t)^2;
                    v = obj.amax*(obj.tf-t);
                    a = -obj.amax;
                end
            else
                if t>=0 && t<=0+obj.ta
                    p = obj.pos(1)+0.5*obj.amax*(t-0)^2;
                    v = obj.amax*t;
                    a = obj.amax;
                elseif t> obj.tf-obj.ta && t<=obj.tf
                    p = obj.pos(2)-0.5*obj.amax*(obj.tf-t)^2;
                    v = obj.amax*(obj.tf-t);
                    a = -obj.amax;
                end
            end
        end
        
        function PlotAVP(obj, dt)
            [q, dq, ddq] = obj.GenerateTraj(dt);
            t = 0:dt:obj.tf;
            subplot(3,1,1); plot(t, q); grid on; ylabel('position');     
            subplot(3,1,2); plot(t, dq); grid on; ylabel('velocity');
            subplot(3,1,3); plot(t, ddq); grid on; ylabel('acceleration');
        end
        
        
    end
end

