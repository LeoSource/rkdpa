classdef LspbTrajPlanner < handle
    % lspb trajectory(trapezoidal velocity profile)
    % TO DO: trajectory through a sequence of points
    % TO DO: add preassigned acceleration and velocity
    
    properties
        vmax
        amax
        
        np
        t0
        tf
        pos
        ta
        dir
        maxvel_reached
    end
    
    methods
        function obj = LspbTrajPlanner(pos, max_vel, max_acc, duration)
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
                obj.SetNoTimeLimit(h, max_vel, max_acc);
            else
                obj.SetTimeLimit(h, max_vel, max_acc, duration);
            end
        end

        function SetNoTimeLimit(obj, h, max_vel, max_acc)
            obj.t0 = 0;
            obj.amax = max_acc;
            if obj.maxvel_reached
                obj.ta = max_vel/max_acc;
                obj.tf = h/max_vel+max_vel/max_acc;
                obj.vmax = max_vel;
            else
                obj.ta = sqrt(h/max_acc);
                obj.tf = 2*obj.ta;
                obj.vmax = h/obj.ta;
            end
        end

        function SetTimeLimit(obj, h, max_vel, max_acc, duration)
            if isscalar(duration)
                obj.t0 = 0; obj.tf = duration;
            else
                obj.t0 = duration(1); obj.tf = duration(end);
            end
            time_length = obj.tf-obj.t0;
            if time_length>=h/max_vel+max_vel/max_acc
                if obj.maxvel_reached
                    obj.amax = max_acc;
                    a = 1; b = -time_length*obj.amax; c = h*obj.amax;
                    obj.vmax = (-b-sqrt(b^2-4*a*c))/2/a;
                    obj.ta = obj.vmax/obj.amax;
                else
                    obj.vmax = 2*h/time_length;
                    obj.ta = 0.5*time_length;
                    obj.amax = obj.vmax/obj.ta;
                end
            else
                error('input a larger time')
            end
        end
        
        function [pos, vel, acc] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = [];
            for t = obj.t0:dt:obj.tf
                [p, v, a] = obj.GenerateMotion(t);
                pos = [pos, p];
                vel = [vel, v];
                acc = [acc, a];
            end
        end
        
        function [p, v, a] = GenerateMotion(obj, t)
            vm = obj.vmax*obj.dir;
            am = obj.amax*obj.dir;
            if obj.maxvel_reached
                if t>=obj.t0 && t<obj.ta+obj.t0
                    p = obj.pos(1)+0.5*am*(t-obj.t0)^2;
                    v = am*(t-obj.t0);
                    a = am;
                elseif t>=obj.ta+obj.t0 && t<obj.tf-obj.ta
                    p = obj.pos(1)+am*obj.ta*(t-obj.t0-0.5*obj.ta);
                    v = vm;
                    a = 0;
                elseif t>=obj.tf-obj.ta && t<=obj.tf
                    p = obj.pos(2)-0.5*am*(obj.tf-t)^2;
                    v = am*(obj.tf-t);
                    a = -am;
                end
            else
                if t>=obj.t0 && t<=obj.t0+obj.ta
                    p = obj.pos(1)+0.5*am*(t-obj.t0)^2;
                    v = am*(t-obj.t0);
                    a = am;
                elseif t> obj.tf-obj.ta && t<=obj.tf
                    p = obj.pos(2)-0.5*am*(obj.tf-t)^2;
                    v = am*(obj.tf-t);
                    a = -am;
                end
            end
        end
        
        function PlotAVP(obj, dt)
            [q, dq, ddq] = obj.GenerateTraj(dt);
            t = obj.t0:dt:obj.tf;
            subplot(3,1,1); plot(t, q); grid on; ylabel('position');     
            subplot(3,1,2); plot(t, dq); grid on; ylabel('velocity');
            subplot(3,1,3); plot(t, ddq); grid on; ylabel('acceleration');
        end
        
        
    end
end

