classdef LspbTrajPlanner < handle
    % lspb trajectory(trapezoidal velocity profile)
    % TO DO: trajectory through a sequence of points
    
    properties
        vmax
        amax
        
        np
        t0
        tf
        v0
        vf
        q0
        qf
        ta
        td
        dir
        maxvel_reached
    end
    
    methods
        function obj = LspbTrajPlanner(pos, max_vel, max_acc, duration, vel_cons)
            % assume that: jmin = -jmax, amin = -amax, vmin = -vmax, t0 = 0
            % generic initial and final values of velocity
            % initial and final accelerations set to zeros
            obj.np = length(pos);
            obj.dir = sign(pos(2)-pos(1));
            h = abs(pos(2)-pos(1));
            if h>max_vel^2/max_acc
                obj.maxvel_reached = 1;
            else
                obj.maxvel_reached = 0;
            end
            if nargin==3
                obj.TransformPVA(pos, [0,0], max_vel, max_acc);
                obj.SetNoTimeLimit(h);
            elseif nargin==4
                obj.TransformPVA(pos, [0,0], max_vel, max_acc);
                obj.SetTimeLimit(h, duration);
            elseif nargin==5
                obj.TransformPVA(pos, vel_cons, max_vel, max_acc);
                obj.SetVelConstraint(h);
            end
        end

        function TransformPVA(obj, pos, vel, max_vel, max_acc)
            obj.q0 = pos(1)*obj.dir;
            obj.qf = pos(2)*obj.dir;
            obj.v0 = vel(1)*obj.dir;
            obj.vf = vel(2)*obj.dir;
            obj.vmax = 0.5*(obj.dir+1)*max_vel-0.5*(obj.dir-1)*max_vel;
            obj.amax = 0.5*(obj.dir+1)*max_acc-0.5*(obj.dir-1)*max_acc;
        end

        function SetNoTimeLimit(obj, h)
            obj.t0 = 0;
            if obj.maxvel_reached
                obj.ta = obj.vmax/obj.amax;
                obj.tf = h/obj.vmax+obj.vmax/obj.amax;
            else
                obj.ta = sqrt(h/obj.amax);
                obj.tf = 2*obj.ta;
                obj.vmax = h/obj.ta;
            end
            obj.td=obj.ta;
        end

        function SetTimeLimit(obj, h, duration)
            if isscalar(duration)
                obj.t0 = 0; obj.tf = duration;
            else
                obj.t0 = duration(1); obj.tf = duration(end);
            end
            time_length = obj.tf-obj.t0;
            if time_length>=h/obj.vmax+obj.vmax/obj.amax
                if obj.maxvel_reached
                    a = 1; b = -time_length*obj.amax; c = h*obj.amax;
                    obj.vmax = (-b-sqrt(b^2-4*a*c))/2/a;
                    obj.ta = obj.vmax/obj.amax;
                else
                    obj.vmax = 2*h/time_length;
                    obj.ta = 0.5*time_length;
                    obj.amax = obj.vmax/obj.ta;
                end
                obj.td=obj.ta;
            else
                error('input a larger time')
            end
        end

        function SetVelConstraint(obj, h)
            obj.t0 = 0;
            if obj.amax*h<0.5*abs(obj.v0^2-obj.vf^2)
                error('the trajectory does not exit');
            else
                if h*obj.amax>(obj.vmax^2-0.5*(obj.v0^2+obj.vf^2))
                    obj.maxvel_reached = 1;
                    obj.ta = (obj.vmax-obj.v0)/obj.amax;
                    obj.td = (obj.vmax-obj.vf)/obj.amax;
                    obj.tf = h/obj.vmax+0.5*obj.vmax/obj.amax*(1-obj.v0/obj.vmax)^2 ...
                            +0.5*obj.vmax/obj.amax*(1-obj.vf/obj.vmax)^2;
                else
                    obj.maxvel_reached = 0;
                    obj.vmax = sqrt(h*obj.amax+0.5*(obj.v0^2+obj.vf^2));
                    obj.ta = (obj.vmax-obj.v0)/obj.amax;
                    obj.td = (obj.vmax-obj.vf)/obj.amax;
                    obj.tf = obj.ta+obj.td;
                end
            end
        end
        
        function [pos, vel, acc] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = [];
            time_pnts = (obj.tf-obj.t0)/dt;
            for idx = 0:time_pnts
                t = obj.t0+dt*idx;
                [p, v, a] = obj.GenerateMotion(t);
                pos = [pos, p];
                vel = [vel, v];
                acc = [acc, a];
            end
        end
        
        function [p, v, a] = GenerateMotion(obj, t)
%             vm = obj.vmax*obj.dir;
%             am = obj.amax*obj.dir;
            if obj.maxvel_reached
                if t>=obj.t0 && t<obj.ta+obj.t0
                    p = obj.q0+obj.v0*(t-obj.t0)+0.5*(obj.vmax-obj.v0)/obj.ta*(t-obj.t0)^2;
                    v = obj.v0+(obj.vmax-obj.v0)/obj.ta*(t-obj.t0);
                    a = obj.amax;
                elseif t>=obj.ta+obj.t0 && t<obj.tf-obj.td
                    p = obj.q0+0.5*obj.v0*obj.ta+obj.vmax*(t-obj.t0-0.5*obj.ta);
                    v = obj.vmax;
                    a = 0;
                elseif t>=obj.tf-obj.td && t<=obj.tf
                    p = obj.qf-obj.vf*(obj.tf-t)-0.5*(obj.vmax-obj.vf)/obj.td*(obj.tf-t)^2;
                    v = obj.vf+(obj.vmax-obj.vf)/obj.td*(obj.tf-t);
                    a = -obj.amax;
                end
            else
                if t>=obj.t0 && t<=obj.t0+obj.ta
                    p = obj.q0+obj.v0*(t-obj.t0)+0.5*(obj.vmax-obj.v0)/obj.ta*(t-obj.t0)^2;
                    v = obj.v0+(obj.vmax-obj.v0)/obj.ta*(t-obj.t0);
                    a = obj.amax;
                elseif t> obj.tf-obj.td && t<=obj.tf
                    p = obj.qf-obj.vf*(obj.tf-t)-0.5*(obj.vmax-obj.vf)/obj.td*(obj.tf-t)^2;
                    v = obj.vf+(obj.vmax-obj.vf)/obj.td*(obj.tf-t);
                    a = -obj.amax;
                end
            end
            p = p*obj.dir; v = v*obj.dir; a = a*obj.dir;
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

