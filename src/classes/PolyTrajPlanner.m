classdef PolyTrajPlanner < handle
    %   third and fifth order polynomial for trajectory plan(for now)
    %   TO DO: extends other style cubic splines
    %   ref: trajectory planning for automatic machines and robots, chapter 4.4  
    %   TO DO: smoothing cubic splines, chapter 4.4.5
    properties
        num
        tf
        dt
        
        poly_params
        order
    end

    
    
    methods
        %% Constructor and Get the Polynomial Parameters
        function obj = PolyTrajPlanner(pos, tf, order)
            obj.order = order;
            n = length(pos);
            obj.num = n;
            if length(tf)==n || length(tf)==1
                obj.tf = tf;
                obj.dt = tf(end)/(n-1);
            else
                error('dimension of positoin and time mismatch');
            end
            obj.poly_params = zeros(order+1, n-1);
            obj.poly_params = obj.PolyParams(pos, tf);
        end
        
        %% Generate the Trajectory
        function [pos, vel, acc] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = [];
            for t=0:dt:obj.tf(end)
                [p, v, a] = obj.GenerateMotion(t);
                pos = [pos, p];
                vel = [vel, v];
                acc = [acc, a];
            end
        end
        
        function [p, v, a] = GenerateMotion(obj, t)
            if length(obj.tf)==1
                time = 0:obj.dt:obj.tf;
            else
                time = obj.tf;
            end
            idx = discretize(t, time);
            p = obj.PolyPos(t)*obj.poly_params(:,idx);
            v = obj.PolyVel(t)*obj.poly_params(:,idx);
            a = obj.PolyAcc(t)*obj.poly_params(:,idx);
        end
        
        %% Elementary Row Vector for Position, Velocity and Acceleration
        function res = PolyPos(obj, t)
            if obj.order==3
                res = [1, t, t^2, t^3];
            elseif obj.order==5
                res = [1, t, t^2, t^3, t^4, t^5];
            end
        end
        
        function res = PolyVel(obj, t)
            if obj.order==3
                res = [0, 1, 2*t, 3*t^2];
            elseif obj.order==5
                res = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4];
            end
        end
        
        function res = PolyAcc(obj, t)
            if obj.order==3
                res = [0, 0, 2, 6*t];
            elseif obj.order==5
                res = [0, 0, 2, 6*t, 12*t^2, 20*t^3];
            end
        end
        
        %% Calculate Polynomial Parameters
        function params = PolyParams(obj, pos, tf)
            if obj.order==3
                if obj.num>2
                    params = obj.PolyContiAcc(pos, tf);
                else
                    params = obj.PolyAutoVel(pos, tf);
                end
            elseif obj.order==5
                rhs = [pos(1); pos(2); 0; 0; 0; 0];
                lhs = obj.LhsMat(0, tf);
                params = lhs\rhs;                
            end
                end
        
        function res = LhsMat(obj, t0, tf)
            if obj.order==3
                res = [obj.PolyPos(t0); obj.PolyPos(tf); obj.PolyVel(t0); obj.PolyVel(tf)];
            elseif obj.order==5
                res = [obj.PolyPos(t0); obj.PolyPos(tf);...
                            obj.PolyVel(t0); obj.PolyVel(tf);...
                            obj.PolyAcc(t0); obj.PolyAcc(tf)];
            end
        end
    
        function params = PolyAutoVel(obj, pos ,tf)
            % calculate the velocity heuristically 
            t = 0:obj.dt:tf;
            n = obj.num;
            for idx=1:n-1
                vel(1) = 0;
                if idx==n-1
                    vel(idx+1) = 0;
                else
                    k1 = (pos(idx+1)-pos(idx))/obj.dt;
                    k2 = (pos(idx+2)-pos(idx+1))/obj.dt;
                    if sign(k1*k2)==1
                        vel(idx+1) = 0.5*(k1+k2);
                    else
                        vel(idx+1) = 0;
                    end
                end
                rhs = [pos(idx); pos(idx+1); vel(idx); vel(idx+1)];
                lhs = obj.LhsMat(t(idx), t(idx+1));
                params(:,idx) = lhs\rhs;
            end                         
        end
        
        function params = PolyContiAcc(obj, pos, tf)
            % make the acceleration continuous
            n = obj.num;
            rhs = zeros(4*(n-1), 1);
            lhs = zeros(4*(n-1), 4*(n-1));
            if length(obj.tf)==1                
                t = 0:obj.dt:tf;
            else
                t = tf;
            end
            if obj.num>2
                rhs(1) = pos(1);
                rhs(2*(n-1)) = pos(n);
                lhs(2*(n-1)+1, 1:4) = obj.PolyVel(t(1));
                lhs(2*(n-1)+n, 4*n-7:4*n-4) = obj.PolyVel(t(n));
                rhs(2*(n-1)+1) = 2;
                rhs(2*(n-1)+n) = -3;
                for idx=2:n-1
                    rhs(2*idx-2) = pos(idx);
                    rhs(2*idx-1) = pos(idx);
                    lhs(2*(n-1)+idx, 4*idx-7:4*idx-4) = obj.PolyVel(t(idx));
                    lhs(2*(n-1)+idx, 4*idx-3:4*idx) = -obj.PolyVel(t(idx));
                    lhs(3*n-3+idx, 4*idx-7:4*idx-4) = obj.PolyAcc(t(idx));
                    lhs(3*n-3+idx, 4*idx-3:4*idx) = -obj.PolyAcc(t(idx));
                end
                
                for idx=1:n-1
                    lhs(2*idx-1, 4*idx-3:4*idx) = obj.PolyPos(t(idx));
                    lhs(2*idx, 4*idx-3:4*idx) = obj.PolyPos(t(idx+1));
                end
                params = lhs\rhs;
                params = reshape(params, 4, n-1);
            else
                error('error input position');
            end
            
        end
        
        %% Plot Function for Test and Presentation
        function PlotAVP(obj, dt)
            [pos, vel, acc] = obj.GenerateTraj(dt);
            t = 0:dt:obj.tf(end);
            motion_data = [pos; vel; acc];
            motion_name = {'position', 'velocity', 'acceleration'};
            for idx=1:3
                subplot(3, 1, idx);
                plot(t, motion_data(idx, :)); grid on
                ylabel(motion_name{idx});
            end                       
        end
        
    end
    
    
    
end