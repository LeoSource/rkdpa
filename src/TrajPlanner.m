classdef TrajPlanner < handle
    %   third order polynomial for trajectory plan(for now)
    %   to do: add other trajectory plan type 
    properties
        num
        dt
        tf
        
        poly_params
    end
    
    methods
        function obj = TrajPlanner(pos, tf)
            obj.tf = tf;
            num = length(pos);
            obj.num = num;
            obj.poly_params = zeros(4, num-1);
            dt = tf/(num-1);
            obj.dt = dt;
            t = 0:dt:tf;
            for idx=1:num-1
                vel(1) = 0;
                if idx==num-1
                    vel(idx+1) = 0;
                else
                    k1 = (pos(idx+1)-pos(idx))/dt;
                    k2 = (pos(idx+2)-pos(idx+1))/dt;
                    if sign(k1*k2)==1
                        vel(idx+1) = 0.5*(k1+k2);
                    else
                        vel(idx+1) = 0;
                    end
                end
                rhs = [pos(idx); pos(idx+1); vel(idx); vel(idx+1)];
                lhs = obj.LhsMat(t(idx), t(idx+1));
                obj.poly_params(:,idx) = lhs\rhs;
            end
        end
        
        function [pos, vel, acc] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = [];
            for t=0:dt:obj.tf
                [p, v, a] = obj.GenerateMotionState(t);
                pos = [pos, p];
                vel = [vel, v];
                acc = [acc, a];
            end
        end
        
        function [pos, vel, acc] = GenerateMotionState(obj, t)
            time = 0:obj.dt:obj.tf;
            idx = discretize(t, time);
            pos = obj.PolyPos(t)*obj.poly_params(:,idx);
            vel = obj.PolyVel(t)*obj.poly_params(:,idx);
            acc = obj.PolyAcc(t)*obj.poly_params(:,idx);
        end
        
        function res = PolyPos(obj, t)
            res = [1, t, t^2, t^3];
        end
        
        function res = PolyVel(obj, t)
            res = [0, 1, 2*t, 3*t^2];
        end
        
        function res = PolyAcc(obj, t)
            res = [0, 0, 2, 6*t];
        end
        
        function res = LhsMat(obj, t0, tf)
            res = [obj.PolyPos(t0); obj.PolyPos(tf); obj.PolyVel(t0); obj.PolyVel(tf)];
        end
            
    end
    
    
    
end