classdef CubicSplinePlanner < handle
    % cubic splines trajectory planning  
    % ref: trajectory planning for automatic machines and robots, chapter 4.4
    
    properties
        np
        poly_params
        duration
        pos
        v_clamped
        style
    end
    
    methods
        function obj = CubicSplinePlanner(pos, t, option, v_clamped)
            obj.pos = pos;
            obj.style = option;
            if strcmp(option, 'clamped')
                obj.v_clamped = v_clamped;
            else
                obj.v_clamped = zeros(1, 2);
            end
            obj.np = length(pos);
            obj.duration = t;
            obj.poly_params = zeros(4*(obj.np-1), 1);
            obj.poly_params = obj.CalcPolyParams(pos);
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

        function rhs = RhsMatPos(obj, pos)
            n = obj.np;
            rhs = zeros(2*(n-1), 1);
            for idx=1:n-1
                rhs(2*idx-1) = pos(idx);
                rhs(2*idx) = pos(idx+1);
            end
        end

        function lhs = LhsMatPos(obj)
            n = obj.np;
            tt = obj.duration;
            lhs = zeros(2*(n-1), 4*(n-1));
            for idx=1:n-1
                lhs(2*idx-1, 4*idx-3:4*idx) = obj.PolyPos(tt(idx));
                lhs(2*idx, 4*idx-3:4*idx) = obj.PolyPos(tt(idx+1));
            end
        end

        function lhs = LhsMatVel(obj)
            n = obj.np;
            tt = obj.duration;
            lhs = zeros(n-2, 4*(n-1));
            for idx=1:n-2
                lhs(idx, 4*idx-3:4*idx) = obj.PolyVel(tt(idx+1));
                lhs(idx, 4*idx+1:4*idx+4) = -obj.PolyVel(tt(idx+1));
            end
        end

        function lhs = LhsMatAcc(obj)
            n = obj.np;
            tt = obj.duration;
            lhs = zeros(n-2, 4*(n-1));
            for idx=1:n-2
                lhs(idx, 4*idx-3:4*idx) = obj.PolyAcc(tt(idx+1));
                lhs(idx, 4*idx+1:4*idx+4) = -obj.PolyAcc(tt(idx+1));
            end
        end

        function params = CalcPolyParams(obj, pos)
            n = obj.np;
            % necesssary constraints for interpolation points
            rhs = zeros(4*(n-1), 1);
            lhs = zeros(4*(n-1), 4*(n-1));
            rhs(1:2*(n-1)) = obj.RhsMatPos(pos);
            lhs(1:2*(n-1), :) = obj.LhsMatPos();
            lhs(2*n-1:3*n-4, :) = obj.LhsMatVel();
            lhs(3*n-3:4*n-6, :) = obj.LhsMatAcc();

            % depends on trajectory type: clamped, natural, cyclic
            switch obj.style
            case 'clamped'
                lhs(4*n-5, 1:4) = obj.PolyVel(obj.duration(1));
                lhs(4*n-4,4*n-7:4*n-4) = obj.PolyVel(obj.duration(n));
                rhs(4*n-5) = obj.v_clamped(1);
                rhs(4*n-4) = obj.v_clamped(2);
            case 'natural'
                lhs(4*n-5, 1:4) = obj.PolyAcc(obj.duration(1));
                lhs(4*n-4, 4*n-7:4*n-4) = obj.PolyAcc(obj.duration(end));
            case 'cyclic'
                lhs(4*n-5, 1:4) = obj.PolyVel(obj.duration(1));
                lhs(4*n-5, 4*n-7:4*n-4) = -obj.PolyVel(obj.duration(end));
                lhs(4*n-4, 1:4) = obj.PolyAcc(obj.duration(1));
                lhs(4*n-4, 4*n-7:4*n-4) = -obj.PolyAcc(obj.duration(end));
            end

            params = lhs\rhs;
            params = reshape(params, 4, n-1);
        end


        function [pos, vel, acc] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = [];
            for t = 0:dt:obj.duration(end)
                [p, v, a] = obj.GenerateMotion(t);
                pos = [pos, p];
                vel = [vel, v];
                acc = [acc, a];
            end            
        end       

        function [p, v, a] = GenerateMotion(obj, t)
            time = obj.duration;
            idx = discretize(t, time);
            p = obj.PolyPos(t)*obj.poly_params(:,idx);
            v = obj.PolyVel(t)*obj.poly_params(:,idx);
            a = obj.PolyAcc(t)*obj.poly_params(:,idx);
        end
        
        function PlotAVP(obj, dt)
            [q, dq, ddq] = obj.GenerateTraj(dt);
            t = 0:dt:obj.duration(end);
            subplot(3,1,1);
            scatter(obj.duration, obj.pos); hold on
            plot(t, q, 'k-'); grid on; ylabel('position');            
            subplot(3,1,2)
            plot(t, dq, 'k-'); grid on; ylabel('velocity');           
            subplot(3,1,3)
            plot(t, ddq, 'k-'); grid on; ylabel('acceleration');
        end
        
        
    end
end

