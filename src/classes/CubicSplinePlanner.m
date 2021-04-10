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

        acceleration
        lambda
        weight
    end
    
    methods
        function obj = CubicSplinePlanner(pos, t, option, v_clamped)
            obj.pos = pos;
            obj.style = option;
            if strcmp(option, 'clamped') || strcmp(option, 'smooth')
                obj.v_clamped = v_clamped;
            else
                obj.v_clamped = zeros(1, 2);
            end
            obj.np = length(pos);
            obj.duration = t;
%             obj.poly_params = obj.CalcPolyParams(pos);
            obj.poly_params = obj.CalcParamsBaseonAcc(pos);
        end

        function SetSmoothParams(obj, mu, w)
            obj.lambda = (1-mu)/6/mu;
            if nargin>2
                obj.weight = w;
            else
                obj.weight = [0, ones(1, obj.np-2), 0];
            end
            obj.poly_params = obj.CalcSmoothParams(obj.pos);
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

        function acc = CalcPointsAcc(obj, pos)
            n = obj.np;
            T = zeros(1, n-1);
            for idx=1:n-1
                T(idx) = obj.duration(idx+1)-obj.duration(idx);
            end
            rhs = zeros(n, 1);
            lhs = zeros(n, n);
            rhs(1) = 6*((pos(2)-pos(1))/T(1)-obj.v_clamped(1));
            rhs(n) = 6*(obj.v_clamped(2)-(pos(n)-pos(n-1))/T(n-1));
            lhs(1, 1) = 2*T(1); lhs(1, 2) = T(1);
            lhs(n, n-1) = T(n-1); lhs(n, n) = 2*T(n-1);
            for idx=2:n-1
                tmp_rhs = (pos(idx+1)-pos(idx))/T(idx)-(pos(idx)-pos(idx-1))/T(idx-1);
                rhs(idx) = 6*tmp_rhs;
                lhs(idx, idx-1) = T(idx-1);
                lhs(idx, idx) = 2*(T(idx-1)+T(idx));
                lhs(idx, idx+1) = T(idx);
            end
            acc = lhs\rhs;
        end

        function params = CalcParamsBaseonAcc(obj, pos)
            n = obj.np;
            params = zeros(4, n-1);
            acc = obj.CalcPointsAcc(pos);
            for idx=1:n-1
                rhs = [pos(idx); pos(idx+1); acc(idx); acc(idx+1)];
                lhs = [obj.PolyPos(obj.duration(idx)); obj.PolyPos(obj.duration(idx+1));...
                        obj.PolyAcc(obj.duration(idx)); obj.PolyAcc(obj.duration(idx+1))];
                params(:, idx) = lhs\rhs;
            end
        end

        function via_points = CalcApproximatePoints(obj, pos)
            n = obj.np;
            T = zeros(1, n-1);
            for idx=1:n-1
                T(idx) = obj.duration(idx+1)-obj.duration(idx);
            end
            C = zeros(n, n);
            A = zeros(n, n);
            C(1, 1) = -6/T(1); C(1, 2) = 6/T(1);
            C(n, n-1) = 6/T(n-1); C(n, n) = -6/T(n-1);
            A(1, 1) = 2*T(1); A(1, 2) = T(1);
            A(n, n-1) = T(n-1); A(n, n) = 2*T(n-1);
            for idx=2:n-1
                C(idx, idx-1) = 6/T(idx-1);
                C(idx, idx) = -6/T(idx-1)-6/T(idx);
                C(idx, idx+1) = 6/T(idx);
                A(idx, idx-1) = T(idx-1);
                A(idx, idx) = 2*(T(idx-1)+T(idx));
                A(idx, idx+1) = T(idx);
            end
            w = zeros(1, n);
            for idx=1:n
                if abs(obj.weight(idx))<1e-5
                    w(idx) = 0;
                else
                    w(idx) = 1/obj.weight(idx);
                end
            end
            rhs = C*pos';
            lhs = A+obj.lambda*C*diag(w)*C';
            acc = lhs\rhs;

            via_points = pos'-obj.lambda*diag(w)*C'*acc;
            via_points = reshape(via_points, 1, n);
        end

        function params = CalcSmoothParams(obj, pos)
            via_points = obj.CalcApproximatePoints(pos);
            params = obj.CalcParamsBaseonAcc(via_points);
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

 