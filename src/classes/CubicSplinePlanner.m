%   Cubic Spline Trajectory Planner Class:
%   A specific class that plans and generates cubilc spline
%   There are 3 kinds of cubic slpine: clamped, natural and cyclic
%   The spline is not smoothed by default, otherwise, you can decide the
%   the smooth style: set smooth weight or set smooth tolerance, 
%   The spline is not time optimized by default, otherwise, you can decide the
%   time optimized style: total time or constrains of velocity and acceleration
%   Referance: 
%   trajectory planning for automatic machines and robots, chapter 4.4
%   Author:
%   liao zhixiang, zhixiangleo@163.com

%   Note:
%   1. use only one kind of spline style, i.e, style, smooth_stylem
%   and time_optimized, as much as possible
%   TO DO:
%   1. optimize the constructor logic according to different spline options,
%   i.e, style, smooth_stylem and time_optimized
%   2. solve the conflict between smooth and time optimize

classdef CubicSplinePlanner < handle 
    properties
        np
        poly_params
        duration
        pos
        v_clamped
        style
        
        smooth_style
        lambda
        weight
        smooth_err
        smooth_tol

        time_optimized
        amax
        vmax
    end
    
    methods
        %% Constructor and Get the 3rd Polynomial Parameters
        function obj = CubicSplinePlanner(pos, t, option, v_clamped)
            obj.pos = pos;
            obj.np = length(pos);
            obj.style = option;
            if strcmp(option, 'clamped')
                obj.v_clamped = v_clamped;
            else
                obj.v_clamped = zeros(1, 2);
            end
            if length(t)==1 && t<0
                obj.time_optimized = 'minimum';
                obj.amax = 10;
                obj.vmax = 10;
            elseif length(t)==1 && t>0
                obj.time_optimized = 'gentle';
                obj.duration = t*obj.CalcIntervals(pos);
                obj.poly_params = obj.CalcPolyParams(pos);
            else
                obj.duration = t;
                obj.poly_params = obj.CalcPolyParams(pos);
            end
            obj.smooth_style = 'none';
            obj.smooth_err = 0;
            obj.smooth_tol = 0;
            obj.weight = [0, ones(1, obj.np-2), 0];
%             obj.poly_params = obj.CalcParamsBaseonAcc(pos);
        end

        function SetSmoothWeight(obj, mu, w)
            obj.lambda = (1-mu)/6/mu;
            obj.smooth_style = 'weight';
            if nargin>2
                obj.weight = w;
            else
                obj.weight = [0, ones(1, obj.np-2), 0];
            end
            obj.poly_params = obj.CalcSmoothParams(obj.pos);
        end

        function SetSmoothTolerance(obj, tol)
            obj.smooth_tol = tol;
            obj.smooth_style = 'tolerance';
            obj.poly_params = obj.CalcSmoothParams(obj.pos);
        end

        function SetTimeOptimizedStyle(obj, style)
            obj.time_optimized = style;
            tf = obj.duration(end);
            obj.duration = tf*obj.CalcIntervals(obj.pos);
            obj.poly_params = obj.CalcPolyParams(obj.pos);
        end

        function SetTimeOptimizedConstrtaints(obj, vmax, amax)%seems like lspb interpolation
            obj.vmax = vmax;
            obj.amax = amax;
            obj.duration = obj.CalcOptimizedIntervals();
            obj.poly_params = obj.CalcPolyParams(obj.pos);
        end

        %% Elementary Row Vector for Position, Velocity and Acceleration
        function res = PolyPos(obj, t)
            res = [1, t, t^2, t^3];
        end

        function res = PolyVel(obj, t)
            res = [0, 1, 2*t, 3*t^2];
        end

        function res = PolyAcc(obj, t)
            res = [0, 0, 2, 6*t];
        end

        %% Calculate Polynomial Parameters with 3 types: clamped, natural, cyclic
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

        %% Calculate Polynomial Parameters Based on Acceleration
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

        %% Smoothing Cubic Spline
        function params = CalcSmoothParams(obj, pos)
            if strcmp(obj.smooth_style, 'weight')
                via_points = obj.CalcApproximatePoints(pos);
            elseif strcmp(obj.smooth_style, 'tolerance')
                mu = obj.CalcSmoothScale(pos);
                obj.lambda = (1-mu)/6/mu;
                via_points = obj.CalcApproximatePoints(pos);
            end
            obj.smooth_err = max(via_points-pos);
            params = obj.CalcPolyParams(via_points);
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

        function res = CalcSmoothScale(obj, pos)
            max_it_nums = 50; it_nums = 0;
            s_mu = 0; e_mu = 1;
            while obj.smooth_err>obj.smooth_tol || it_nums<max_it_nums
                mu = 0.5*(s_mu+e_mu);
                obj.lambda = (1-mu)/6/mu;
                via_points = obj.CalcApproximatePoints(pos);
                obj.smooth_err = max(via_points-pos);
                if obj.smooth_err>obj.smooth_tol
                    s_mu = mu;
                else
                    e_mu = mu;
                end
                it_nums = it_nums+1;
            end
            res = mu;
        end

        %% Choice of the Time Instants and Optimization
        function duration = CalcIntervals(obj, pos)
            n = obj.np;
            if strcmp(obj.time_optimized, 'fast')
                dk = ones(1,n-1)/(n-1);
            elseif strcmp(obj.time_optimized, 'middle')
                dk = zeros(1,n-1);
                for idx=1:n-1
                    dk(idx) = abs(pos(idx+1)-pos(idx));
                end
            elseif strcmp(obj.time_optimized, 'gentle')
                dk = zeros(1,n-1);
                for idx=1:n-1
                    dk(idx) = sqrt(abs(pos(idx+1)-pos(idx)));
                end
            end
            d = sum(dk);
            t = zeros(1, n); t(end) = 1;
            for idx=2:n-1
                t(idx) = t(idx-1)+dk(idx)/d;
            end
            duration = t;
        end

        function duration = CalcOptimizedIntervals(obj)
            n = obj.np;
            A = []; b = [];
            Aeq = []; beq = [];
            x0 = ones(1, n-1);
            options = optimoptions(@fmincon, 'MaxFunEvals', 15000, 'Display', 'iter', 'ConstraintTolerance', 1e-6);
            [x, fval] = fmincon(@obj.ObjFunction, x0, A, b, Aeq, beq, [], [], @obj.CondFunction, options);
            duration = zeros(1, n);
            for idx=2:n
                duration(idx) = duration(idx-1)+x(idx-1);
            end
        end

        function objective = ObjFunction(obj, x)
            objective = sum(x);
        end

        function [c, ceq] = CondFunction(obj, x)
            n = obj.np;
            num_opts = 200;
            condvmax = zeros(num_opts, 1);
            condvmin = zeros(num_opts, 1);
            condamax = zeros(num_opts, 1);
            condamin = zeros(num_opts, 1);
            condpos = zeros(n, 1);
            time = zeros(1, n);
            for idx=2:n
                time(idx) = time(idx-1)+x(idx-1);
            end
            obj.duration = time;
            obj.poly_params = obj.CalcPolyParams(obj.pos);
            t = linspace(time(1), time(end), num_opts);
            for idx=1:num_opts
                [~, v, a] = obj.GenerateMotion(t(idx));
                condvmax(idx) = v-obj.vmax;
                condvmin(idx) = -obj.vmax-v;
                condamax(idx) = a-obj.amax;
                condamin(idx) = -obj.amax-a;
            end
            for idx=1:n
                [p, ~, ~] = obj.GenerateMotion(time(idx));
                condpos(idx) = p-obj.pos(idx);
            end
            ceq = condpos;
            c = [condvmax; condvmin; condamax; condamin];
        end
        
        %% Generate the Trajectory
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
        
        %% Plot Function for Test and Presentation
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

 