%   Cubic B-Spline Trajectory Planner Class:
%   A specific class that plans and generates cubic B-spline for 2d/3d trajectory
%   There are 2 kins of planner: interpolation and approximation
%   Generally, the definition of properties is 
%   via_pos(q): the given position that cubic B-spline should pass
%   ctrl_pos(p): the control position which composes the cubic B-spline parametric formula
%   n: number of via_pos
%   m: number of ctrl_pos, for interpolation, which equal to n+2
%   p: degree of B-spline, four cubic, which is 3
%   n_knot: number of knot vector, equal to m+p+1
%   Referance: 
%   trajectory planning for automatic machines and robots, chapter 8.4&8.5
%   Author:
%   liao zhixiang, zhixiangleo@163.com

classdef CubicBSplinePlanner < handle
    properties
        nump
        num_ctrlp
        pdegree
        knot_vec
        uknot_vec
        ctrl_pos

        option
    end

    methods
        %% Constructor of Class and Other Settings
        function obj = CubicBSplinePlanner(via_pos, option, uk)
            obj.nump = size(via_pos, 2);
            obj.pdegree = 3;
            if nargin>2
                if isscalar(uk)
                    obj.uknot_vec = uk*obj.CalcUKnot(via_pos);
                else
                    obj.uknot_vec = uk;
                end
            else
                obj.uknot_vec = obj.CalcUKnot(via_pos);
            end
            if strcmp(option, 'interpolation')
                obj.num_ctrlp = obj.nump+2;
                obj.knot_vec = obj.CalcKnotVec();
                obj.ctrl_pos = obj.CalcCtrlPos(via_pos);
            elseif strcmp(option, 'approximation')
                obj.num_ctrlp = 16;
                obj.knot_vec = obj.CalcApproKnotVec();
                obj.ctrl_pos = obj.CalcApproCtrlPos(via_pos);
            else
                error('error option')
            end
        end

        %% Calculate Control Position for Interpolation 
        function p = CalcCtrlPos(obj, q)
            n = obj.nump; m =obj.num_ctrlp; u_hat = obj.uknot_vec;
            dim = size(q, 1);
            p = zeros(dim, m);
            t1 = (q(:,2)-q(:,1))/(u_hat(2)-u_hat(1));%u(p+2)-u(p+1)
            tn = (q(:,n)-q(:,n-1))/(u_hat(n)-u_hat(n-1));
            p(:,1) = q(:,1);
            p(:,2) = q(:,1)+(u_hat(2)-u_hat(1))/3*t1;
            p(:,n+1) = q(:,n)-(u_hat(n)-u_hat(n-1))/3*tn;
            p(:,n+2) = q(:,n);
            B = zeros(n-2,n-2); R = zeros(n-2, dim);
            for idx=1:n-2
                if idx==1
                    R(idx,:) = q(:,2)'-obj.CalcBSplineCoeff(3,2,u_hat(2))*p(:,2)';
                    B(1,1) = obj.CalcBSplineCoeff(3, 3, u_hat(2));
                    B(1,2) = obj.CalcBSplineCoeff(3, 4, u_hat(2));
                elseif idx==n-2
                    R(idx,:) = q(:,n-1)'-obj.CalcBSplineCoeff(3, n+1, u_hat(n-1))*p(:,n+1)';
                    B(n-2, n-3) = obj.CalcBSplineCoeff(3, n-1, u_hat(n-1));
                    B(n-2, n-2) = obj.CalcBSplineCoeff(3, n, u_hat(n-1));
                else
                    R(idx,:) = q(:,idx+1)';
                    B(idx, idx-1) = obj.CalcBSplineCoeff(3, idx+1, u_hat(idx+1));
                    B(idx, idx) = obj.CalcBSplineCoeff(3, idx+2, u_hat(idx+1));
                    B(idx, idx+1) = obj.CalcBSplineCoeff(3, idx+3, u_hat(idx+1));
                end
            end
            p_tmp = B\R;
            p(:,3:n) = p_tmp';
        end

        %% Calculate Control Position for Approximation
        function p = CalcApproCtrlPos(obj, q)
            n = obj.nump; dim = size(q,1); u_hat = obj.uknot_vec;
            m = obj.num_ctrlp;
            p = zeros(dim, m);
            p(:,1) = q(:,1);
            p(:,m) = q(:,n);
            R = zeros(n-2, dim); B = zeros(n-2, m-2);
            for idx=2:n-1
                R(idx-1,:) = q(:,idx)'-obj.CalcBSplineCoeff(3,1,u_hat(idx))*q(:,1)'...
                                        -obj.CalcBSplineCoeff(3,m,u_hat(idx))*q(:,n)';
            end
            for nidx=2:n-1
                for midx=2:m-1
                    B(nidx-1, midx-1) = obj.CalcBSplineCoeff(3,midx, u_hat(nidx));
                end
            end
            W = diag(ones(n-2,1));
            pseinv_B = (B'*W*B)\(B'*W);
            p_tmp = pseinv_B*R;
            p(:,2:m-1) = p_tmp';
        end
        
        %% Calculate Knot Vector for Interpolation and Approximation
        function knot_vec = CalcKnotVec(obj)
            uk = obj.uknot_vec;
            m = obj.num_ctrlp; p = obj.pdegree;
            knot_vec = zeros(1, m+p+1);
            knot_vec(1:3) = deal(uk(1));
            knot_vec(end-2:end) = deal(uk(end));
            knot_vec(4:end-3) = uk;
        end

        function uk = CalcUKnot(obj, q)
            uk = zeros(obj.nump, 1);
            uk(end) = 1;
            d = 0;
            for idx=2:obj.nump
                d = d+norm(q(:,idx)-q(:,idx-1));
            end
            for idx=2:obj.nump-1
                uk(idx) = uk(idx-1)+norm(q(:,idx)-q(:,idx-1))/d;
            end
        end

        function knot_vec = CalcApproKnotVec(obj)
            n = obj.nump; m = obj.num_ctrlp; p =3;
            d = (n+1)/(m-p+1);
            uk = obj.uknot_vec;
            knot_vec = zeros(1, m+p+1);
            for jidx=2:m-p
                idx = floor(jidx*d);
                alph = jidx*d-idx;
                knot_vec(jidx+p) = (1-alph)*uk(idx-1)+alph*uk(idx);
            end
            knot_vec(1:p+1) = deal(uk(1));
            knot_vec(m+1:m+p+1) = deal(uk(end));
        end
        
        %% Standard B-Spline Basis Function and Derivative
        function b_coeff = CalcBSplineCoeff(obj, p, idx, u)
            if p==0
                if u>=obj.knot_vec(idx) && u<obj.knot_vec(idx+1)
                    b_coeff = 1;
                else
                    b_coeff = 0;
                end
            else
                den1 = obj.knot_vec(idx+p)-obj.knot_vec(idx);
                num1 = u-obj.knot_vec(idx);
                coef1 = Divide(num1, den1);
                den2 = obj.knot_vec(idx+p+1)-obj.knot_vec(idx+1);
                num2 = obj.knot_vec(idx+p+1)-u;
                coef2 = Divide(num2, den2);
                b_coeff = coef1*obj.CalcBSplineCoeff(p-1, idx, u)...
                            +coef2*obj.CalcBSplineCoeff(p-1, idx+1, u);
            end
        end

        function diff_bcoeff = DiffBSplineCoeff(obj, p, jidx, u, k)
            ak = zeros(1, k+1); b_coeff = zeros(k+1,1);
            for idx=0:k
                ak(idx+1) = obj.CalcDiffCoeff(k,idx,jidx);
                b_coeff(idx+1) = obj.CalcBSplineCoeff(p-k,jidx+idx,u);
            end
            diff_bcoeff = factorial(p)/factorial(p-k)*ak*b_coeff;
        end

        function ak = CalcDiffCoeff(obj, k, idx, jidx)
            p = obj.pdegree; u = obj.knot_vec;
            if k==0
                ak = 1;
            else
                if idx==0
                    ak = Divide(obj.CalcDiffCoeff(k-1, 0, jidx),(u(jidx+p-k+1)-u(jidx)));
                elseif idx==k
                    ak = -Divide(obj.CalcDiffCoeff(k-1, k-1, jidx), u(jidx+p+1)-u(jidx+k));
                else
                    num = obj.CalcDiffCoeff(k-1, idx, jidx)-obj.CalcDiffCoeff(k-1, idx-1, jidx);
                    den = u(jidx+p+idx-k+1)-u(jidx+idx);
                    ak = Divide(num, den);
                end
            end
        end

        %% Generate B-Spline Curve, Velocity and Acceleration
        function [p, v, a] = GenerateMotion(obj, u, du, ddu)
            p = obj.GeneratePos(u);
            v = obj.GenerateVel(u, du);
            a = obj.GenerateAcc(u, du, ddu);
        end

        function pos = GeneratePos(obj, u)
            m = obj.num_ctrlp;
            b_coeff = zeros(m, 1);
            if abs(u-max(obj.knot_vec))<eps
                b_coeff(end) = 1;
            else
                for idx=1:m
                    b_coeff(idx) = obj.CalcBSplineCoeff(3, idx, u);
                end
            end
            pos = obj.ctrl_pos*b_coeff;
        end

        function vel = GenerateVel(obj, u, du)
            m = obj.num_ctrlp;
            diff_bcoeff = zeros(m,1);
            for idx=1:m
                diff_bcoeff(idx) = obj.DiffBSplineCoeff(3, idx, u, 1);
            end
            % dp/dt=(dp/du)*(du/dt);
            vel = obj.ctrl_pos*diff_bcoeff*du;
        end

        function acc = GenerateAcc(obj, u, du, ddu)
            m = obj.num_ctrlp;
            diff_bcoeff = zeros(m,1);
            diff2_bcoeff = zeros(m,1);
            for idx=1:m
                diff_bcoeff(idx) = obj.DiffBSplineCoeff(3, idx, u, 1);
                diff2_bcoeff(idx) = obj.DiffBSplineCoeff(3, idx, u, 2);
            end
            % ddp/ddt = (dp/du)*ddu+(ddp/ddu)*du^2
            acc = obj.ctrl_pos*diff2_bcoeff*du^2+obj.ctrl_pos*diff_bcoeff*ddu;
        end

        function curve = GenerateBSpline(obj, du)
            curve = [];
            for u=obj.uknot_vec(1):du:obj.uknot_vec(end)
                p = obj.GeneratePos(u);
                curve = [curve, p];
            end
        end

        %% Plot Function for Test and Presentation
        function PlotAVP(obj, dt)
            pos = []; vel = []; acc = [];
            vel_norm = []; acc_norm = [];
            uk = obj.uknot_vec;
            uplanner1 = PolyTrajPlanner(uk(1:2), uk(1:2), [0,1], 5);
            uplanner2 = PolyTrajPlanner(uk(end-1:end), uk(end-1:end), [1,0], 5);
            for t=uk(1):dt:uk(end)
                if t>=uk(1) && t<=uk(2)
                    [u,du,ddu] = uplanner1.GenerateMotion(t);
                elseif t>=uk(end-1) && t<=uk(end)
                    [u,du,ddu] = uplanner2.GenerateMotion(t);
                else
                    u = t; du = 1; ddu = 0;
                end
                [p, v, a] = obj.GenerateMotion(u, du, ddu);
                pos = [pos, p]; vel = [vel, v]; acc = [acc, a];
                vel_norm = [vel_norm, norm(v)];
                acc_norm = [acc_norm, norm(a)];
            end
            t = uk(1):dt:uk(end);
            str_vel = {'vx', 'vy', 'vz', '|v|'};
            figure
            for idx=1:length(str_vel)
                subplot(4,1,idx)
                if idx~=4
                    plot(t,vel(idx,:),'k-'); grid on; ylabel(str_vel{idx});
                else
                    plot(t,vel_norm,'k-'); grid on; ylabel(str_vel{idx});
                end
            end
            str_acc = {'ax', 'ay', 'az', '|a|'};
            figure
            for idx=1:length(str_acc)
                subplot(4,1,idx)
                if idx~=4
                    plot(t,acc(idx,:),'k-'); grid on; ylabel(str_acc{idx});
                else
                    plot(t,acc_norm,'k-'); grid on; ylabel(str_acc{idx});
                end
            end
        end

        function PlotBSpline(obj, du)
            curve = obj.GenerateBSpline(du);
            figure
            plot2(curve', 'k-'); grid on;
            xlabel('x'); ylabel('y'); zlabel('z');
        end

    end
end

function res = Divide(num, den)
    if abs(num)<eps && abs(den)<eps
        res = 0;
    elseif abs(den)<eps
        den = 1;
        res = num/den;
    else
        res = num/den;
    end
end