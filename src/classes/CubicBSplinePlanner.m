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
        ctrl_rpy

        option
        ori_planned
    end

    methods
        %% Constructor of Class and Other Settings
        function obj = CubicBSplinePlanner(via_posrpy, option, uk)
            if size(via_posrpy,1)==6
                obj.ori_planned = true;
            else
                obj.ori_planned = false;
            end
            %%%usually, uk is the final time of of planner%%%
            obj.pdegree = 3;
            obj.option = option;
            via_pos = via_posrpy(1:3,:);
            %%%input position is the control position%%%
            if strcmp(option, 'ctrlpos')
                obj.ctrl_pos = via_pos;
                if obj.ori_planned
                    axis_angle = RPY2AxisAngle(via_posrpy(4:6,:));
                    obj.ctrl_rpy = axis_angle;
                end
                obj.num_ctrlp = size(via_pos,2);
                obj.nump = obj.num_ctrlp-2;
                if nargin>2
                    if isscalar(uk)
                        obj.uknot_vec = uk*obj.CalcUKnot(via_pos);
                    else
                        obj.uknot_vec = uk;
                    end
                else
                    obj.uknot_vec = obj.CalcUKnot(via_pos);
                end
                obj.knot_vec = obj.CalcKnotVec();
            else                
                obj.nump = size(via_pos, 2);
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
                    if obj.ori_planned
                        axis_angle = RPY2AxisAngle(via_posrpy(4:6,:));
                        obj.ctrl_rpy = obj.CalcCtrlRot(axis_angle);
                    end
                elseif strcmp(option, 'approximation')
                    obj.num_ctrlp = 20;
                    obj.knot_vec = obj.CalcApproKnotVec();
                    obj.ctrl_pos = obj.CalcApproCtrlPos(via_pos);
                    if obj.ori_planned
                        axis_angle = RPY2AxisAngle(via_posrpy(4:6,:));
                        obj.ctrl_rpy = obj.CalcApproCtrlPos(axis_angle);
                    end
                else
                    error('error option')
                end
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

        function p = CalcCtrlRot(obj, q)
            n = obj.nump; m = obj.num_ctrlp; u_hat = obj.uknot_vec;
            dim = size(q,1);
            p = zeros(dim, m);
            p(:,1) = q(:,1);
            quat1 = AxisAngle2Quat(q(:,1));
            quat2 = AxisAngle2Quat(q(:,2));
            qi = quatinterp(quat1',quat2',0.333,'slerp')';
            p(:,2) = Quat2AxisAngle(qi);
            quat1 = AxisAngle2Quat(q(:,n-1));
            quat2 = AxisAngle2Quat(q(:,n));
            qi = quatinterp(quat1',quat2',0.333,'slerp')';
            p(:,n+1) = Quat2AxisAngle(qi);
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
            knot_vec = zeros(m+p+1,1);
            for jidx=2:m-p
                idx = floor((jidx-1)*d);
                alph = (jidx-1)*d-idx;
                knot_vec(jidx+p) = (1-alph)*uk(idx)+alph*uk(idx+1);
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
        function [pos, vel, acc] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = [];
            tf = obj.uknot_vec(end);
            uplanner = LspbPlanner([0,tf],2,1,tf);
            for t=0:dt:tf
                [u, du, ddu] = uplanner.GenerateMotion(t);
                [p, v, a] = obj.GenerateMotion(u, du, ddu);
                pos = [pos,p]; vel = [vel,v]; acc = [acc,a];
            end
            if obj.ori_planned
                rpy = AxisAngle2RPY(pos(4:6,:));
                pos = [pos(1:3,:); rpy];
            end
        end
        
        function [p, v, a] = GenerateMotion(obj, u, du, ddu)
            p = obj.GeneratePos(u);
            v = obj.GenerateVel(u, du);
            a = obj.GenerateAcc(u, du, ddu);
%             v = [v; norm(v)]; a = [a; norm(a)];
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
            if obj.ori_planned
                rpy = obj.ctrl_rpy*b_coeff;
                pos = [pos;rpy];
            end
        end

        function vel = GenerateVel(obj, u, du)
            m = obj.num_ctrlp;
            diff_bcoeff = zeros(m,1);
            for idx=1:m
                diff_bcoeff(idx) = obj.DiffBSplineCoeff(3, idx, u, 1);
            end
            % dp/dt=(dp/du)*(du/dt);
            vel = obj.ctrl_pos*diff_bcoeff*du;
            if obj.ori_planned
                vel = [vel;zeros(3,1)];
            end
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
            if obj.ori_planned
                acc = [acc;zeros(3,1)];
            end
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
            uk = obj.uknot_vec;
%             uplanner1 = PolyTrajPlanner(uk(1:2), uk(1:2), [0,1], 5);
%             uplanner2 = PolyTrajPlanner(uk(end-1:end), uk(end-1:end)-uk(end-1), [1,0], 5);
            npts = 3;
%             uplanner1 = PolyTrajPlanner(uk(1:npts), uk(1:npts), [0,1], 3);
%             uplanner2 = PolyTrajPlanner(uk(end-npts+1:end), uk(end-npts+1:end)-uk(end-npts+1), [1,0], 3);
            uplanner = LspbPlanner([uk(1),uk(end)],2,1,uk(end));
            for t=uk(1):dt:uk(end)
%                 if t>=uk(1) && t<=uk(npts)
%                     [u,du,ddu] = uplanner1.GenerateMotion(t);
%                 elseif t>=uk(end-npts+1) && t<=uk(end)
%                     [u,du,ddu] = uplanner2.GenerateMotion(t-uk(end-npts+1));
%                 else
%                     u = t; du = 1; ddu = 0;

%                 end
                [u,du,ddu] = uplanner.GenerateMotion(t);
                [p, v, a] = obj.GenerateMotion(u, du, ddu);
                pos = [pos, p]; vel = [vel, v]; acc = [acc, a];
            end
            t = uk(1):dt:uk(end);
            str_vel = {'vx', 'vy', 'vz', '|v|'};
            str_acc = {'ax', 'ay', 'az', '|a|'};
            figure
            for idx=1:length(str_vel)
                subplot(4,2,2*idx-1); plot(t,vel(idx,:),'k-'); grid on; ylabel(str_vel{idx});
                subplot(4,2,2*idx); plot(t,acc(idx,:),'k-'); grid on; ylabel(str_acc{idx});
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

function axis_angle = RPY2AxisAngle(rpy)
    for idx=1:size(rpy,2)
        rot_mat = rpy2r(180/pi*rpy(:,idx)', 'xyz');
        [theta, k] = tr2angvec(rot_mat);
        axis_angle(:,idx) = theta*k'/2;
    end
end


function rpy = AxisAngle2RPY(axis_angle)
    for idx=1:size(axis_angle,2)
        theta = 2*norm(axis_angle(:,idx));
        k = 2*axis_angle(:,idx)/theta;
        rot_mat = angvec2r(theta, k);
        rpy(:,idx) = tr2rpy(rot_mat, 'xyz')';
    end
end

function quat = AxisAngle2Quat(axis_angle)
    n = size(axis_angle,2);
    quat = zeros(4,n);
    for idx=1:n
        theta = 2*norm(axis_angle(:,idx));
        if abs(theta)<1e-5
            rot_mat = eye(3);
        else
            dir_vec = 2*axis_angle(:,idx)/theta;
            rot_mat = angvec2r(theta, dir_vec);
        end
        q = dcm2quat(rot_mat);
        quat(:,idx) = q';
    end
end


function axis_angle = Quat2AxisAngle(quat)
    n = size(quat,2);
    axis_angle = zeros(3,n);
    for idx=1:n
        half_theta = acos(quat(1,idx));
        dir_vec = quat(2:end,idx)/sin(half_theta);
        axis_angle(:,idx) = half_theta*dir_vec;
    end
end
