classdef CubicBSplinePlanner < handle

    properties
        nump
        knot_vec
        ctrl_pos

    end

    methods
        function obj = CubicBSplinePlanner(via_pos)
            obj.nump = size(via_pos, 2);
            obj.knot_vec = obj.CalcKnotVec(via_pos);
            obj.ctrl_pos = obj.CalcCtrlPos(via_pos);
        end

        function knot_vec = CalcKnotVec(obj, q)
            uk = zeros(obj.nump, 1);
            uk(end) = 1;
            d = 0;
            for idx=2:obj.nump
                d = d+norm(q(:,idx)-q(:,idx-1));
            end
            for idx=2:obj.nump-1
                uk(idx) = uk(idx-1)+norm(q(:,idx)-q(:,idx-1))/d;
            end
            n = obj.nump+6;
            knot_vec = zeros(1, n);
            [knot_vec(1), knot_vec(2), knot_vec(3)] = deal(uk(1));
            [knot_vec(obj.nump+4), knot_vec(obj.nump+5), knot_vec(obj.nump+6)] = deal(uk(end));
            for idx=1:obj.nump
                knot_vec(idx+3) = uk(idx);
            end
        end

        function p = CalcCtrlPos(obj, q)
            n = obj.nump; u_hat = obj.knot_vec(4:n+3);
            dim = size(q, 1);
            p = zeros(dim, n+2);
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
                if abs(den1)<eps && abs(num1)<eps
                    coef1 = 0;
                elseif abs(den1)<eps
                    den1 = 1;
                    coef1 = num1/den1;
                else
                    coef1 = num1/den1;
                end
                den2 = obj.knot_vec(idx+p+1)-obj.knot_vec(idx+1);
                num2 = obj.knot_vec(idx+p+1)-u;
                if abs(den2)<eps && abs(num2)<eps
                    coef2 = 0;
                elseif abs(den2)<eps
                    den2 = 1;
                    coef2 = num2/den2;
                else
                    coef2 = num2/den2;
                end
                b_coeff = coef1*obj.CalcBSplineCoeff(p-1, idx, u)...
                            +coef2*obj.CalcBSplineCoeff(p-1, idx+1, u);
            end
        end

        function pos = GeneratePos(obj, u)
            n = obj.nump;
            b_coeff = zeros(n+2, 1);
            if abs(u-max(obj.knot_vec))<eps
                b_coeff(end) = 1;
            else
                for idx=1:n+2
                    b_coeff(idx) = obj.CalcBSplineCoeff(3, idx, u);
                end
            end
            pos = obj.ctrl_pos*b_coeff;
        end

        function curve = GenerateBSpline(obj, du)
            curve = [];
            for u=0:du:1
                p = obj.GeneratePos(u);
                curve = [curve, p];
            end
        end

        function PlotBSpline(obj, du)
            curve = obj.GenerateBSpline(du);
            plot2(curve', 'k-'); grid on;
            xlabel('x'); ylabel('y'); zlabel('z');
        end

    end


end