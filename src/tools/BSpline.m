function [pos, b_coeff] = BSpline(via_pos, knots_vec, u)
% 3 degree by default
% therefore, there must be 4 or more via positions
    nump = size(via_pos,2);
    if nump<=3
        error('input the appropriate number of via position');
    else
        for idx=1:nump
            b_coeff(idx,1) = BaseFunc(knots_vec, 3, idx, u);
        end
    end
    pos = via_pos*b_coeff;

end


function b_base = BaseFunc(knots_vec, degree, idx, u)
    if degree==0
        if u>=knots_vec(idx) &&u<knots_vec(idx+1)
            b_base = 1;
        else
            b_base = 0;
        end
    else
        den1 = (knots_vec(idx+degree)-knots_vec(idx));
        num1 = (u-knots_vec(idx));
        if abs(den1)<eps && abs(num1)<eps
            coef1 = 0;
        elseif abs(den1)<eps
            den1 = 1;
            coef1 = num1/den1;
        else
            coef1 = num1/den1;
        end

        den2 = (knots_vec(idx+degree+1)-knots_vec(idx+1));
        num2 = (knots_vec(idx+degree+1)-u);
        if abs(den2)<eps && abs(num2)<eps
            coef2 = 0;
        elseif abs(den2)<eps
            den2 = 1;
            coef2 = num2/den2;
        else
            coef2 = num2/den2;
        end
        b_base = coef1*BaseFunc(knots_vec,degree-1,idx,u)+...
                coef2*BaseFunc(knots_vec,degree-1,idx+1,u);
    end

end