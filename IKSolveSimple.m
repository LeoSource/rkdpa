function q = IKSolveSimple(pos, option, alpha)

    %%to simplify the problem of end-effector's pose: theta5 = -theta1
    q = zeros(1,5);
    height_limit = [0.5, 1.5];
    ty = 0.2; tz = 0;
    switch option
        case 'vertical'
            if pos(3)>height_limit(2)
                q(2) = height_limit(2);
            elseif pos(3)<height_limit(1)
                q(2) = height_limit(1);
            else
                q(2) = pos(3);
            end        
            q(1) = atan(pos(1)/(ty-pos(2)));
            a = sin(q(1))*(pos(3)-q(2));
            b = pos(1)-sin(q(1))*cos(q(1))*ty;
            c = sin(q(1))*tz;
            if abs(a+c)<eps
                q(3) = 0;
            else
                q3_tmp1 = 2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));
                q3_tmp2 = 2*atan((b-sqrt(b^2+a^2-c^2))/(a+c));
                if (q3_tmp1>pi/2) || (q3_tmp1<-pi/2)
                    q(3) = q3_tmp2;
                else
                    q(3) = q3_tmp1;
                end
            end
            tmp_value = pos(2)-ty*((sin(q(1)))^2+(cos(q(1)))^2*cos(q(3)))+cos(q(1))*cos(q(3))*tz;
            q(4) = tmp_value/(cos(q(1))*cos(q(3)));
            q(5) = -q(1);
            
        case 'horizontal'
            q(3) = -pi/6;
            q(1) = atan(pos(1)/(ty-pos(2)));
            tmp_value = pos(2)-ty*((sin(q(1)))^2+(cos(q(1)))^2*cos(q(3)))+cos(q(1))*cos(q(3))*tz;
            q(4) = tmp_value/(cos(q(1))*cos(q(3)));
            q(2) = pos(3)-cos(q(1))*sin(q(3))*ty-cos(q(3))*tz-sin(q(3))*q(4);
            q(5) = -q(1);            
        
        case 'circle'
            circle_option = 'slope';
            q = IKSolveCircle(pos, circle_option, alpha);
            
        otherwise
            error('IKSolver dose not work');
    end

end
