% calculate the transcendental  equation like
% a*cos(theta)+b*sin(theta) = c
function theta = CalcTransEqua(a, b, c, pre_value)
    if b^2+a^2-c^2<0
        theta = pre_value;
    else
        if abs(a+c)<1e-5
            u = (c-a)/(2*b);
            theta = 2*atan(u);
        else
            tmp_value1 = 2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));
            tmp_value2 = 2*atan((b-sqrt(b^2+a^2-c^2))/(a+c));
            if (tmp_value1>pi/2) || (tmp_value1<-pi/2)
                theta = tmp_value2;
            else
                theta = tmp_value1;
            end
        end
    end

end

