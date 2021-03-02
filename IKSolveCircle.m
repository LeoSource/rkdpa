function q = IKSolveCircle(cart_pos, option, alpha)

    h = 0.6; r = 0.3; ty = 0.2;
    switch option
        case 'horizontal'
            q(1) = atan(sin(alpha)*(r-0.2)/(h+cos(alpha)*(r-0.2)));
            q(2) = cart_pos(3);
            q(3) = 0;
            q(4) = sqrt((sin(alpha)*(r-0.2))^2+(h+(r-0.2)*cos(alpha))^2);
            q(5) = alpha - q(1);
        case 'slope'
            q(1) = atan(sin(alpha)*(r-ty)/(h+cos(alpha)*(r-ty)));
            q(3) = -pi/6;
            q(5) = alpha-q(1);
            tmp_value = h+r*cos(alpha)+ty*sin(q(1))*sin(q(5));
            q(4) = tmp_value/(cos(q(1))*cos(q(3)))-ty*cos(q(5));
            q(2) = cart_pos(3)-sin(q(3))*(q(4)+ty*cos(q(5)));
        otherwise
            error('IKSolver can not slove the circle trajectory');
    end
    

end