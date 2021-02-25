function q = IKSolve(frame, option, alpha)

    q = zeros(1,5);
    height_limit = [0.5, 1.5];
    [~, pos] = tr2rt(frame);
    if strcmp(option, 'vertical')
        if pos(3)>height_limit(2)
            q(2) = height_limit(2);
            q(1) = -atan(pos(1)/pos(2));
            q(3) = -atan(sin(q(1))*(pos(3)-q(2))/pos(1));
            q(4) = (pos(3)-q(2))/sin(q(3));
            q(5) = -q(1);
        elseif pos(3)<height_limit(1)
            q(2) = height_limit(1);
            q(1) = -atan(pos(1)/pos(2));
            q(3) = -atan(sin(q(1))*(pos(3)-q(2))/pos(1));
            q(4) = (pos(3)-q(2))/sin(q(3));
            q(5) = -q(1);
        else
            q(1) = -atan(pos(1)/pos(2));
            q(2) = pos(3);
            q(3) = 0;
            q(4) = -pos(1)/sin(q(1));
            q(5) = -q(1);
        end        
    elseif strcmp(option, 'horizontal')
        q(3) = -pi/6;
        q(1) = -atan(pos(1)/pos(2));
        q(4) = pos(2)/(cos(q(1)*cos(q(3))));
        q(2) = pos(3)-sin(q(3))*q(4);
        q(5) = -q(1);
    elseif strcmp(option, 'washbasin')
        q(3) = -pi/6;
        q(1) = -atan(pos(1)/pos(2));
        q(4) = pos(2)/(cos(q(1)*cos(q(3))));
        q(2) = pos(3)-sin(q(3))*q(4);
        q(5) = 0-q(1);
    else
        error('IKSolver dose not work');
    end


end

