function dir = SignVel(vel)

    thre_val = 0.01;
    if abs(vel)<=thre_val
        dir = 0;
    else
        dir = sign(vel);
    end

end