function rpy = Rot2RPY(rot)
    % X->Y->Z rotation about the fixed axis

    bet = atan2(-rot(3,1), sqrt(rot(1,1)^2+rot(2,1)^2));
    if abs(bet-pi/2)<1e-5
        alph = 0;
        gamm = atan2(rot(1,2), rot(2,2));
    elseif abs(bet+pi/2)<1e-5
        alph = 0;
        gamm = -atan2(rot(1,2), rot(2,2));
    else
        alph = atan2(rot(2,1)/cos(bet), rot(1,1)/cos(bet));
        gamm = atan2(rot(3,2)/cos(bet), rot(3,3)/cos(bet));
    end
    rpy = [gamm, bet, alph];
end