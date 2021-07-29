function jaco = RPY2JAC(rpy)
    gamm = rpy(1); bet = rpy(2); alph = rpy(3);
    jaco = zeros(3,3);
    jaco(3,3) = 1;
    jaco(1,1) = cos(alph)*cos(bet);
    jaco(1,2) = -sin(alph);
    jaco(2,1) = sin(alph)*cos(bet);
    jaco(2,2) = cos(alph);
    jaco(3,1) = -sin(bet);
end