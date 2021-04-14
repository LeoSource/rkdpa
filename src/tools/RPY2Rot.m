function rot = RPY2Rot(gamm, bet, alph)
    % X->Y->Z rotation about the fixed axis
    % rot = Rz(alph)*Ry(bet)*Rx(gamm);
    ca = cos(alph); sa = sin(alph);
    cb = cos(bet); sb = sin(bet);
    cg = cos(gamm); sg = sin(gamm);

    rot = eye(3);
    rot(1,1) = ca*cb;
    rot(1,2) = ca*sb*sg-sa*cg;
    rot(1,3) = ca*sb*cg+sa*sg;
    rot(2,1) = sa*cb;
    rot(2,2) = sa*sb*sg+ca*cg;
    rot(2,3) = sa*sb*cg-ca*sg;
    rot(3,1) = -sb;
    rot(3,2) = cb*sg;
    rot(3,3) = cb*cg;

end