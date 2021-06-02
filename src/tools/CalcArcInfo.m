function [center,radius,rot]=CalcArcInfo(pos1,pos2,pos3)

    p2p1 = pos1-pos2; p2p3 = pos3-pos2;
    inc_angle = acos(dot(p2p1,p2p3)/norm(p2p1)/norm(p2p3));
%     theta = pi-inc_angle;
    radius = norm(p2p1)*tan(0.5*inc_angle);
    pc = 0.5*(pos1+pos3); p2pc = pc-pos2;
    scale = radius/sin(0.5*inc_angle)/norm(p2pc);
    p2center = scale*p2pc;
    center = pos2+p2center;

    n = (pos1-center)/norm(pos1-center);
    [a1, b1, c1, ~] = CalcPlaneParams(pos1, pos2, pos3);
    a = [a1; b1; c1]/norm([a1; b1; c1]);
    o = cross(a,n);
    rot = [n, o, a];

end