function [center ,radius, theta, trans] = CalcArc(pos1, pos2, pos3)
% calculate the arc information according to 3 points
% arc information: centor, radius, angle range

    [a1, b1, c1, d1] = PointsCoplane(pos1, pos2, pos3);
    [a2, b2, c2, d2] = RadiusEqual(pos1, pos2);
    [a3, b3, c3, d3] = RadiusEqual(pos1, pos3);

    center(1) = -(b1*c2*d3-b1*c3*d2-b2*c1*d3+b2*c3*d1+b3*c1*d2-b3*c2*d1)/...
                        (a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
    center(2) = (a1*c2*d3-a1*c3*d2-a2*c1*d3+a2*c3*d1+a3*c1*d2-a3*c2*d1)/...
                        (a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
    center(3) = -(a1*b2*d3-a1*b3*d2-a2*b1*d3+a2*b3*d1+a3*b1*d2-a3*b2*d1)/...
                        (a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
    tmp_value = (pos1(1)-center(1))^2+(pos1(2)-center(2))^2+(pos1(3)-center(3))^2;
    radius = sqrt(tmp_value);
    line_length = norm(pos3-pos1);
    tmp_cos = (radius^2+radius^2-line_length^2)/(2*radius*radius);
    theta = acos(tmp_cos);
end

function [a, b, c, d] = RadiusEqual(pos1, pos2)
    a = 2*(pos2(1)-pos1(1));
    b = 2*(pos2(2)-pos1(2));
    c = 2*(pos2(3)-pos1(3));
    d = pos1(1)^2+pos1(2)^2+pos1(3)^2-...
        pos2(1)^2-pos2(2)^2-pos2(3)^2;
end

function [a, b, c, d] = PointsCoplane(pos1, pos2, pos3)
    x1 = pos1(1); y1 = pos1(2); z1 = pos1(3);
    x2 = pos2(1); y2 = pos2(2); z2 = pos2(3);
    x3 = pos3(1); y3 = pos2(2); z3 = pos3(3);
    a = y1*z2-y2*z1-y1*z3+y3*z1+y2*z3-y3*z2;
    b = -(x1*z2-x2*z1-x1*z3+x3*z1+x2*z3-x3*z2);
    c = x1*y2-x2*y1-x1*y3+x3*y1+x2*y3-x3*y2;
    d = -(x1*y2*z3-x1*y3*z2-x2*y1*z3+x2*y3*z1+x3*y1*z2-x3*y2*z1);
end
