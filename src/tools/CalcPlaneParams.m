function [a,b,c,d] = CalcPlaneParams(pos1,pos2,pos3)

    x1 = pos1(1); y1 = pos1(2); z1 = pos1(3);
    x2 = pos2(1); y2 = pos2(2); z2 = pos2(3);
    x3 = pos3(1); y3 = pos3(2); z3 = pos3(3);
    a = y1*z2-y2*z1-y1*z3+y3*z1+y2*z3-y3*z2;
    b = -(x1*z2-x2*z1-x1*z3+x3*z1+x2*z3-x3*z2);
    c = x1*y2-x2*y1-x1*y3+x3*y1+x2*y3-x3*y2;
    d = -(x1*y2*z3-x1*y3*z2-x2*y1*z3+x2*y3*z1+x3*y1*z2-x3*y2*z1);

end