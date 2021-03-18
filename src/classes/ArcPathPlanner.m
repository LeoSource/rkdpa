classdef ArcPathPlanner < handle
% calculate the arc information according to 3 points
% arc information: center, radius, angle range(include direction),
% and transformation matrix
    properties
        center
        radius
        theta
        rot
    end
    
    
    methods
        function obj = ArcPathPlanner(pos1, pos2, pos3)
            [a1, b1, c1, d1] = obj.PointsCoplane(pos1, pos2, pos3);
            [a2, b2, c2, d2] = obj.RadiusEqual(pos1, pos2);
            [a3, b3, c3, d3] = obj.RadiusEqual(pos1, pos3);
            center(1) = -(b1*c2*d3-b1*c3*d2-b2*c1*d3+b2*c3*d1+b3*c1*d2-b3*c2*d1)/...
                                (a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
            center(2) = (a1*c2*d3-a1*c3*d2-a2*c1*d3+a2*c3*d1+a3*c1*d2-a3*c2*d1)/...
                                (a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
            center(3) = -(a1*b2*d3-a1*b3*d2-a2*b1*d3+a2*b3*d1+a3*b1*d2-a3*b2*d1)/...
                                (a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
            obj.center = reshape(center, 3,1);
            tmp_value = (pos1(1)-center(1))^2+(pos1(2)-center(2))^2+(pos1(3)-center(3))^2;
            obj.radius = sqrt(tmp_value);
            line_length = norm(pos3-pos1);
            tmp_cos = (obj.radius^2+obj.radius^2-line_length^2)/(2*obj.radius*obj.radius);
            obj.theta = acos(tmp_cos);
            
            n = (pos1-center)/norm(pos1-center);
            n = reshape(n, 3,1);
            a = [a1; b1; c1]/norm([a1; b1; c1]);
            o = cross(a,n);
            obj.rot = [n, o, a];
        end
    
        function [a, b, c, d] = RadiusEqual(obj, pos1, pos2)
            a = 2*(pos2(1)-pos1(1));
            b = 2*(pos2(2)-pos1(2));
            c = 2*(pos2(3)-pos1(3));
            d = pos1(1)^2+pos1(2)^2+pos1(3)^2-...
                pos2(1)^2-pos2(2)^2-pos2(3)^2;
        end
    
        function [a, b, c, d] = PointsCoplane(obj, pos1, pos2, pos3)
            x1 = pos1(1); y1 = pos1(2); z1 = pos1(3);
            x2 = pos2(1); y2 = pos2(2); z2 = pos2(3);
            x3 = pos3(1); y3 = pos3(2); z3 = pos3(3);
            a = y1*z2-y2*z1-y1*z3+y3*z1+y2*z3-y3*z2;
            b = -(x1*z2-x2*z1-x1*z3+x3*z1+x2*z3-x3*z2);
            c = x1*y2-x2*y1-x1*y3+x3*y1+x2*y3-x3*y2;
            d = -(x1*y2*z3-x1*y3*z2-x2*y1*z3+x2*y3*z1+x3*y1*z2-x3*y2*z1);
        end    
    
        function p = GeneratePath(obj, varp)
            pc = zeros(3,1);
            pc(1) = obj.radius*cos(varp);
            pc(2) = obj.radius*sin(varp);    
            p = obj.center+obj.rot*pc;    
        end
    
        function [p, v, a] = GenerateMotion(obj, varp, varv, vara)
            p = obj.GeneratePath(varp);
            vc = zeros(3,1); ac = zeros(3,1);
            vc(1) = -varv*sin(varp);
            vc(2) = varv*cos(varp);
            v = obj.rot*vc;
            ac(1) = -varv^2*cos(varp)-vara*sin(varp);
            ac(2) = -varv^2*sin(varp)+vara*cos(varp);
            a = obj.rot*ac;
        end
    
        function [pos, vel, acc] = GenerateTraj(obj, varp, varv, vara)
            pos = []; vel = []; acc = [];
            for idx=1:length(varp)
                [p, v, a] = obj.GenerateMotion(varp(idx), varv(idx), vara(idx));
                pos = [pos, p];
                vel = [vel, v];
                acc = [acc, a];
            end
        end
        
        function PlotTraj(obj, varp, varv, vara, tf, dt)
            [pos, vel, ~] = obj.GenerateTraj(varp, varv, vara);
            time=0:dt:tf;
            figure
            subplot(3,1,1)
            plot(time, pos(1,:));
            ylabel('p\_x');
            subplot(3,1,2)
            plot(time, pos(2,:));
            ylabel('p\_y');
            subplot(3,1,3)
            plot(time, pos(3,:));
            ylabel('p\_z');

            figure
            subplot(3,1,1)
            plot(time, vel(1,:));
            ylabel('v\_x');
            subplot(3,1,2)
            plot(time, vel(2,:));
            ylabel('v\_y');
            subplot(3,1,3)
            plot(time,vel(3,:));
            ylabel('v\_z');        
        end
    
    end
     
end