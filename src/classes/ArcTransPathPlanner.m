classdef ArcTransPathPlanner < handle
% plan a continuous path according to the given points
% arc transits between 2 lines
% ref: https://blog.csdn.net/qq_26565435/article/details/98789361
    
    properties
        center
        radius
        theta
        rot
        dr
        dl
        pt
    
        nump
        numarc
    end
    
    
    methods
        function obj = ArcTransPathPlanner(pos, radius)
            obj.radius = radius;
            obj.nump = size(pos, 2);
            obj.numarc = obj.nump-2;
            obj.CalcArcInfo(pos);
        end
        
        
        function CalcArcInfo(obj, pos)
            for idx=1:obj.numarc
                [obj.center(:,idx), obj.theta(idx), pt1, pt2] = obj.CalcArcPoints(pos(:,idx), pos(:,idx+1), pos(:,idx+2));
                obj.pt(:,2*idx-1) = pt1;
                obj.pt(:,2*idx) = pt2;
                obj.dr(idx) = obj.radius*(pi-obj.theta(idx));
                obj.dl(idx) = norm(pt1-pos(:,idx));
                if idx ~=1
                    obj.dl(idx) = obj.dl(idx)-norm(pos(:,idx)-obj.pt(:,2*(idx-1)));
                end
                if idx==obj.numarc
                    obj.dl(idx+1) = norm(pos(:,end)-pt2);
                end
            end
        end
        
        
        function [center, theta, pt1, pt2] = CalcArcPoints(obj, p1, p2, p3)
            p21 = p1-p2; p23 = p3-p2;
            tmp_cos = dot(p21, p23)/norm(p21)/norm(p23);
            theta = acos(tmp_cos);
            
            p2t1 = obj.radius/tan(0.5*theta)*p21/norm(p21);
            pt1 = p2+p2t1;
            p2t2 = obj.radius/tan(0.5*theta)*p23/norm(p23);
            pt2 = p2+p2t2;
            
            pt1t2 = pt2-pt1;
            pt1m1 = 0.5*pt1t2;
            p2m1 = p2t1+pt1m1;
            p2c = obj.radius/sin(0.5*theta)/norm(p2m1)*p2m1;
            center = p2+p2c;            
        end
        
    end
      
end