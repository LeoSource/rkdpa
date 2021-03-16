classdef CollisionDetector < handle
    
    properties
        type
        tol
    end
    
    methods
        function obj = CollisionDetector(type)
            obj.type = type;%to do: add others bounding box 
            obj.tol = 1e-3;
        end
        
        function result = CheckBoxs(obj, box1, box2)
            result = (obj.AxisIntersect(box1.axis_x, box2.axis_x)>obj.tol) &&...
                        (obj.AxisIntersect(box1.axis_y, box2.axis_y)>obj.tol) &&...
                        (obj.AxisIntersect(box1.axis_z, box2.axis_z)>obj.tol);
        end
        
        function res = CheckLineBox(obj, pointA, pointB, box)
            dir_line = pointB-pointA;
            tx(1) = (box.axis_x(1)-pointA(1))/dir_line(1);
            tx(2) = (box.axis_x(2)-pointA(1))/dir_line(1);
            ty(1) = (box.axis_y(1)-pointA(1))/dir_line(2);
            ty(2) = (box.axis_y(2)-pointA(1))/dir_line(2);
            tz(1) = (box.axis_z(1)-pointA(1))/dir_line(3);
            tz(2) = (box.axis_z(2)-pointA(1))/dir_line(3);
            if max([tx(1), ty(1), tz(1)])>min([tx(2), ty(2), tz(2)]) && min([tx(2), ty(2), tz(2)])>0
                res = 1;
            else
                res = 0;
            end
        end
        
        function range = AxisOverlap(obj, axis1, axis2)
            overlap_max = min([axis1(2), axis2(2)]);
            overlap_min = max([axis1(1), axis2(1)]);
            if overlap_max<overlap_min
                range = 0;
            else
                range = overlap_max-overlap_min;
            end
        end
        

    end
    
end