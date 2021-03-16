classdef AABBBox < handle
    
    properties
        min_pos
        max_pos
        center
        axis_x
        axis_y
        axis_z
    end
    
    methods
        function obj = AABBBox(min_pos, max_pos)
            obj.min_pos = min_pos;
            obj.max_pos = max_pos;
            obj.center = 0.5*(min_pos+max_pos);
            obj.axis_x = [min_pos(1), max_pos(2)];
            obj.axis_y = [min_pos(2), max_pos(2)];
            obj.axis_z = [min_pos(3), max_pos(3)];
        end
        
        function Move(obj, pos)
            obj.min_pos = obj.min_pos+pos;
            obj.max_pos = obj.max_pos+pos;
        end
        
        function box = Copy(obj, pos)
            box = AABBBox(zeros(1,3), zeros(1,3));
            box.min_pos = obj.min_pos+pos;
            box.max_pos = obj.max_pos+pos;
        end
        
        function res = Contain(obj, pos)
            res = pos(1)>obj.min_pos(1) && pos(1)<obj.max_pos(1) ...
                && pos(2)>obj.min_pos(2) && pos(2)<obj.max_pos(2)...
                && pos(3)>obj.min_pos(3) && pos(3)<obj.max_pos(3);
        end
        
    end
    
        
    
end