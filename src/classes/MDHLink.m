classdef MDHLink < handle
    
    properties
        theta
        d
        a
        alpha
        type
        offset
    
        m
        cog
        inertia
        fv
        fc
        
        pose
    end
    
    methods
        function obj = MDHLink(dh)
            obj.theta = dh(1);
            obj.d = dh(2);
            obj.a = dh(3);
            obj.alpha = dh(4);
            obj.type = dh(5);
            obj.offset = dh(6);

            obj.m = 0;
            obj.cog = zeros(3,1);
            obj.inertia = zeros(3,3);
            obj.fv = 0;
            obj.fc = 0;

            obj.pose = eye(4);            
        end
    
        function Transform(obj, q)
            if obj.type==0
                obj.theta = q+obj.offset;
            else
                obj.d = q+obj.offset;
            end
            st = sin(obj.theta);
            sa = sin(obj.alpha);
            ct = cos(obj.theta);
            ca = cos(obj.alpha);
            rot(1,1) = ct; rot(1,2) = -st; rot(1,3) = 0;
            rot(2,1) = st*ca; rot(2,2) = ct*ca; rot(2,3) = -sa;
            rot(3,1) = st*sa; rot(3,2) = ct*sa; rot(3,3) = ca;
            pos(1) = obj.a; pos(2) = -sa*obj.d; pos(3) = ca*obj.d;
            obj.pose(1:3,1:3) = rot;
            obj.pose(1:3,end) = pos;
        end

    end
    
    
    
end
