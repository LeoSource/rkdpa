classdef JointDifferentiator < handle
    
    properties
        njoint
        order
        proj_vel
        proj_acc
        init_diff
        
        pre4
        pre3
        pre2
        pre1
        pre0
        dt
        acc_filter
    end
    
    
    methods
        function obj = JointDifferentiator(order,dt)
            obj.order = order;
            obj.init_diff = false;
            obj.dt = dt;
            if order==1
                obj.proj_vel = [0,0,0,-1,1];
                obj.proj_acc = [0,0,1,-2,1];
            elseif order==2
                obj.proj_vel = [0,0,1,-4,3]/2;
                obj.proj_acc = [0,-1,4,-5,2];
            elseif order==3
                obj.proj_vel = [1,-8,0,8,-1]/12;
                obj.proj_acc = [-1,16,-30,16,-1]/12;
            end
        end
        
        function [v,a] = ProcessPosition(obj,qin)
            obj.njoint = length(qin);
            v = zeros(obj.njoint,1);
            a = zeros(obj.njoint,1);
            if ~obj.init_diff
                obj.init_diff = true;
                for jidx=1:obj.njoint
                    obj.pre4(jidx) = qin(jidx);
                    obj.pre3(jidx) = qin(jidx);
                    obj.pre2(jidx) = qin(jidx);
                    obj.pre1(jidx) = qin(jidx);
                    obj.pre0(jidx) = qin(jidx);
                    obj.acc_filter{jidx} = Biquad('LOWPASS', 2/(1/obj.dt), sqrt(2)/2, 0);
                end
            end
            
            for jidx=1:obj.njoint
                obj.pre4(jidx) = obj.pre3(jidx);
                obj.pre3(jidx) = obj.pre2(jidx);
                obj.pre2(jidx) = obj.pre1(jidx);
                obj.pre1(jidx) = obj.pre0(jidx);
                obj.pre0(jidx) = qin(jidx);
                vel = (obj.proj_vel*[obj.pre4(jidx);obj.pre3(jidx);obj.pre2(jidx);obj.pre1(jidx);obj.pre0(jidx)])/obj.dt;
                acc = (obj.proj_acc*[obj.pre4(jidx);obj.pre3(jidx);obj.pre2(jidx);obj.pre1(jidx);obj.pre0(jidx)])/obj.dt/obj.dt;
                v(jidx) = vel;
                a(jidx) = obj.acc_filter{jidx}.Filter(acc);
            end
            
        end
        
        function a = ProcessVelocity(obj,vel)
            obj.njoint = length(vel);
            a = zeros(obj.njoint,1);
            if ~obj.init_diff
                obj.init_diff = true;
                for jidx=1:obj.njoint
                    obj.pre4(jidx) = vel(jidx);
                    obj.pre3(jidx) = vel(jidx);
                    obj.pre2(jidx) = vel(jidx);
                    obj.pre1(jidx) = vel(jidx);
                    obj.pre0(jidx) = vel(jidx);
                    obj.acc_filter{jidx} = Biquad('LOWPASS', 2/(1/obj.dt), sqrt(2)/2, 0);
                end
            end
            
            for jidx=1:obj.njoint
                obj.pre4(jidx) = obj.pre3(jidx);
                obj.pre3(jidx) = obj.pre2(jidx);
                obj.pre2(jidx) = obj.pre1(jidx);
                obj.pre1(jidx) = obj.pre0(jidx);
                obj.pre0(jidx) = vel(jidx);
                acc = (obj.proj_vel*[obj.pre4(jidx);obj.pre3(jidx);obj.pre2(jidx);obj.pre1(jidx);obj.pre0(jidx)])/obj.dt;
                a(jidx) = obj.acc_filter{jidx}.Filter(acc);
            end
            
        end
        
    end
    
end
 
