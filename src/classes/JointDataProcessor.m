classdef JointDataProcessor < handle
    
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
        fs
        acc_filter
        tau_filter
        acc_filtered
        tau_filtered
    end
    
    
    methods
        function obj = JointDataProcessor(fs,fc_acc,fc_tau,acc_filtered,tau_filtered)
            obj.acc_filtered = acc_filtered;
            obj.tau_filtered = tau_filtered;
            obj.fs = fs;
            obj.order = 1;
            obj.init_diff = false;
            obj.njoint = 6;
            if obj.order==1
                obj.proj_vel = [0,0,0,-1,1];
                obj.proj_acc = [0,0,1,-2,1];
            elseif obj.order==2
                obj.proj_vel = [0,0,1,-4,3]/2;
                obj.proj_acc = [0,-1,4,-5,2];
            elseif obj.order==3
                obj.proj_vel = [1,-8,0,8,-1]/12;
                obj.proj_acc = [-1,16,-30,16,-1]/12;
            end
            for jidx=1:obj.njoint
                obj.acc_filter{jidx} = Biquad('LOWPASS',fc_acc/fs,sqrt(2)/2,0);
                obj.tau_filter{jidx} = Biquad('LOWPASS',fc_tau/fs,sqrt(2)/2,0);
            end
        end
        
        function [p,v,a,t] = ProcessData(obj,qin,dqin,tin)
            p = qin;
            v = dqin;
            t = tin;
            a = zeros(obj.njoint,1);
            if ~obj.init_diff
                obj.init_diff = true;
                for jidx=1:obj.njoint
                    obj.pre4(jidx) = dqin(jidx);
                    obj.pre3(jidx) = dqin(jidx);
                    obj.pre2(jidx) = dqin(jidx);
                    obj.pre1(jidx) = dqin(jidx);
                    obj.pre0(jidx) = dqin(jidx);
                end
            end
            
            for jidx=1:obj.njoint
                obj.pre4(jidx) = obj.pre3(jidx);
                obj.pre3(jidx) = obj.pre2(jidx);
                obj.pre2(jidx) = obj.pre1(jidx);
                obj.pre1(jidx) = obj.pre0(jidx);
                obj.pre0(jidx) = dqin(jidx);
                a(jidx) = (obj.proj_vel*[obj.pre4(jidx);obj.pre3(jidx);obj.pre2(jidx);obj.pre1(jidx);obj.pre0(jidx)])*obj.fs;
                if obj.acc_filtered
                    a(jidx) = obj.acc_filter{jidx}.Filter(a(jidx));
                end
                if obj.tau_filtered
                    t(jidx) = obj.tau_filter{jidx}.Filter(tin(jidx));
                end
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
        
    end
    
end
 
