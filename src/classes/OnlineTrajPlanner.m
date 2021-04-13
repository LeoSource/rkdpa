classdef OnlineTrajPlanner < handle
    properties
        order
        vmax
        amax
        ts

        pre_pos
        pre_vel
        pre_acc
    end
    
    methods
        function obj = OnlineTrajPlanner(vmax, amax, sample_time, order)
            obj.vmax = vmax;
            obj.amax = amax;
            obj.ts = sample_time;
            obj.order = order;
            obj.pre_pos = 0;
            obj.pre_vel = 0;
            obj.pre_acc = 0;
        end

        function InitPlanner(obj, p0, v0, a0)
            obj.pre_pos = p0;
            obj.pre_vel = v0;
            obj.pre_acc = a0;
        end
        

        function [p, v, a] = GenerateMotion(obj, avp_cmd, avp_fdb)
            U = obj.amax;
            err = (avp_fdb(1)-avp_cmd(1))/U;
            derr = (avp_fdb(2)-avp_cmd(2))/U;

            zk = (err/obj.ts+derr*0.5)/obj.ts;
            dzk = derr/obj.ts;
            m = floor((1+sqrt(1+8*abs(zk)))*0.5);
            sigmak = dzk+zk/m+(m-1)*sign(zk)*0.5;
            uk = -U*LimitNumber(-1, sigmak, 1)...
                *(1+sign(avp_fdb(2)*sign(sigmak)+obj.vmax-obj.ts*U))*0.5;
            a = uk; 
            v = obj.pre_vel+obj.ts*obj.pre_acc;
            p = obj.pre_pos+obj.ts*0.5*(v+obj.pre_vel); 
            obj.pre_acc = a; obj.pre_vel = v; obj.pre_pos = p;
        end
    end

end