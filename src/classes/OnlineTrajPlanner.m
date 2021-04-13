%   Nonlinear Filters for Optimal Trajectory Planning
%   Online Trajectory Planner Class
%   it contains 2 kinds of type, 3rd or 5th order
%   Referance: 
%   trajectory planning for automatic machines and robots, chapter 4.6
%   Author:
%   liao zhixiang, zhixiangleo@163.com

classdef OnlineTrajPlanner < handle
    properties
        order
        vmax
        amax
        jmax
        ts

        pre_pos
        pre_vel
        pre_acc
        pre_jerk
    end
    
    methods
        function obj = OnlineTrajPlanner(vmax, amax, jmax, sample_time, order)
            obj.vmax = vmax;
            obj.amax = amax;
            obj.jmax = jmax;
            obj.ts = sample_time;
            obj.order = order;
            obj.pre_pos = 0;
            obj.pre_vel = 0;
            obj.pre_acc = 0;
            obj.pre_jerk = 0;
        end

        function InitPlanner(obj, p0, v0, a0)
            obj.pre_pos = p0;
            obj.pre_vel = v0;
            obj.pre_acc = a0;
        end

        function [p, v, a, j] = GenerateMotion5th(obj, avp_cmd, avp_fdb)
            U = obj.jmax;
            % define the error
            ek = (avp_fdb(1)-avp_cmd(1))/U;
            dek = (avp_fdb(2)-avp_cmd(2))/U;
            ddek = (avp_fdb(3)-avp_cmd(3))/U;
            de_min = (-obj.vmax-avp_cmd(2))/U;
            de_max = (obj.vmax - avp_cmd(2))/U;
            dde_min = (-obj.amax-avp_cmd(3))/U;
            dde_max = (obj.amax-avp_cmd(3))/U;
            % calculate the controller paramters
            delta = dek+ddek*abs(ddek)*0.5;
            sig = ek+dek*ddek*sign(delta)-ddek^3/6*(1-3*abs(sign(delta)))...
                    +sign(delta)/4*sqrt(2*(ddek^2+2*dek*sign(delta))^3);
            nu_pos = ek-dde_max*(ddek^2-2*dek)/4-(ddek^2-2*dek)^2/8/dde_max...
                    -ddek*(3*dek-ddek^2)/3;
            nu_neg = ek-dde_min*(ddek^2+2*dek)/4-(ddek^2+2*dek)^2/8/dde_min...
                    +ddek*(3*dek+ddek)/3;
            if ddek<=dde_max && dek<=(ddek^2*0.5-dde_max^2)
                Sigma = nu_pos;
            elseif ddek>=dde_min && dek>=(dde_min^2-ddek^2*0.5)
                Sigma = nu_neg;
            else
                Sigma = sig;
            end
            uc_tmp = Sigma+(1-abs(sign(Sigma)))*(sig+(1-abs(sign(sig)))*ddek);
            uc = -U*sign(uc_tmp);
            uk_value1 = obj.ForcesVelocity(de_min, dde_min, dde_max, ddek, dek);
            uv = obj.ForcesVelocity(de_max, dde_min, dde_max, ddek, dek);
            uk_value2 = min(uc, uv);
            uk = max(uk_value1, uk_value2);

            j = uk;
            a = obj.pre_acc+obj.ts*obj.pre_jerk;
            v = obj.pre_vel+0.5*obj.ts*(a+obj.pre_acc);
            p = obj.pre_pos+0.5*obj.ts*(v+obj.pre_vel);
            obj.pre_jerk = j; obj.pre_acc = a; obj.pre_vel = v; obj.pre_pos = p;
        end

        function res = ForcesVelocity(obj, v, dde_min, dde_max, ddek, dek)
            uv_value1 = obj.ua(dde_min, ddek);
            ucv = obj.ucv(v, dek, ddek);
            ua = obj.ua(dde_max, ddek);
            uv_value2 = min(ucv, ua);
            res = max(uv_value1, uv_value2);
        end

        function res = ua(obj, a, ddek)
            U = obj.jmax;
            res = -U*sign(ddek-a);
        end

        function res = deltav(obj, v, dek, ddek)
            res = ddek*abs(ddek)+2*(dek-v);
        end

        function res = ucv(obj, v, dek, ddek)
            U = obj.jmax;
            tmp_value = (1-abs(sign(obj.deltav(v, dek, ddek))))*ddek;
            res = -U*sign(obj.deltav(v,dek,ddek)+tmp_value);
        end

        
        function [p, v, a] = GenerateMotion3rd(obj, avp_cmd, avp_fdb)
            U = obj.amax;
            err = (avp_fdb(1)-avp_cmd(1))/U;
            derr = (avp_fdb(2)-avp_cmd(2))/U;
            if abs(derr)<1e-5 && abs(err)<1e-5
                uk = 0;
                obj.pre_acc = 0;
            else
                zk = (err/obj.ts+derr*0.5)/obj.ts;
                dzk = derr/obj.ts;
                m = floor((1+sqrt(1+8*abs(zk)))*0.5);
                sigmak = dzk+zk/m+(m-1)*sign(zk)*0.5;
                uk = -U*LimitNumber(-1, sigmak, 1)...
                    *(1+sign(avp_fdb(2)*sign(sigmak)+obj.vmax-obj.ts*U))*0.5;
            end
            a = uk; 
            v = obj.pre_vel+obj.ts*obj.pre_acc;
            p = obj.pre_pos+obj.ts*0.5*(v+obj.pre_vel); 
            obj.pre_acc = a; obj.pre_vel = v; obj.pre_pos = p;
        end


    end

end