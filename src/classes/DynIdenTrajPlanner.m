%   Dynamic Identification Trajectory Class:
%   A specific class that plans and generates robot dynamic identification trajectory
%   Author:
%   liao zhixiang, zhixiangleo@163.com

classdef DynIdenTrajPlanner < handle

    properties
        traj_params_half1hz
        traj_params_1hz
        w0
        order
        njoint
        q_b
        qd_b
        qdd_b
        q0
        
        q_base
        qd_base
        qdd_base
        traj_order_params
        traj_id
    end

    methods
        function obj = DynIdenTrajPlanner(traj_params_half1hz,traj_params_1hz)
            obj.traj_params_half1hz = obj.InitTrajParams(traj_params_half1hz,5);
            obj.traj_params_1hz = obj.InitTrajParams(traj_params_1hz,10);
            obj.w0 = 2*pi*0.1;
            obj.njoint = 6;
            obj.q0 = [];
        end
        
        function traj_params_hz = InitTrajParams(obj,traj_params,order)
            for traj_idx=1:size(traj_params,1)
                params = reshape(traj_params(traj_idx,:),2*order+1,[]);
                traj_params_hz{traj_idx} = params';
            end
            if order==5
                order_idx = 1;
            else
                order_idx = 2;
            end
            obj.q_b{order_idx} = ones(2*order+1,1);
            obj.qd_b{order_idx} = zeros(2*order+1,1);
            obj.qdd_b{order_idx} = zeros(2*order+1,1);
        end
        
        function AddTraj(obj,order,traj_id)
            obj.order = order;
            obj.traj_id = traj_id;
            if order==5
                order_idx = 1;
                obj.traj_order_params = obj.traj_params_half1hz;
            else
                order_idx = 2;
                obj.traj_order_params = obj.traj_params_1hz;
            end
            obj.q_base = obj.q_b{order_idx};
            obj.qd_base = obj.qd_b{order_idx};
            obj.qdd_base = obj.qdd_b{order_idx};
            for idx=traj_id
                [q,~,~] = obj.GenerateMotion(idx,0);
                obj.q0 = [obj.q0,q];
            end
        end
        
        function [jpos,jvel,jacc] = GenerateTraj(obj,dt)
            jpos = []; jvel = []; jacc = [];
            for t=0:dt:10
                [jp,jv,ja] = obj.GenerateMotion(obj.traj_id,t);
                jpos = [jpos,jp]; jvel = [jvel,jv]; jacc = [jacc,ja];
            end
        end
        
        function [jp,jv,ja] = GenerateMotion(obj,traj_idx,t)
            jp = zeros(obj.njoint,1);
            jv = zeros(obj.njoint,1);
            ja = zeros(obj.njoint,1);
            for jidx=1:obj.njoint
                [jp(jidx),jv(jidx),ja(jidx)] = obj.GenerateJointMotion(traj_idx,jidx,t);
            end
        end
        
        function [p,v,a] = GenerateJointMotion(obj,traj_idx,jidx,t)
            for idx=1:obj.order
                tmp = obj.w0*idx;
                sa = sin(tmp*t);
                ca = cos(tmp*t);
                obj.q_base(idx) = sa/tmp;
                obj.q_base(obj.order+idx) = -ca/tmp;
                obj.qd_base(idx) = ca;
                obj.qd_base(obj.order+idx) = sa;
                obj.qdd_base(idx) = -sa*tmp;
                obj.qdd_base(obj.order+idx) = ca*tmp;
            end
            p = obj.traj_order_params{traj_idx}(jidx,:)*obj.q_base;
            v = obj.traj_order_params{traj_idx}(jidx,:)*obj.qd_base;
            a = obj.traj_order_params{traj_idx}(jidx,:)*obj.qdd_base;
        end
    end
    
end