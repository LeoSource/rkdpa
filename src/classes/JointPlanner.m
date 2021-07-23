classdef JointPlanner < handle
    
    properties
        q0
        qf
        vmax
        amax
        tf
        uplanner
        nj
        plan_idx
        unplan_idx

        sync
    end

    methods
        function obj = JointPlanner(q0,qf,vmax,amax,sync)
            obj.q0 = q0;
            obj.qf = qf;
            obj.vmax = vmax;
            obj.amax = amax;
            obj.sync = sync;
            obj.nj = length(q0);
            obj.plan_idx = []; obj.unplan_idx = [];
            for idx=1:obj.nj
                if(abs(q0(idx)-qf(idx))>1e-5)
                    obj.plan_idx = [obj.plan_idx,idx];
                else
                    obj.unplan_idx = [obj.unplan_idx,idx];
                end
            end
            tf_tmp = [];
            for idx=obj.plan_idx
                obj.uplanner{idx} = LspbPlanner([q0(idx),qf(idx)],...
                                    vmax(idx),amax(idx));
                tf_tmp = [tf_tmp,obj.uplanner{idx}.tf];
            end
            obj.tf = max(tf_tmp);
            if(sync)
                for idx=obj.plan_idx
                    obj.uplanner{idx} = LspbPlanner([q0(idx),qf(idx)],...
                                        vmax(idx),amax(idx),obj.tf);
                end
            end
        end


        function [p,v,a] = GenerateMotion(obj,t)
            p = zeros(obj.nj,1); v = zeros(obj.nj,1); a = zeros(obj.nj,1);
            if ~isempty(obj.plan_idx)
                for idx=obj.plan_idx
                    t_limit(idx) = LimitNumber(0,t,obj.uplanner{idx}.tf);
                    [p(idx),v(idx),a(idx)] = obj.uplanner{idx}.GenerateMotion(t_limit(idx));
                end
            end
            if ~isempty(obj.unplan_idx)
                for idx=obj.unplan_idx
                    p(idx) = obj.q0(idx);
                end
            end
        end

        function [pos,vel,acc] = GenerateTraj(obj,dt)
            pos = []; vel = []; acc = [];
            for t=0:dt:obj.tf
                [p,v,a] = obj.GenerateMotion(t);
                pos = [pos,p]; vel = [vel,v]; acc = [acc, a];
            end
        end

    end




end