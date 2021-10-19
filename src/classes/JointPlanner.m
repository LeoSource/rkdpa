classdef JointPlanner < handle
    
    properties
        jpos
        tf
        nj
        ntraj
        segplanner
        plan_idx
        unplan_idx

        sync
    end

    methods
        function obj = JointPlanner(q0,sync)
            obj.jpos = q0;
            obj.sync = sync;
            obj.nj = length(q0);
            obj.plan_idx = [];
            obj.unplan_idx = [];
        end

        function AddJntPos(obj,q,jvmax,jamax)
            obj.jpos = [obj.jpos, q];
            obj.ntraj = size(obj.jpos,2)-1;
            for traj_idx=1:obj.ntraj
                q0 = obj.jpos(:,traj_idx);
                qf = obj.jpos(:,traj_idx+1);
                planidx = []; unplanidx = [];
                for idx=1:obj.nj
                    if(abs(q0(idx)-qf(idx))>1e-5)
                        planidx = [planidx,idx];
                    else
                        unplanidx = [unplanidx,idx];
                    end
                end
                tf_tmp = [];
                for idx=planidx
                    uplanner{idx} = LspbPlanner([q0(idx),qf(idx)],...
                                            jvmax(idx),jamax(idx));
                    tf_tmp = [tf_tmp,uplanner{idx}.tf];
                end
                obj.tf(traj_idx) = max(tf_tmp);
                if(obj.sync)
                    for idx=planidx
                        uplanner{idx} = LspbPlanner([q0(idx),qf(idx)],...
                                                jvmax(idx),jamax(idx),obj.tf(traj_idx));
                    end
                end
                obj.segplanner{traj_idx} = uplanner;
                obj.plan_idx{traj_idx} = planidx;
                obj.unplan_idx{traj_idx} = unplanidx;
            end
        end


        function [p,v,a] = GenerateMotion(obj,traj_idx,t)
            p = zeros(obj.nj,1); v = zeros(obj.nj,1); a = zeros(obj.nj,1);
            if ~isempty(obj.plan_idx{traj_idx})
                for idx=obj.plan_idx{traj_idx}
                    t_limit(idx) = LimitNumber(0,t,obj.segplanner{traj_idx}{idx}.tf);
                    [p(idx),v(idx),a(idx)] = obj.segplanner{traj_idx}{idx}.GenerateMotion(t_limit(idx));
                end
            end
            if ~isempty(obj.unplan_idx{traj_idx})
                for idx=obj.unplan_idx{traj_idx}
                    p(idx) = obj.jpos(idx,traj_idx);
                end
            end
        end

        function [pos,vel,acc] = GenerateTraj(obj,dt)
            pos = []; vel = []; acc = [];
            for traj_idx=1:obj.ntraj
                for t=0:dt:obj.tf(traj_idx)
                    [p,v,a] = obj.GenerateMotion(traj_idx,t);
                    pos = [pos,p]; vel = [vel,v]; acc = [acc, a];
                end
            end
        end

    end




end