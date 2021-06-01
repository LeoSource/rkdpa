classdef CTrajPlanner < handle

    properties
        segpath_planner
        ntraj

        pos_corner
        rpy_corner
        pos_seg
        rpy_seg
    end

    methods
        function obj = CTrajPlanner(pos0, rpy0)
            obj.pos_corner = pos0;
            obj.rpy_corner = rpy0;
            obj.ntraj = 0;
        end

        function AddSegment(obj,traj_planner)
            obj.ntraj = obj.ntraj+1;
            obj.segpath_planner{obj.ntraj} = traj_planner;
        end
        
        function AddPosRPY(obj,pos_rpy,opt)
            global g_cvmax g_camax
            obj.pos_corner = [obj.pos_corner,pos_rpy(1:3)];
            obj.rpy_corner = [obj.rpy_corner,pos_rpy(4:6)];
            pos_idx = size(obj.pos_corner,2);
            obj.ntraj = obj.ntraj+1;
            pos0 = obj.pos_corner(:,pos_idx-1);
            posn = obj.pos_corner(:,pos_idx);
            rpy0 = obj.rpy_corner(:,pos_idx-1);
            rpyn = obj.rpy_corner(:,pos_idx);
            obj.segpath_planner{obj.ntraj} = LineTrajPlanner(pos0,posn,g_cvmax,g_camax,[0,0],...
                                                            rpy0,rpyn,0.15,0.3,[0,0],opt);
        end

        function [pos,rpy] = GeneratePath(obj,dt)
            pos = []; rpy = [];
            for idx=1:obj.ntraj
                [p,r] = obj.segpath_planner{idx}.GeneratePath(dt);
                pos = [pos,p]; rpy = [rpy,r];
            end
        end


    end


end