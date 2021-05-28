classdef CTrajPlanner < handle

    properties
        line_planner
        arc_planner
        ntraj

        pos_initial
        rpy_initial
    end

    methods
        function obj = CTrajPlanner(pos0, rpy0)
            obj.pos_initial = pos0;
            obj.rpy_initial = rpy0;
            obj.ntraj = 0;
        end

        function AddSegment(obj,traj_planner,style)
            obj.ntraj = obj.ntraj+1;
            if strcmp(style,'line')
                obj.line_planner{obj.ntraj} = traj_planner;
            end
        end

        function [pos,rpy] = GeneratePath(obj,dt)
            pos = []; rpy = [];
            for idx=1:obj.ntraj
                [p,r] = obj.line_planner{idx}.GeneratePath(dt);
                pos = [pos,p]; rpy = [rpy,r];
            end
        end


    end


end