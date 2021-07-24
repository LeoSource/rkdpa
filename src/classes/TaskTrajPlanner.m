classdef TaskTrajPlanner < handle
    
    properties
        ntraj
        traj_type
        robot
        tf
        segplanner
    end
    
    methods
        function obj = TaskTrajPlanner(rbt_model)
            obj.robot = rbt_model;
            obj.ntraj = 0;
        end
        
        function AddTraj(obj, via_pos, traj_type, traj_opt)
            switch traj_type
                case 'cartesian'
                    cplanner = CartesianBasePlanner(via_pos(:,1), traj_opt);
                    cplanner.AddPosRPY(via_pos(:,2:end));
                    obj.ntraj = obj.ntraj+1;
                    obj.segplanner{obj.ntraj} = cplanner;
                    obj.traj_type{obj.ntraj} = 'cartesian';
                case 'joint'
                    jplanner = JointPlanner(via_pos(:,1), traj_opt);
                    jplanner.AddJntPos(via_pos(:,2:end));
                    obj.ntraj = obj.ntraj+1;
                    obj.segplanner{obj.ntraj} = jplanner;
                    obj.traj_type{obj.ntraj} = 'joint';
            end
        end

        function [pos,vel,acc] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = [];
            for traj_idx=1:obj.ntraj
                switch obj.traj_type{traj_idx}
                    case 'cartesian'
                        [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                        pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                        pos = [pos,pos_tmp]; vel = [vel,vel_tmp]; acc = [acc,acc_tmp];
                    case 'joint'
                        [p,v,a] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                        pos = [pos, p]; vel = [vel, v]; acc = [acc, a];
                end
            end
        end



    end

    

end

