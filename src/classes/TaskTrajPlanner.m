classdef TaskTrajPlanner < handle
    
    properties
        ntraj
        traj_type
        tf
        segplanner

        robot
        pre_q
        pre_trajq
        pre_trajpose
        compare_plan
    end
    
    methods
        function obj = TaskTrajPlanner(rbt_model,q0,compare_plan)
            obj.robot = rbt_model;
            obj.pre_q = q0;
            obj.compare_plan = compare_plan;
            obj.ntraj = 0;
            obj.pre_trajq = q0;
            obj.pre_trajpose = obj.robot.fkine(q0);
        end
        
        function AddTraj(obj, via_pos, traj_type, traj_opt)
            switch traj_type
            case 'cartesian'
                pos0 = obj.pre_trajpose.t;
                rpy0 = tr2rpy(obj.pre_trajpose,'xyz')';
                cplanner = CartesianBasePlanner([pos0;rpy0], traj_opt);
                cplanner.AddPosRPY(via_pos);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = cplanner;
                obj.traj_type{obj.ntraj} = 'cartesian';
                obj.pre_trajpose  = SE3.rpy(180/pi*via_pos(4:6,end)', 'xyz');
                obj.pre_trajpose.t = via_pos(1:3,end);
                obj.pre_trajq = obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5)';
            case 'joint'
                jplanner = JointPlanner(obj.pre_trajq, traj_opt);
                jplanner.AddJntPos(via_pos);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = jplanner;
                obj.traj_type{obj.ntraj} = 'joint';
                obj.pre_trajq = via_pos(:,end);
                obj.pre_trajpose = obj.robot.fkine(obj.pre_trajq);
            end
        end


        function [cpos,cvel,cacc,jpos,jvel,jacc,cpos_sim] = GenerateBothTraj(obj,dt)
            cpos=[]; cvel=[]; cacc=[];
            jpos=[]; jvel=[]; jacc=[];
            cpos_sim=[];
            for traj_idx=1:obj.ntraj
                switch obj.traj_type{traj_idx}
                case 'cartesian'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
                    [jp,jv,ja,cp_sim] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                    cpos_sim=[cpos_sim,cp_sim];
                case 'joint'
                    [jp,jv,ja] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
%                     [p,v,a] = obj.Transform2Cart(jp,jv,ja);
%                     cpos = [cpos, p]; cvel = [cvel, v]; cacc = [cacc, a];
%                     if obj.compare_plan
%                         cpos_sim = [cpos_sim,p(1:3,:)];
%                     end
                end
            end
        end

        function [cpos,cvel,cacc] = GenerateCartTraj(obj, dt)
            cpos = []; cvel = []; cacc = [];
            for traj_idx=1:obj.ntraj
                switch obj.traj_type{traj_idx}
                case 'cartesian'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
                case 'joint'
                    [jp,jv,ja] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    [p,v,a] = obj.Transform2Cart(jp,jv,ja);
                    cpos = [cpos, p]; cvel = [cvel, v]; cacc = [acc, a];
                end
            end
        end

        function [jpos,jvel,jacc] = GenerateJointTraj(obj,dt)
            jpos=[]; jvel=[]; jacc=[];
            for traj_idx=1:obj.ntraj
                switch obj.traj_type{traj_idx}
                case 'cartesian'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    [jp,jv,ja] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                case 'joint'
                    [jp,jv,ja] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                end
            end
        end
        
        function [cpos,cvel,cacc] = Transform2Cart(obj,jp,jv,ja)
            cpos=[]; cvel=[]; cacc=[];
            np = size(jp,2);
            for idx=1:np
                q = jp(:,idx); qd = jv(:,idx); qdd = ja(:,idx);
                jaco = obj.robot.jacob0(q);
                jaco_dot = obj.robot.jacob_dot(q,qd);
                pose = obj.robot.fkine(q);
                pos = pose.t; rpy = tr2rpy(pose,'xyz');
                cp = [pos;rpy'];
                cv = jaco*qd;
                ca = jaco*qdd+jaco_dot;
                obj.pre_q = q;
                cpos=[cpos,cp]; cvel=[cvel,cv]; cacc=[cacc,ca];
            end
        end

        function [jpos,jvel,jacc,cpos_sim] = Transform2Joint(obj,cp,cv,ca)
            jpos=[]; jvel=[]; jacc=[]; cpos_sim=[];
            np = size(cp,2);
            for idx=1:np
                cmd_pose = SE3.rpy(180/pi*cp(4:6,idx)','xyz');
                cmd_pose.t = cp(1:3,idx);
                q = obj.robot.ikine(cmd_pose,'q0',obj.pre_q','tol',1e-5)';
                jaco = obj.robot.jacob0(q);
                qd = jaco\cv(:,idx);
                jaco_dot = obj.robot.jacob_dot(q',qd');
                qdd = jaco\(ca(:,idx)-jaco_dot);
                obj.pre_q = q;
                jpos=[jpos,q]; jvel=[jvel,qd]; jacc=[jacc,qdd];
                if obj.compare_plan
                    cp_sim = obj.robot.fkine(q).t;
                    cpos_sim = [cpos_sim, cp_sim];
                end
            end
        end


    end



end

