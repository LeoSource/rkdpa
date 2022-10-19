%   Trajectory Planner Class:
%   A specific class that contains all kinds of trajectories: joint space, line , arc and b-spline
%   Planning scene can be updated in this class, but there will be only one
%   planning scene, it should be reset when use another scene
%   Author:
%   liao zhixiang, zhixiangleo@163.com

classdef TaskTrajPlanner < handle
    
    properties
        ntraj
        traj_type
%         tf
        segplanner
        seg_idx
        task_completed

        robot
        pre_q
        pre_trajq
        pre_trajpose
        compare_plan
        
        cycle_time
        jvmax
        jamax
        cvmax
        camax
        
        params_clean_toilet
    end
    
    methods
        %% Constructor of Class and Other Settings
        function obj = TaskTrajPlanner(rbt_model,q0,cycle_time,jvmax,jamax,...
                                                        cvmax,camax,compare_plan)
            obj.robot = rbt_model;
            obj.pre_q = q0;
            obj.compare_plan = compare_plan;
            obj.ntraj = 0;
            obj.seg_idx = 1;
            obj.task_completed = false;
            obj.pre_trajq = q0;
            obj.pre_trajpose = obj.robot.fkine(q0);
            obj.cycle_time = cycle_time;
            obj.jvmax = jvmax;
            obj.jamax = jamax;
            obj.cvmax = cvmax;
            obj.camax = camax;
        end
        
        %% Add All Kinds of Trajectories
        function SetPlanningScene(obj,vertices,slant_angle)
            obj.params_clean_toilet.center = mean(vertices,2);
            obj.params_clean_toilet.slant = slant_angle;
%             center = mean(vertices,2);
%             for idx=1:size(vertices,2)
%                 pos_tmp = vertices(:,idx);
%                 center_tmp = [center(1),center(2),pos_tmp(3)]';
%                 len = norm(center_tmp-pos_tmp);
%                 peak(:,idx) = center_tmp;
%                 peak(3,idx) = center_tmp(3)+len*cot(slant_angle);
%             end
%             obj.params_clean_toilet.peak = mean(peak,2);
        end
        
        function AddTraj(obj, via_pos, traj_type, traj_opt, vmax_arg, amax_arg)
            if strcmp(traj_type,'joint')
                vmax = obj.jvmax;
                amax = obj.jamax;
            else
                vmax = obj.cvmax;
                amax = obj.camax;
            end
            if nargin==6
                if isscalar(vmax_arg)
                    vmax = vmax*vmax_arg;
                else
                    vmax = vmax_arg;
                end
                if isscalar(amax_arg)
                    amax = amax*amax_arg;
                else
                    amax = amax_arg;
                end
            end

            switch traj_type
            case 'cartesian'
                pos0 = obj.pre_trajpose.t;
                rpy0 = tr2rpy(obj.pre_trajpose,'xyz')';
                cplanner = CartesianPlanner([pos0;rpy0], traj_opt, obj.cycle_time);
                cplanner.AddPosRPY(via_pos,vmax,amax);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = cplanner;
                obj.traj_type{obj.ntraj} = 'cartesian';
                obj.pre_trajpose  = SE3.rpy(180/pi*via_pos(4:6,end)', 'xyz');
                obj.pre_trajpose.t = via_pos(1:3,end);
                if obj.compare_plan
                    obj.pre_trajq = reshape(obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5)',...
                                                            obj.robot.n,1);
                end
            case 'joint'
                jplanner = JointPlanner(obj.pre_trajq, traj_opt);
                jplanner.AddJntPos(via_pos, vmax, amax);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = jplanner;
                obj.traj_type{obj.ntraj} = 'joint';
                obj.pre_trajq = reshape(via_pos(:,end),obj.robot.n,1);
                obj.pre_trajpose = obj.robot.fkine(obj.pre_trajq);
            case 'bspline'
                %add line trajectory before b-spline for transition
                pos0 = obj.pre_trajpose.t;
                rpy0 = tr2rpy(obj.pre_trajpose, 'xyz')';
                cplanner = CartesianPlanner([pos0;rpy0],false);
                rpy = obj.CalcCleanToiletRPY(via_pos(:,1));
                cplanner.AddPosRPY([via_pos(:,1);rpy'],vmax,amax);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = cplanner;
                obj.traj_type{obj.ntraj} = 'cartesian';
                obj.pre_trajpose = SE3.rpy(180/pi*rpy,'xyz');
                obj.pre_trajpose.t = via_pos(:,1);
%                 if obj.compare_plan
%                     obj.pre_trajq = reshape(obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5),...
%                                                             obj.robot.n,1);
%                 end
                %add b-spline trajectory
                tf_uk = CalcBSplineTime(via_pos,vmax);
                if traj_opt==1
                    option = 'interpolation';
                elseif traj_opt==0
                    option = 'approximation';
                else
                    option = 'ctrlpos';
                end
                posplanner = CubicBSplinePlanner(via_pos, option, tf_uk);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = posplanner;
                obj.traj_type{obj.ntraj} = 'bspline';
            case 'arc'
                %add line trajectory befor arc for transition
                pos0 = obj.pre_trajpose.t;
                rpy0 = tr2rpy(obj.pre_trajpose, 'xyz')';
                cplanner = CartesianPlanner([pos0;rpy0], false);
                cplanner.AddPosRPY(via_pos(:,1),vmax,amax);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = cplanner;
                obj.traj_type{obj.ntraj} = 'cartesian';
                obj.pre_trajpose = SE3.rpy(180/pi*via_pos(4:6,1)', 'xyz');
                obj.pre_trajpose.t = via_pos(1:3,1);
                if obj.compare_plan
                    obj.pre_trajq = reshape(obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5),...
                                                            obj.robot.n,1);
                end
                %add arc trajectory
                pos1 = via_pos(1:3,1); pos2 = via_pos(1:3,2); pos3 = via_pos(1:3,3);
                rpy1 = via_pos(4:6,1); rpy3 = via_pos(4:6,3);
                arcplanner = ArcPlanner(pos1,pos2,pos3,vmax(1),amax(1),[0,0],...
                                        rpy1,rpy3,vmax(2),amax(2),[0,0], 'arc');
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = arcplanner;
                obj.traj_type{obj.ntraj} = 'arc';
                obj.pre_trajpose = SE3.rpy(180/pi*rpy3', 'xyz');
                obj.pre_trajpose.t = pos3;
                if obj.compare_plan
                    obj.pre_trajq = reshape(obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5),...
                                                            obj.robot.n,1);
                end                              
            end
        end
        
        function AddDynIdenTraj(obj,planner)
            % add joint trajectory before dynamic identification trajectory
            jplanner = JointPlanner(obj.pre_trajq,1);
            jplanner.AddJntPos(planner.q0,obj.jvmax,obj.jamax);
            obj.ntraj = obj.ntraj+1;
            obj.segplanner{obj.ntraj} = jplanner;
            obj.traj_type{obj.ntraj} = 'joint';
            obj.pre_trajq = planner.q0;
            obj.pre_trajpose = obj.robot.fkine(obj.pre_trajq);
            % add robot dynamic identification trajectory
            obj.ntraj = obj.ntraj+1;
            obj.segplanner{obj.ntraj} = planner;
            obj.traj_type{obj.ntraj} = 'dyniden';
        end

        %% Generate Joint-Space and Cartesian-Space Trajectory
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
                    [p,v,a] = obj.Transform2Cart(jp,jv,ja);
                    cpos = [cpos, p]; cvel = [cvel, v]; cacc = [cacc, a];
                    if obj.compare_plan
                        cpos_sim = [cpos_sim,p(1:3,:)];
                    end
                case 'bspline'
                    [p,vp,ap] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    clear r vr ar;
                    for idx=1:size(p,2)
                        r(:,idx) = obj.CalcCleanToiletRPY(p(:,idx));
                        vr(:,idx) = zeros(3,1);
                        ar(:,idx) = zeros(3,1);
                    end
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
                    [jp,jv,ja,cp_sim] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                    cpos_sim=[cpos_sim,cp_sim];
                    
                    obj.pre_trajpose = SE3.rpy(180/pi*cpos(4:6,end)', 'xyz');
                    obj.pre_trajpose.t = cpos(1:3,end);
                    obj.pre_trajq = reshape(obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5),...
                                                            obj.robot.n,1);
                case 'arc'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
                    [jp,jv,ja,cp_sim] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                    cpos_sim=[cpos_sim,cp_sim];
                case 'dyniden'
                    [jp,jv,ja] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                    [p,v,a] = obj.Transform2Cart(jp,jv,ja);
                    cpos = [cpos, p]; cvel = [cvel, v]; cacc = [cacc, a];
                    if obj.compare_plan
                        cpos_sim = [cpos_sim,p(1:3,:)];
                    end
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
                    cpos = [cpos, p]; cvel = [cvel, v]; cacc = [cacc, a];
                case 'bspline'
                    [p,vp,ap] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    for idx=1:size(p,2)
                        r(:,idx) = obj.CalcCleanToiletRPY(p(:,idx));
                        vr(:,idx) = zeros(3,1);
                        ar(:,idx) = zeros(3,1);
                    end
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
%                     uplanner = LspbPlanner([0,obj.test_tf],2,1,obj.test_tf);
%                     for idx=1:size(obj.test_rpy,2)
%                         rot_mat = rpy2r(180/pi*obj.test_rpy(:,idx)', 'xyz');
%                         quat(:,idx) = dcm2quat(rot_mat)';
%                     end
%                     r = [];
%                     for t=0:dt:obj.test_tf
%                         [u,du,ddu] = uplanner.GenerateMotion(t);
%                         s = u/obj.test_tf;
%                         qu = quat_squad(quat, s);
%                         quat_save(:,idx) = qu';
%                         rot_mat = quat2dcm(qu);
%                         r = [r, tr2rpy(rot_mat, 'xyz')'];
%                     end

%                     [r,vr,ar] = obj.segplanner{traj_idx}{2}.GenerateTraj(dt);
%                     r = AxisAngle2RPY(r_tmp);
%                     vr = zeros(size(r));
%                     ar = zeros(size(r));
%                     pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
                    obj.pre_trajpose = SE3.rpy(180/pi*cpos(4:6,end)', 'xyz');
                    obj.pre_trajpose.t = cpos(1:3,end);
                    obj.pre_trajq = reshape(obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5),...
                                                            obj.robot.n,1);
                case 'arc'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
                case 'dyniden'
                   [jp,jv,ja] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                   [p,v,a] = obj.Transform2Cart(jp,jv,ja);
                   cpos = [cpos, p]; cvel = [cvel, v]; cacc = [cacc, a];
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
                case 'bspline'
                    [p,vp,ap] = obj.segplanner{traj_idx}{1}.GenerateTraj(dt);
                    for idx=1:size(p,2)
                        r(:,idx) = obj.CalcCleanToiletRPY(p(:,idx));
                        vr(:,idx) = zeros(3,1);
                        ar(:,idx) = zeros(3,1);
                    end
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    obj.pre_trajpose = SE3.rpy(180/pi*pos_tmp(4:6,end)', 'xyz');
                    obj.pre_trajpose.t = pos_tmp(1:3,end);
                    obj.pre_trajq = reshape(obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5),...
                                                            obj.robot.n,1);
                    [jp,jv,ja] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
                    jpos = [jpos,jp]; jvel = [jvel,jv]; jacc = [jacc,ja];
                case 'arc'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    [jp,jv,ja] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                case 'dyniden'
                    [jp,jv,ja] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    obj.pre_trajq = jp(:,end);
                    obj.pre_trajpose = obj.robot.fkine(obj.pre_trajq);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                end
            end
        end
        
        function [cp,cv,ca,jp,jv,ja] = GenerateBothMotion(obj,t)
            switch obj.traj_type{obj.seg_idx}
                case 'cartesian'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{obj.seg_idx}.GenerateMotion();
                    cp = [p;r]; cv = [vp;vr]; ca = [ap;ar];
                    [jp,jv,ja,~] = obj.Transform2Joint(cp,cv,ca);
                
                
            end
            
            if obj.segplanner{obj.seg_idx}.plan_completed
                if obj.seg_idx==obj.ntraj
                    obj.task_completed = true;
                    obj.pre_trajq = jp;
                    obj.pre_trajpose = SE3.rpy(180/pi*cp(4:6)', 'xyz');
                    obj.pre_trajpose.t = cp(1:3);
                else
                    obj.seg_idx = obj.seg_idx+1;
                end
            end
        end

        function rpy = CalcCleanToiletRPY(obj,pos)
            % the intersecting line is perpendicular to normal vectors of two intersecting surface
            % z0 is one of normal vector
            % [0,1,0]' is another normal vector
%             z0 = pos-obj.params_clean_toilet.peak;
            height = 0.1;
            len = height*tan(obj.params_clean_toilet.slant);
            vec_tmp = obj.params_clean_toilet.center-pos;
            pos_tmp = pos+len*vec_tmp/norm(vec_tmp);
            peak = [pos_tmp(1);pos_tmp(2);pos_tmp(3)+height];
            z0 = pos-peak;
            z0 = z0/norm(z0);
            % TO DO: there is a risk in calculating y0 when z0 is changing
            % because that y0 has 2 directions
            y0 = cross(z0,[-1,0,0]');
            x0 = cross(y0,z0);
            rpy = tr2rpy([x0,y0,z0],'xyz');
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
                if length(q)~=6
                    a = 1;
                end
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
        
        function Reset(obj,q0)
            obj.pre_q = q0;
            obj.pre_trajq = q0;
            obj.pre_trajpose = obj.robot.fkine(q0);
            obj.segplanner = {};
            obj.traj_type = {};
            obj.ntraj = 0;
        end

    end



end

function tf = CalcBSplineTime(pos_rpy,cvmax)
    np = size(pos_rpy,2);
    pos_len = 0;
    rot_len = 0;
    for idx=1:np-1
        pos_len = pos_len+norm(pos_rpy(1:3,idx+1)-pos_rpy(1:3,idx));
        if size(pos_rpy,1)==6
            r0 = rpy2r(180/pi*pos_rpy(4:6,idx)','xyz');
            rn = rpy2r(180/pi*pos_rpy(4:6,idx+1)','xyz');
            delta_rot = rn*r0';
            [rpy_len,~] = tr2angvec(delta_rot);
            rot_len = rot_len+rpy_len;
        end
    end
    tscale = 3;
    tf_pos = pos_len/cvmax(1)*tscale;
    tf_rot = rot_len/cvmax(2)*tscale;
    tf = max([tf_pos,tf_rot]);
end



