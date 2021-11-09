classdef TaskTrajPlanner < handle
    
    properties
        ntraj
        traj_type
%         tf
        segplanner

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
    end
    
    methods
        function obj = TaskTrajPlanner(rbt_model,q0,cycle_time,jvmax,jamax,...
                                                        cvmax,camax,compare_plan)
            obj.robot = rbt_model;
            obj.pre_q = q0;
            obj.compare_plan = compare_plan;
            obj.ntraj = 0;
            obj.pre_trajq = q0;
            obj.pre_trajpose = obj.robot.fkine(q0);
            obj.cycle_time = cycle_time;
            obj.jvmax = jvmax;
            obj.jamax = jamax;
            obj.cvmax = cvmax;
            obj.camax = camax;
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
                cplanner = CartesianPlanner([pos0;rpy0], traj_opt);
                cplanner.AddPosRPY(via_pos,vmax,amax);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = cplanner;
                obj.traj_type{obj.ntraj} = 'cartesian';
                obj.pre_trajpose  = SE3.rpy(180/pi*via_pos(4:6,end)', 'xyz');
                obj.pre_trajpose.t = via_pos(1:3,end);
                obj.pre_trajq = obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5)';
            case 'joint'
                jplanner = JointPlanner(obj.pre_trajq, traj_opt);
                jplanner.AddJntPos(via_pos, vmax, amax);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = jplanner;
                obj.traj_type{obj.ntraj} = 'joint';
                obj.pre_trajq = via_pos(:,end);
                obj.pre_trajpose = obj.robot.fkine(obj.pre_trajq);
            case 'bspline'
                pos0 = obj.pre_trajpose.t;
                rpy0 = tr2rpy(obj.pre_trajpose, 'xyz')';
                pos_rpy0 = [pos0;rpy0];
                tf_uk = CalcBSplineTime([pos_rpy0,via_pos],vmax);
                if traj_opt==1
                    option = 'interpolation';
                elseif traj_opt==0
                    option = 'approximation';
                else
                    option = 'ctrlpos';
                end
                posplanner = CubicBSplinePlanner([pos_rpy0,via_pos], option, tf_uk);
%                 rot_data = RPY2AxisAngle([rpy0,via_pos(4:6,:)]);
%                 rotplanner = CubicBSplinePlanner(rot_data, traj_opt, tf_uk);
%                 rotplanner = CubicBSplinePlanner([rpy0,via_pos(4:6,:)], traj_opt, tf_uk);
%                 obj.test_rpy = [rpy0,via_pos(4:6,:)];
%                 obj.test_tf = tf_uk;
                obj.ntraj = obj.ntraj+1;
%                 obj.segplanner{obj.ntraj} = {posplanner, rotplanner};
                obj.segplanner{obj.ntraj} = posplanner;
                obj.traj_type{obj.ntraj} = 'bspline';
                obj.pre_trajpose = SE3.rpy(180/pi*via_pos(4:6,end)', 'xyz');
                obj.pre_trajpose.t = via_pos(1:3,end);
                obj.pre_trajq = obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5);
            case 'arc'
                %add line trajectory befor arc for transition
                pos0 = obj.pre_trajpose.t;
                rpy0 = tr2rpy(obj.pre_trajpose, 'xyz')';
                cplanner = CartesianPlanner([pos0;rpy0], false);
                cplanner.AddPosRPY(via_pos(:,1),vmax,amax);
                obj.ntraj = obj.ntraj+1;
                obj.segplanner{obj.ntraj} = cplanner;
                obj.traj_type{obj.ntraj} = 'cartesian';
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
                obj.pre_trajq = obj.robot.ikine(obj.pre_trajpose,'q0',obj.pre_trajq','tol',1e-5);
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
                case 'bspline'
                    [p,vp,ap] = obj.segplanner{traj_idx}{1}.GenerateTraj(dt);
                    [r,vr,ar] = obj.segplanner{traj_idx}{2}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
                    [jp,jv,ja,cp_sim] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                    cpos_sim=[cpos_sim,cp_sim];
                case 'arc'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
                    [jp,jv,ja,cp_sim] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
                    jpos = [jpos,jp]; jvel = [jvel, jv]; jacc = [jacc,ja];
                    cpos_sim=[cpos_sim,cp_sim];
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
%                     cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
                    [cpos] = [cpos,p]; cvel = [cvel,vp]; cacc = [cacc, ap];
                case 'arc'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    cpos = [cpos,pos_tmp]; cvel = [cvel,vel_tmp]; cacc = [cacc,acc_tmp];
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
                    [r,vr,ar] = obj.segplanner{traj_idx}{2}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    [jp,jv,ja] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
                    jpos = [jpos,jp]; jvel = [jvel,jv]; jacc = [jacc,ja];
                case 'arc'
                    [p,vp,ap,r,vr,ar] = obj.segplanner{traj_idx}.GenerateTraj(dt);
                    pos_tmp = [p;r]; vel_tmp = [vp;vr]; acc_tmp = [ap;ar];
                    [jp,jv,ja] = obj.Transform2Joint(pos_tmp,vel_tmp,acc_tmp);
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
                if size(q)~=6
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
        r0 = rpy2r(180/pi*pos_rpy(4:6,idx)','xyz');
        rn = rpy2r(180/pi*pos_rpy(4:6,idx+1)','xyz');
        delta_rot = rn*r0';
        [rpy_len,~] = tr2angvec(delta_rot);
        rot_len = rot_len+rpy_len;
    end
    tscale = 1.5;
    tf_pos = pos_len/cvmax(1)*tscale;
    tf_rot = rot_len/cvmax(2)*tscale;
    tf = max([tf_pos,tf_rot]);
end



