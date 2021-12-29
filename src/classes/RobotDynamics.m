classdef RobotDynamics < handle
    
    properties
%         barycenter_j5 %% l5x,l5y,l6x,l6y,l6z,m6
%         barycenter_j4 %% l4x,l4y,l5x,l5y,l5z,m5,l6x,l6y,l6z,m6
%         barycenter_j3 %% l3x,l3y,l4x,l4y,m4,l5x,l5y,l5z,m5,l6x,l6y,l6z,m6
%         barycenter_j2 %% l2x,l2y,l3x,l3y,m3,l4x,l4y,m4,l5x,l5y,l5z,m5,l6x,l6y,l6z,m6
        barycenter_params
        fric_params
        rbtdef
        njoint
        g
    end
    
    
    methods
        function obj = RobotDynamics(rbtdef)
            obj.rbtdef = rbtdef;
            obj.g = abs(rbtdef.gravity(3));
            obj.njoint = rbtdef.n;
        end
        
        function LoadParams(obj,grav_file,fric_file)
            obj.barycenter_params = load(grav_file);
            obj.fric_params = load(fric_file);
        end
        
        function GravityIden(obj,jpos,jtau)
            [jpos_grav,jtau_grav] = obj.ProcessGravJointData(jpos,jtau);
            %%gravity identification%%
            sparse_value = 5;
            jpos_iden = jpos_grav(1:sparse_value:end,:);
            jtau_iden_tmp = jtau_grav(1:sparse_value:end,:);
            reg_mat = []; jtau_iden = [];
            for idx=1:size(jpos_iden,1)
                reg_tmp = obj.CalcRegMat(jpos_iden(idx,:));
                reg_mat = [reg_mat;reg_tmp];
                jtau_iden = [jtau_iden; jtau_iden_tmp(idx,2:5)'];
            end
            obj.barycenter_params = pinv(reg_mat)*jtau_iden;
            tau_val = [];
            for idx=1:size(jpos_grav,1)
                tau_tmp = obj.GenerateGravTau(jpos_grav(idx,:));
                tau_val = [tau_val;tau_tmp'];
            end
            figure
            for idx=2:5
                subplot(2,2,idx-1)
                plot(jtau_grav(:,idx),'r'); grid on; hold on;
                plot(tau_val(:,idx),'k'); hold off;
%                 legend('identification','validation');
                title(['joint',num2str(idx),' gravity validation']);
            end
        end

        function FrictionIden(obj,jv1,jt1,jv2,jt2)
            [jvel1,jtau1,jvel2,jtau2] = obj.ProcessFricJointData(jv1,jt1,jv2,jt2);
            start1_idx = [273,2588,7666,11365,15562,18382,21877,24959,...
                            28487,31705,35257,38562,42131,45473,48896,...
                            52033,55243,58229,61284,64155,67088,69855];
            stop1_idx = [1512,6318,10146,14468,17413,20856,23929,27426,...
                            30636,34166,37461,41014,44370,47816,50979,...
                            54205,57209,60272,63156,66096,68884,71651];
            start2_idx = [143,1309,3861,5725,7852,9293,11083,12670,...
                            14491,16161,18010,19737,21609,23370,25184,...
                            26858,28582,30195,31855,33426,35041,36575];
            stop2_idx = [757,3168,5090,7267,8757,10508,12078,13871,...
                            15524,17348,19058,20910,22666,24479,26153,...
                            27871,29480,31133,32697,34302,35834,37368];
            [fc1,fv1,jvel1_iden,jtau1_iden] = obj.JointFrictionIden(jvel1,jtau1,start1_idx,stop1_idx);
            [fc2,fv2,jvel2_iden,jtau2_iden] = obj.JointFrictionIden(jvel2,jtau2,start2_idx,stop2_idx);
            obj.fric_params = [fc1,fv1,fc1,fv1,fc1,fv1,fc2,fv2,fc2,fv2,fc2,fv2]';
            obj.PolyfitJointFriction(jvel1_iden,jtau1_iden,jvel2_iden,jtau2_iden);
            figure
            subplot(2,1,1); obj.ValidateJointFriction(jv1,jt1,1);
            subplot(2,1,2); obj.ValidateJointFriction(jv2,jt2,5);
        end

        function [fc,fv,jvel_iden,tau_iden] = JointFrictionIden(obj,jv,jt,start_idx,stop_idx)
            tau_iden = []; jvel_iden = []; reg_fric = [];
            for idx=1:length(start_idx)
                vel = jv(start_idx(idx):stop_idx(idx));
                vel = mean(vel);
                reg_mat = [sign(vel),vel];
                reg_fric = [reg_fric; reg_mat];
                jvel_iden = [jvel_iden;vel];
                tau_iden = [tau_iden;mean(jt(start_idx(idx):stop_idx(idx)))];
            end
            joint_fric_params = pinv(reg_fric)*tau_iden;
            fc = joint_fric_params(1);
            fv = joint_fric_params(2);
        end

        function PolyfitJointFriction(obj,jv1_iden,jt1_iden,jv2_iden,jt2_iden)
            vpoly1 = sort(jv1_iden);
            tpoly1 = sort(jt1_iden);
            vpoly2 = sort(jv2_iden);
            tpoly2 = sort(jt2_iden);
            figure
            subplot(2,1,1)
            plot(vpoly1,tpoly1,'o'); grid on; hold on;
            vv = min(vpoly1):0.01:max(vpoly1);
            plot(vv,obj.fric_params(1)*sign(vv)+obj.fric_params(2)*vv,'k');
            xlabel('velocity(rad/s)'); ylabel('tau(Nm)');
            subplot(2,1,2)
            plot(vpoly2,tpoly2,'o'); grid on; hold on;
            vv = min(vpoly2):0.01:max(vpoly2);
            plot(vv,obj.fric_params(end-1)*sign(vv)+obj.fric_params(end)*vv,'k');
            xlabel('velocity(rad/s)'); ylabel('tau(Nm)');
        end

        function ValidateJointFriction(obj,jv,jt,jidx)
            if jidx==1
                pidx=1;
            elseif jidx==5
                pidx=11;
            end
            tau_val = [];
            for nidx=1:length(jv)
                v = jv(nidx);
                tau_tmp = obj.fric_params(pidx)*SignVel(v)+obj.fric_params(pidx+1)*v;
                tau_val = [tau_val;tau_tmp];
            end
            plot(jt,'r'); grid on; hold on;
            plot(tau_val,'k');
        end
        
        function [jpos_grav,jtau_grav] = ProcessGravJointData(obj,jpos,jtau)
            %%process identification data%%
            start_idx = [1000,22339];
            stop_idx = [20341,41680];
            jpos_grav = jpos(start_idx(1):stop_idx(1),:);
            tau1 = jtau(start_idx(1):stop_idx(1),:);
            tau2_tmp = jtau(start_idx(2):stop_idx(2),:);
            tau2 = fliplr(tau2_tmp');
            tau2 = tau2';
            jtau_grav = 0.5*(tau1+tau2);
            %%filter joint torque data%%
            fs = 200;
            N = 2;
            fc = 5;
            [Btorque, Atorque] = butter(N, fc/(fs/2));
            for idx=1:size(jtau_grav,2)
                jtau_grav(:,idx) = filtfilt(Btorque, Atorque, jtau_grav(:,idx));
            end
            figure
            for idx=2:5
                subplot(2,2,idx-1)
                plot(tau1(:,idx)); grid on; hold on;
                plot(tau2(:,idx)); plot(jtau_grav(:,idx)); hold off;
                title(['joint',num2str(idx),' gravity']);
            end
        end

        function [jvel1,jtau1,jvel2,jtau2] = ProcessFricJointData(obj,jv1,jt1,jv2,jt2)
            fs = 200;
            N = 2;
            t1 = 0:1/fs:1/fs*(length(jv1)-1);
            fc_vel = 1; fc_tau = 2; fc_acc = 0.5;
            [Bvel,Avel] = butter(N,fc_vel/(fs/2));
            [Btau,Atau] = butter(N,fc_tau/(fs/2));
            [Bacc,Aacc] = butter(N,fc_acc/(fs/2));
            jvel1 = filtfilt(Bvel,Avel,jv1);
            jtau1 = filtfilt(Btau,Atau,jt1);
            ja1 = diff(jv1)*fs;
            jacc1 = filtfilt(Bacc,Aacc,ja1);
            jacc1 = [0;jacc1];

            jvel2 = filtfilt(Bvel,Avel,jv2);
            jtau2 = filtfilt(Btau,Atau,jt2);
            ja2 = diff(jv2)*fs;
            jacc2 = filtfilt(Bacc,Aacc,ja2);
            jacc2 = [0;jacc2];
            t2 = 0:1/fs:1/fs*(length(jv2)-1);

            figure
            subplot(2,1,1)
            plot(t1,jv1,'r'); grid on; hold on;
            plot(t1,jvel1,'k'); plot(t1,jacc1,'b'); hold off;
            title('joint1')
            subplot(2,1,2)
            plot(t2,jv2,'r'); grid on; hold on;
            plot(t2,jvel2,'k'); plot(t2,jacc2,'b'); hold off;
            title('joint5');
            figure
            subplot(2,1,1)
            plot(t1,jt1,'r',t1,jtau1,'k'); grid on; title('joint1');
            subplot(2,1,2)
            plot(t2,jt2,'r',t2,jtau2,'k'); grid on; title('joint5');
        end

        function reg_mat = CalcRegMat(obj,jpos)
            reg_mat = zeros(4,16);
            reg_mat(1,:) = obj.CalcRegressorJoint2(jpos);
            reg_mat(2,:) = obj.CalcRegressorJoint3(jpos);
            reg_mat(3,:) = obj.CalcRegressorJoint4(jpos);
            reg_mat(4,:) = obj.CalcRegressorJoint5(jpos);
        end
    
        function reg_vec = CalcRegressorJoint5(obj,q)
            reg_vec = zeros(1,16);
            q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
            reg_vec(9) = -9.81*sin(q5)*cos(q2+q3+q4);
            reg_vec(10) = -9.81*cos(q5)*cos(q2 + q3 + q4);
            reg_vec(13) = -9.81*sin(q5)*cos(q6)*cos(q2 + q3 + q4);
            reg_vec(14) = 9.81*sin(q5)*sin(q6)*cos(q2 + q3 + q4);
            reg_vec(15) = 9.81*cos(q5)*cos(q2 + q3 + q4);
            reg_vec(16) = obj.g*obj.rbtdef.d(6)*cos(q5)*cos(q2 + q3 + q4);
        end
        
        function reg_vec = CalcRegressorJoint4(obj,q)
            reg_vec = zeros(1,16);
            q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
            reg_vec(6) = -9.81*sin(q2 + q3 + q4);
            reg_vec(7) = -9.81*cos(q2 + q3 + q4);
            reg_vec(9) = -9.81*sin(q2 + q3 + q4)*cos(q5);
            reg_vec(10) = 9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(11) = -9.81*cos(q2 + q3 + q4);
            reg_vec(12) = -obj.g*obj.rbtdef.d(5)*cos(q2 + q3 + q4);
            reg_vec(13) = -9.81*sin(q6)*cos(q2 + q3 + q4) - 9.81*sin(q2 + q3 + q4)*cos(q5)*cos(q6);
            reg_vec(14) = 9.81*sin(q6)*sin(q2 + q3 + q4)*cos(q5) - 9.81*cos(q6)*cos(q2 + q3 + q4);
            reg_vec(15) = -9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(16) = -obj.g*obj.rbtdef.d(6)*sin(q5)*sin(q2 + q3 + q4)... 
                                - obj.g*obj.rbtdef.d(5)*cos(q2 + q3 + q4);          
        end
        
        function reg_vec = CalcRegressorJoint3(obj,q)
            reg_vec = zeros(1,16);
            q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
            reg_vec(3) = -9.81*cos(q2 + q3);
            reg_vec(4) = 9.81*sin(q2 + q3);
            reg_vec(6) = -9.81*sin(q2 + q3 + q4);
            reg_vec(7) = -9.81*cos(q2 + q3 + q4);
            reg_vec(8) = -obj.g*obj.rbtdef.a(4)*cos(q2 + q3);
            reg_vec(9) = -9.81*sin(q2 + q3 + q4)*cos(q5);
            reg_vec(10) = 9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(11) = -9.81*cos(q2 + q3 + q4);
            reg_vec(12) = -obj.g*obj.rbtdef.a(4)*cos(q2 + q3)...
                                - obj.g*obj.rbtdef.d(5)*cos(q2 + q3 + q4);
            reg_vec(13) = -9.81*sin(q6)*cos(q2 + q3 + q4) - 9.81*sin(q2 + q3 + q4)*cos(q5)*cos(q6);
            reg_vec(14) = 9.81*sin(q6)*sin(q2 + q3 + q4)*cos(q5) - 9.81*cos(q6)*cos(q2 + q3 + q4);
            reg_vec(15) = -9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(16) = -obj.g*obj.rbtdef.d(6)*sin(q5)*sin(q2 + q3 + q4)...
                                - obj.g*obj.rbtdef.a(4)*cos(q2 + q3)...
                                - obj.g*obj.rbtdef.d(5)*cos(q2 + q3 + q4);          
        end
        
        function reg_vec = CalcRegressorJoint2(obj,q)
            reg_vec = zeros(1,16);
            q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
            reg_vec(1) = -9.81*sin(q2);
            reg_vec(2) = -9.81*cos(q2);
            reg_vec(3) = -9.81*cos(q2 + q3);
            reg_vec(4) = 9.81*sin(q2 + q3);
            reg_vec(5) = obj.g*obj.rbtdef.a(3)*sin(q3)*cos(q2 + q3)...
                                - 0.01*obj.g*obj.rbtdef.a(3)*sin(q2 + q3)*cos(q3);
            reg_vec(6) = -9.81*sin(q2 + q3 + q4);
            reg_vec(7) = -9.81*cos(q2 + q3 + q4);
            reg_vec(8) = obj.g*obj.rbtdef.a(3)*sin(q3)*cos(q2 + q3)...
                                - obj.g*obj.rbtdef.a(4)*sin(q4)*sin(q2 + q3 + q4)...
                                - 0.01*obj.g*obj.rbtdef.a(3)*sin(q2 + q3)*cos(q3)...
                                - obj.g*obj.rbtdef.a(4)*cos(q4)*cos(q2 + q3 + q4);
            reg_vec(9) = -9.81*sin(q2 + q3 + q4)*cos(q5);
            reg_vec(10) = 9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(11) = -9.81*sin(q5)^2*cos(q2 + q3 + q4) - 9.81*cos(q5)^2*cos(q2 + q3 + q4);
            reg_vec(12) = obj.g*obj.rbtdef.a(3)*sin(q3)*cos(q2 + q3)...
                                - obj.g*obj.rbtdef.a(4)*sin(q4)*sin(q2 + q3 + q4)...
                                - obj.g*obj.rbtdef.d(5)*sin(q5)^2*cos(q2 + q3 + q4)...
                                - 0.01*obj.g*obj.rbtdef.a(3)*sin(q2 + q3)*cos(q3)...
                                - obj.g*obj.rbtdef.a(4)*cos(q4)*cos(q2 + q3 + q4)...
                                - obj.g*obj.rbtdef.d(5)*cos(q5)^2*cos(q2 + q3 + q4);
            reg_vec(13) = -9.81*sin(q5)^2*sin(q6)*cos(q2 + q3 + q4)...
                                - 9.81*sin(q6)*cos(q5)^2*cos(q2 + q3 + q4)...
                                - 9.81*sin(q2 + q3 + q4)*cos(q5)*cos(q6);
            reg_vec(14) = -9.81*sin(q5)^2*cos(q6)*cos(q2 + q3 + q4)...
                                + 9.81*sin(q6)*sin(q2 + q3 + q4)*cos(q5)...
                                 - 9.81*cos(q5)^2*cos(q6)*cos(q2 + q3 + q4);
            reg_vec(15) = -9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(16) = obj.g*obj.rbtdef.a(3)*sin(q3)*cos(q2 + q3)...
                                - obj.g*obj.rbtdef.a(4)*sin(q4)*sin(q2 + q3 + q4)...
                                - obj.g*obj.rbtdef.d(5)*sin(q5)^2*cos(q2 + q3 + q4)...
                                - obj.g*obj.rbtdef.d(6)*sin(q5)*sin(q2 + q3 + q4)...
                                - 0.01*obj.g*obj.rbtdef.a(3)*sin(q2 + q3)*cos(q3)...
                                - obj.g*obj.rbtdef.a(4)*cos(q4)*cos(q2 + q3 + q4)...
                                - obj.g*obj.rbtdef.d(5)*cos(q5)^2*cos(q2 + q3 + q4);
        end
        
        function tau_iden = GenerateIdenTorque(obj,jpos,jvel)
            tau_iden = obj.GenerateGravTau(jpos)+obj.GenerateFricTau(jvel);
        end
        
        function tau_grav = GenerateGravTau(obj,jpos)
            tau_grav = zeros(obj.njoint,1);
            reg_mat = zeros(4,16);
            reg_mat(1,:) =obj. CalcRegressorJoint2(jpos);
            reg_mat(2,:) =obj. CalcRegressorJoint3(jpos);
            reg_mat(3,:) =obj. CalcRegressorJoint4(jpos);
            reg_mat(4,:) =obj. CalcRegressorJoint5(jpos);
            tau_grav(2:5) = reg_mat*obj.barycenter_params;
        end
        
        function tau_fric = GenerateFricTau(obj,jvel)
            tau_fric = zeros(obj.njoint,1);
            for jidx=1:obj.njoint
                tau_fric(jidx) = obj.fric_params(2*jidx-1)*SignVel(jvel(jidx))...
                                        +obj.fric_params(2*jidx)*jvel(jidx);
            end
        end
        
        
    end
    
end
 
