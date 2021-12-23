classdef RobotGravityIden < handle
    
    properties
%         barycenter_j5 %% l5x,l5y,l6x,l6y,l6z,m6
%         barycenter_j4 %% l4x,l4y,l5x,l5y,l5z,m5,l6x,l6y,l6z,m6
%         barycenter_j3 %% l3x,l3y,l4x,l4y,m4,l5x,l5y,l5z,m5,l6x,l6y,l6z,m6
%         barycenter_j2 %% l2x,l2y,l3x,l3y,m3,l4x,l4y,m4,l5x,l5y,l5z,m5,l6x,l6y,l6z,m6
        barycenter_params
        rbtdef
        g
    end
    
    
    methods
        function obj = RobotGravityIden(rbtdef)
            obj.rbtdef = rbtdef;
            obj.g = abs(rbtdef.gravity(3));
        end
        
        function LoadGravityParams(obj,file)
            obj.barycenter_params = load(file);
        end
        
        function Identification(obj,jpos,jtau)
            [jpos_grav,jtau_grav] = obj.ProcessJointData(jpos,jtau);
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
        
        function [jpos_grav,jtau_grav] = ProcessJointData(obj,jpos,jtau)
            %%process identification data%%
            record_value = [-0.87,0.87];
            eps = 1e-5;
            start_idx = find(jpos(:,2)>record_value(1)-4*eps & jpos(:,2)<record_value(1)+5*eps);
            stop_idx = find(jpos(:,2)>record_value(2)-4*eps & jpos(:,2)<record_value(2)+4*eps);
            if length(start_idx) ~=2  || length(stop_idx) ~=2
                error('rescop1');
            end
            start_idx(2) = start_idx(2)+0;
            if abs(stop_idx(1)-start_idx(1)) ~= abs(stop_idx(2)-start_idx(2))
                error('rescope2');
            end
            jpos_grav = jpos(start_idx(1):stop_idx(1),:);
            tau1 = jtau(start_idx(1):stop_idx(1),:);
            tau2_tmp = jtau(stop_idx(2):start_idx(2),:);
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
        
        function tau_grav = GenerateGravTau(obj,jpos)
            tau_grav = zeros(6,1);
            reg_mat = zeros(4,16);
            reg_mat(1,:) =obj. CalcRegressorJoint2(jpos);
            reg_mat(2,:) =obj. CalcRegressorJoint3(jpos);
            reg_mat(3,:) =obj. CalcRegressorJoint4(jpos);
            reg_mat(4,:) =obj. CalcRegressorJoint5(jpos);
            tau_grav(2:5) = reg_mat*obj.barycenter_params;
        end
        
        function tau = CalcGravJoint5(obj,jpos)
            tau = obj.GenerateGravTau(jpos);
            tau = tau(5);
        end
        
        function tau = CalcGravJoint4(obj,jpos)
            tau = obj.GenerateGravTau(jpos);
            tau = tau(4);
        end
        
        function tau = CalcGravJoint3(obj,jpos)
            tau = obj.GenerateGravTau(jpos);
            tau = tau(3);
        end
        
        function tau = CalcGravJoint2(obj,jpos)
            tau = obj.GenerateGravTau(jpos);
            tau = tau(2);
        end
        
        
    end
    
 end