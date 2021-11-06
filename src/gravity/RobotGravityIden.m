classdef RobotGravityIden < handle
    
    properties
        barycenter_j5 %% l5x,l5y,l6x,l6y,l6z,m6
        barycenter_j4 %% l4x,l4y,l5x,l5y,l5z,m5,l6x,l6y,l6z,m6
        barycenter_j3 %% l3x,l3y,l4x,l4y,m4,l5x,l5y,l5z,m5,l6x,l6y,l6z,m6
        barycenter_j2 %% l2x,l2y,l3x,l3y,m3,l4x,l4y,m4,l5x,l5y,l5z,m5,l6x,l6y,l6z,m6
        barycenter_params
    end
    
    
    methods
        function obj = RobotGravityIden()
            
        end
    
        function reg_vec = CalcRegressorJoint5(obj,q)
            reg_vec = zeros(1,16);
            q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
            reg_vec(9) = -9.81*sin(q5)*cos(q2+q3+q4);
            reg_vec(10) = -9.81*cos(q5)*cos(q2 + q3 + q4);
            reg_vec(13) = -9.81*sin(q5)*cos(q6)*cos(q2 + q3 + q4);
            reg_vec(14) = 9.81*sin(q5)*sin(q6)*cos(q2 + q3 + q4);
            reg_vec(15) = 9.81*cos(q5)*cos(q2 + q3 + q4);
            reg_vec(16) = 0.34335*cos(q5)*cos(q2 + q3 + q4);
        end
        
        function reg_vec = CalcRegressorJoint4(obj,q)
            reg_vec = zeros(1,16);
            q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
            reg_vec(6) = -9.81*sin(q2 + q3 + q4);
            reg_vec(7) = -9.81*cos(q2 + q3 + q4);
            reg_vec(9) = -9.81*sin(q2 + q3 + q4)*cos(q5);
            reg_vec(10) = 9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(11) = -9.81*cos(q2 + q3 + q4);
            reg_vec(12) = -0.849742200001078*cos(q2 + q3 + q4);
            reg_vec(13) = -9.81*sin(q6)*cos(q2 + q3 + q4) - 9.81*sin(q2 + q3 + q4)*cos(q5)*cos(q6);
            reg_vec(14) = 9.81*sin(q6)*sin(q2 + q3 + q4)*cos(q5) - 9.81*cos(q6)*cos(q2 + q3 + q4);
            reg_vec(15) = -9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(16) = -0.34335*sin(q5)*sin(q2 + q3 + q4) - 0.849742200001078*cos(q2 + q3 + q4);          
        end
        
        function reg_vec = CalcRegressorJoint3(obj,q)
            reg_vec = zeros(1,16);
            q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
            reg_vec(3) = -9.81*cos(q2 + q3);
            reg_vec(4) = 9.81*sin(q2 + q3);
            reg_vec(6) = -9.81*sin(q2 + q3 + q4);
            reg_vec(7) = -9.81*cos(q2 + q3 + q4);
            reg_vec(8) = -5.0031*cos(q2 + q3);
            reg_vec(9) = -9.81*sin(q2 + q3 + q4)*cos(q5);
            reg_vec(10) = 9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(11) = -9.81*cos(q2 + q3 + q4);
            reg_vec(12) = -5.0031*cos(q2 + q3) - 0.849742200001078*cos(q2 + q3 + q4);
            reg_vec(13) = -9.81*sin(q6)*cos(q2 + q3 + q4) - 9.81*sin(q2 + q3 + q4)*cos(q5)*cos(q6);
            reg_vec(14) = 9.81*sin(q6)*sin(q2 + q3 + q4)*cos(q5) - 9.81*cos(q6)*cos(q2 + q3 + q4);
            reg_vec(15) = -9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(16) = -0.34335*sin(q5)*sin(q2 + q3 + q4) - 5.0031*cos(q2 + q3) - 0.849742200001078*cos(q2 + q3 + q4);          
        end
        
        function reg_vec = CalcRegressorJoint2(obj,q)
            reg_vec = zeros(1,16);
            q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
            reg_vec(1) = -9.81*sin(q2);
            reg_vec(2) = -9.81*cos(q2);
            reg_vec(3) = -9.81*cos(q2 + q3);
            reg_vec(4) = 9.81*sin(q2 + q3);
            reg_vec(5) = 5.0031*sin(q3)*cos(q2 + q3) - 0.050031*sin(q2 + q3)*cos(q3);
            reg_vec(6) = -9.81*sin(q2 + q3 + q4);
            reg_vec(7) = -9.81*cos(q2 + q3 + q4);
            reg_vec(8) = 5.0031*sin(q3)*cos(q2 + q3) - 5.0031*sin(q4)*sin(q2 + q3 + q4)...
                            - 0.050031*sin(q2 + q3)*cos(q3) - 5.0031*cos(q4)*cos(q2 + q3 + q4);
            reg_vec(9) = -9.81*sin(q2 + q3 + q4)*cos(q5);
            reg_vec(10) = 9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(11) = -9.81*sin(q5)^2*cos(q2 + q3 + q4) - 9.81*cos(q5)^2*cos(q2 + q3 + q4);
            reg_vec(12) = 5.0031*sin(q3)*cos(q2 + q3) - 5.0031*sin(q4)*sin(q2 + q3 + q4)...
                                - 0.849742200001078*sin(q5)^2*cos(q2 + q3 + q4)...
                                - 0.050031*sin(q2 + q3)*cos(q3) - 5.0031*cos(q4)*cos(q2 + q3 + q4)...
                                - 0.849742200001078*cos(q5)^2*cos(q2 + q3 + q4);
            reg_vec(13) = -9.81*sin(q5)^2*sin(q6)*cos(q2 + q3 + q4)...
                                - 9.81*sin(q6)*cos(q5)^2*cos(q2 + q3 + q4)...
                                - 9.81*sin(q2 + q3 + q4)*cos(q5)*cos(q6);
            reg_vec(14) = -9.81*sin(q5)^2*cos(q6)*cos(q2 + q3 + q4)...
                                + 9.81*sin(q6)*sin(q2 + q3 + q4)*cos(q5)...
                                 - 9.81*cos(q5)^2*cos(q6)*cos(q2 + q3 + q4);
            reg_vec(15) = -9.81*sin(q5)*sin(q2 + q3 + q4);
            reg_vec(16) = 5.0031*sin(q3)*cos(q2 + q3) - 5.0031*sin(q4)*sin(q2 + q3 + q4)...
                                - 0.849742200001078*sin(q5)^2*cos(q2 + q3 + q4)...
                                - 0.34335*sin(q5)*sin(q2 + q3 + q4) - 0.050031*sin(q2 + q3)*cos(q3)...
                                - 5.0031*cos(q4)*cos(q2 + q3 + q4) - 0.849742200001078*cos(q5)^2*cos(q2 + q3 + q4);
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