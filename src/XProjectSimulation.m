clear
close all
clc

rbt = CleanRobot;
%% task plan
clean_task = {'mirror', 'table', 'circle', 'sphere', 'ellipsoid'};
task = 'ellipsoid';
interp_pos = [];
switch task
    case clean_task(1)
        % wipe the mirror
        tmp_interp = [-1, 1, 1, -1]*0.5; 
        interp_pos(:,1) = [tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp];
        interp_pos(:,2) = 0.7;
        interp_pos(:,3) = [0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 0.9, 0.9, 1.0, 1.0, 1.1, 1.1, 1.2, 1.2, 1.3, 1.3, 1.4, 1.4, 1.5, 1.5, 1.6, 1.6, 1.7, 1.7, 1.8, 1.8];   
        ik_option = 'q2first';
    case clean_task(2)
        % wipe the table
        interp_pos(:,1) = [-0.5, -0.5, -0.4, -0.4, -0.3, -0.3, -0.2, -0.2, -0.1, -0.1, 0, 0, 0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.6];
        tmp_interp = [0.3, 0.8, 0.8, 0.3];
        interp_pos(:,2) = [tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp];
        interp_pos(:,3) = 0.5;
        ik_option = 'q3first';
    case clean_task(3)
        % wipe the washbasin 
%         interp_pos = circle([0,0.5,0.5], 0.3);
%         interp_pos = interp_pos';
        ik_option = 'q3first';
    case clean_task(4)
        %wipe the washbasin 
        ik_option = 'q3first';
    case clean_task(5)
        %wipe the washbasin 
        ik_option = 'q3first';
    otherwise
        error('CleanRobot can not accomplish the task');
end


%% trajectory plan
sample_time = 0.01;
tf = 20;
pos = [];
alpha = [];
switch task
    case clean_task(3)%circle
        step_alpha = 2*pi*sample_time/tf;
        alpha = 0: step_alpha: 2*pi;
        origin = [0; 0.5; 0.5];
        radius = 0.3;
        pos = [0.3*sin(alpha); 0.5+0.3*cos(alpha); 0.5*ones(1,length(alpha))];
        pos = pos';        
    case clean_task(4)%sphere
        step = 1*pi/180;
        origin = [0; 0.5; 0.5];
        radius = 0.3;
        pos = [0; origin(2)+radius; origin(3)];
        interp_phi = [0:-15:-90]*pi/180;
        for idx=1:length(interp_phi)-1
            pos_z = pos(3,end);
            tmp_alpha = 0:step:2*pi;
            alpha = [alpha, tmp_alpha];
            r = sqrt(radius^2-(pos_z-origin(3))^2);
            tmp_pos = [-sin(tmp_alpha)*r; origin(2)+cos(tmp_alpha)*r; pos_z*ones(1,length(tmp_alpha))];
            pos = [pos, tmp_pos];

            phi = interp_phi(idx):-step:interp_phi(idx+1);
            tmp_pos = [zeros(1,length(phi)); 0.5+0.3*cos(phi); 0.5+0.3*sin(phi)];
            pos = [pos, tmp_pos];
            alpha = [alpha, zeros(1,length(phi))];
        end
        pos = pos(:,2:end);
        pos = pos';        
    case clean_task(5)%ellipsoid
        step = 1*pi/180;
        origin = [0; 0.5; 1];
        a = 0.4; b = 0.5; c = 0.3;
        pos = [0; origin(2)+b; origin(3)];
        interp_phi = [0: -15: -90]*pi/180;
        for idx=1:length(interp_phi)-1
            pos_z = pos(3,end);
            tmp_alpha = 0:step:2*pi;
            alpha = [alpha, tmp_alpha];
            rs_value = sqrt(abs(1-(pos_z-origin(3))^2/c^2));
            a_new = a*rs_value; b_new = b*rs_value;
            tmp_pos = [-a_new*sin(tmp_alpha); b_new*cos(tmp_alpha)+origin(2); pos_z*ones(1,length(tmp_alpha))];
            pos = [pos, tmp_pos];
            
            phi = interp_phi(idx):-step:interp_phi(idx+1);
            tmp_pos = [zeros(1,length(phi)); origin(2)+b*cos(phi); origin(3)+c*sin(phi)];
            pos = [pos, tmp_pos];
            alpha = [alpha, zeros(1,length(phi))];
        end
        pos = pos(:,2:end);
        pos = pos';           
    otherwise
        for idx=1:size(interp_pos,1)-1
           if mod(idx,2)==1
               interp_num = 100;
           else
               interp_num = 10;
           end
           initial_frame = transl(interp_pos(idx,:)');
           end_frame = transl(interp_pos(idx+1,:)');
           tmp_frame = ctraj(initial_frame, end_frame, interp_num);
           pos = [pos; transl(tmp_frame)];
           alpha = zeros(1,size(pos,1));
        end           
end

sim_q = [];
sim_pos = [];
for idx=1:size(pos,1)    
    tmp_q = rbt.IKSolve(pos(idx,:), ik_option, alpha(idx));    
    sim_q = [sim_q; tmp_q];
    tmp_pose = rbt.FKSolve(tmp_q);
    sim_pos = [sim_pos; tmp_pose.t'];
end
pos_err = pos-sim_pos;
t = [0:sample_time:sample_time*(size(sim_q,1)-1)]';
sim_q(:,2) = sim_q(:,2) - 0.75;

%% simulation with simscape and plot
sim('x_project_g3.slx');

figure(1)
plot2(pos, '--');
grid on
hold on
plot2(sim_pos, 'r');
legend('cmd\_pos', 'sim\_pos');
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
hold off

figure(2)
plot(t,sim_q(:,1),'-', t, sim_q(:,2), '--', t, sim_q(:,3), '-.', t, sim_q(:,4), ':', t, sim_q(:,5), '-');
grid on
legend('q1', 'q2', 'q3', 'q4', 'q5');

figure(3)
plot(pos_err);
grid on
legend('x', 'y', 'z');