clear
close all
clc

rbt = CleanRobot;
%% task plan
clean_task = {'mirror', 'table', 'circle', 'sphere', 'ellipsoid'};
task = 'mirror';
interp_pos = [];
switch task
    case clean_task(1)
        % wipe the mirror
        tmp_interp = [-1, 1, 1, -1]*0.5; 
        interp_pos(:,1) = [tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp];
        interp_pos(:,2) = 0.7;
        interp_pos(:,3) = [0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 0.9, 0.9, 1.0, 1.0, 1.1, 1.1, 1.2, 1.2, 1.3, 1.3, 1.4, 1.4, 1.5, 1.5, 1.6, 1.6, 1.7, 1.7, 1.8, 1.8];   
        ik_option = 'vertical';
    case clean_task(2)
        % wipe the table
        interp_pos(:,1) = [-0.5, -0.5, -0.4, -0.4, -0.3, -0.3, -0.2, -0.2, -0.1, -0.1, 0, 0, 0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.6];
        tmp_interp = [0.3, 0.8, 0.8, 0.3];
        interp_pos(:,2) = [tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp];
        interp_pos(:,3) = 0.5;
        ik_option = 'horizontal';
    case clean_task(3)
        % wipe the washbasin 
        interp_pos = circle([0,0.5,0.5], 0.3);
        interp_pos = interp_pos';
        ik_option = 'circle';
    case clean_task(4)
        %wipe the washbasin 
        for pos_z=0.5:-0.05:0.2
            interp_pos_tmp = circle([0, 0.5, pos_z], sqrt(abs(0.3^2-(pos_z-0.5)^2)));
            interp_pos = [interp_pos, interp_pos_tmp];
        end             
        interp_pos = interp_pos';
        ik_option = 'circle';
    case clean_task(5)
        %wipe the washbasin 
        ik_option = 'circle';
    otherwise
        error('CleanRobot can not accomplish the task');
end


%% trajectory plan
sample_time = 0.01;
tf = 20;
pos = [];
alpha = [];
radius_store = [];
switch task
    case clean_task(3)
        step_alpha = 2*pi*sample_time/tf;
        alpha = 0: step_alpha: 2*pi;
        pos = [0.3*sin(alpha); 0.5+0.3*cos(alpha); 0.5*ones(1,length(alpha))];
        pos = pos';        
    case clean_task(4)
        step = 1*pi/180;
        origin = [0; 0.5; 0.5];
        radius = 0.3;
        pos = [0; origin(2)+radius; origin(3)];
        interp_phi = [0:-15:-90]*pi/180;
        for idx=1:length(interp_phi)-1
            pos_z = pos(3,end);
            tmp_alpha = 0:pi/180:2*pi;
            alpha = [alpha, tmp_alpha];
            r = sqrt(radius^2-(pos_z-origin(3))^2);
            tmp_pos = [-sin(tmp_alpha)*r; origin(2)+cos(tmp_alpha)*r; pos_z*ones(1,length(tmp_alpha))];
            pos = [pos, tmp_pos];
            radius_store = [radius_store, r*ones(1,length(tmp_alpha))];

            phi = interp_phi(idx):-step:interp_phi(idx+1);
            tmp_pos = [zeros(1,length(phi)); 0.5+0.3*cos(phi); 0.5+0.3*sin(phi)];
            pos = [pos, tmp_pos];
            alpha = [alpha, zeros(1,length(phi))];
            radius_store = [radius_store, zeros(1,length(phi))];
        end
        pos = pos(:,2:end);
        pos = pos';        
    case clean_task(5)
        step = 1*pi/180;
        origin = [0; 0.5; 0.5];
        a = 0.4; b = 0.5; c = 0.3;
        pos = [0; origin(2)+b; origin(3)];
        interp_phi = [0: -15: -90]*pi/180;
        for idx=1:length(interp_phi)-1
            pos_z = pos(3,end);
            tmp_alpha = 0:step:2*pi;
            alpha = [alpha, tmp_alpha];
            rs_value = sqrt(1-(pos_z-origin(3))^2/c^2);
            a_new = a*rs_value; b_new = b*rs_value;
            tmp_pos = [-a_new*sin(alpha); b_new*cos(alpha)+origin(2); pos_z*ones(1,length(tmp_alpha))];
            pos = [pos, tmp_pos];
            
        end
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
        end           
end


sim_q = [];
sim_pos = [];
for idx=1:size(pos,1)    
    tmp_q = rbt.IKSolve(pos(idx,:), ik_option, 0);
%     circle_params.origin = [0; 0.5; 0.5];
%     circle_params.radius = radius_store(idx);
%     circle_params.alpha = alpha(idx);
%     if abs(circle_params.alpha)<eps
%         tmp_q = rbt.IKSolve(pos(idx,:), 'horizontal', 0);
%     else
%         tmp_q = rbt.IKSolve(pos(idx,:), ik_option, circle_params);
%     end
    
    sim_q = [sim_q; tmp_q];
    tmp_pose = rbt.FKSolve(tmp_q);
    sim_pos = [sim_pos; tmp_pose.t'];
end
t = [0:sample_time:sample_time*(size(sim_q,1)-1)]';
sim_q(:,2) = sim_q(:,2) - 0.75;

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



