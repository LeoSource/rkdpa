clear
close all
clc

rbt = CleanRobot;
%% trajector plan
clean_task = {'mirror', 'table', 'washbasin'};
task = 'washbasin';
% wipe the mirror
if strcmp(task,clean_task(1))
    tmp_interp = [-1, 1, 1, -1]*0.5; 
    interp_pos(:,1) = [tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp];
    interp_pos(:,2) = 0.7;
    interp_pos(:,3) = [0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 0.9, 0.9, 1.0, 1.0, 1.1, 1.1, 1.2, 1.2, 1.3, 1.3, 1.4, 1.4, 1.5, 1.5, 1.6, 1.6, 1.7, 1.7, 1.8, 1.8];   
    ik_option = 'vertical';
elseif strcmp(task, clean_task(2))    
    % wipe the table
    interp_pos(:,1) = [-0.5, -0.5, -0.4, -0.4, -0.3, -0.3, -0.2, -0.2, -0.1, -0.1, 0, 0, 0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.6];
    tmp_interp = [0.3, 0.8, 0.8, 0.3];
    interp_pos(:,2) = [tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp, tmp_interp];
    interp_pos(:,3) = 0.5;
    ik_option = 'horizontal';
elseif strcmp(task, clean_task(3))
    % wipe the washbasin 
    interp_pos = circle([0,0.5,0.5], 0.3);
    interp_pos = interp_pos';
    ik_option = 'circle';
else
    error('CleanRobot can not accomplish the task');
end

sample_time = 0.01;
tf = 20;
sim_cart_pos = [];
alpha = [];
if strcmp(task, clean_task(3))
    for t=0:sample_time:tf
        tmp_alpha = 2*pi/tf*t;
        alpha = [alpha, tmp_alpha];
        tmp_cart_pos = [0.3*sin(tmp_alpha), 0.5+0.3*cos(tmp_alpha), 0.5];
        sim_cart_pos = [sim_cart_pos; tmp_cart_pos];
    end
else
    for idx=1:size(interp_pos,1)-1
        if mod(idx,2)==1
            interp_num = 100;
        else
            interp_num = 10;
        end
        initial_frame = transl(interp_pos(idx,:)');
        end_frame = transl(interp_pos(idx+1,:)');
        tmp_frame = ctraj(initial_frame, end_frame, interp_num);
        sim_cart_pos = [sim_cart_pos; transl(tmp_frame)];
    end    
end

height_limit = rbt.arm.links(2).qlim;
sim_q = [];
for idx=1:size(sim_cart_pos,1)
    pitch = 0;
    if sim_cart_pos(idx,3)<height_limit(1)
        pitch = atan((sim_cart_pos(idx,3) - height_limit(1)) / sim_cart_pos(idx,2));
    elseif sim_cart_pos(idx,3)>height_limit(2)
        pitch = atan((sim_cart_pos(idx,3) - height_limit(2)) / sim_cart_pos(idx,2));
    else
        pitch = 0;
    end    
    
    tmp_frame1 = transl(sim_cart_pos(idx,:)');
    tmp_frame2 = SE3.Rx(pitch*180/pi);
    tmp_frame = SE3(tmp_frame1)*tmp_frame2;
%     tmp_q = rbt.IKSolveSimple(tmp_frame.t, ik_option, 0);
    tmp_q = rbt.IKSolveSimple(tmp_frame.t, ik_option, alpha(idx));
%     there is problem when using ikine method under the 4 or 5 DOF
%     tmp_q = rbt.ikine(tmp_frame, 'mask', [1, 1, 1, 1, 0, 1], 'quiet');
    sim_q = [sim_q; tmp_q];
end
t = [0:sample_time:sample_time*(size(sim_q,1)-1)]';
sim_q(:,2) = sim_q(:,2) - 0.75;

sim('x_project_g3.slx');

figure(1)
plot2(interp_pos, '--');
grid on
hold on
plot2(sim_cart_pos, 'r');
legend('interp\_pos', 'trajectory');
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
hold off

figure(2)
plot(t,sim_q(:,1),'-', t, sim_q(:,2), '--', t, sim_q(:,3), '-.', t, sim_q(:,4), ':', t, sim_q(:,5), '-');
grid on
legend('q1', 'q2', 'q3', 'q4', 'q5');



