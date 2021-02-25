clear
close all
clc

%% build robot model via robotic toolbox
% mdh parameters: theta d a alpha sigma offset
mdh_table = [      0,   0,   0,       0,    0,   0
                        pi/2,   0,   0,       0,    1,   0 
                            0,    0,   0,   pi/2,    0,   pi/2
                        pi/2,    0,   0,   pi/2,   1,   0
                            0,    0,   0,    pi/2,   0,   0];
                    
rbt = SerialLink(mdh_table,'modified','name','CleanRobot');
rbt.links(1).qlim = [-pi/2, pi/2];
rbt.links(2).qlim = [0.5, 1.5];
rbt.links(3).qlim = [-pi/2, pi/2];
rbt.links(4).qlim = [0.2, 1];
rbt.links(5).qlim = [-2*pi, 2*pi];

%% validation for robot model by simscape
%%there is lag in Simulink_PS Converter block because of the filter
%{
sample_time = 0.001;
tf = 10;
t = [0:sample_time:tf]';
q(:,1) = pi/2*sin(2*pi*0.5*t);
q(:,2) = 0.5*sin(2*pi*0.5*t);
q(:,3) = pi/2*sin(2*pi*0.5*t);
q(:,4) =  0.5*sin(2*pi*0.5*t);
q(:,5) = pi/2*sin(2*pi*0.5*t);
q_offset = [0, 0.75, 0, 0, 0];
cart_pos = [];
for idx=1:size(t,1)
    q_in = q(idx,:) + q_offset;
    tmp_frame = rbt.fkine(q_in);
    end_rot = [tmp_frame.n, tmp_frame.o, tmp_frame.a];
    cart_pos = [cart_pos, tmp_frame.t + end_rot*[0, 0.2 ,0]'];
end
sim('x_project_g3');
sim_pos(:,1) = sim_pos_x;
sim_pos(:,2) = sim_pos_y;
sim_pos(:,3) = sim_pos_z;
%figure
figure(1)
subplot(2,1,1)
plot(t,sim_pos_x ,t, cart_pos(1,:),'--');
xlabel('time(s)');
ylabel('pos_x(m)');
legend('simscape model', 'mdh model');
grid on
subplot(2,1,2)
plot(t, sim_pos_x - cart_pos(1,:)');
legend('position error');

figure(2)
subplot(2,1,1)
plot(t,sim_pos_y ,t, cart_pos(2,:),'--');
xlabel('time(s)');
ylabel('pos_y(m)');
legend('simscape model', 'mdh model');
grid on
subplot(2,1,2)
plot(t, sim_pos_y - cart_pos(2,:)');
legend('position error');

figure(3)
subplot(2,1,1)
plot(t,sim_pos_z ,t, cart_pos(3,:),'--');
xlabel('time(s)');
ylabel('pos_z(m)');
legend('simscape model', 'mdh model');
grid on
subplot(2,1,2)
plot(t, sim_pos_z - cart_pos(3,:)');
legend('position error');
%}

%% trajector plan
clean_task = {'mirror', 'table', 'washbasin'};
task = 'table';
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
else
    error('CleanRobot can not accomplish the task');
end

sim_cart_pos = [];
sim_q = [];
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

height_limit = rbt.links(2).qlim;
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
    tmp_q = IKSolve(tmp_frame, ik_option);
%     there is problem when using ikine method under the 4 or 5 DOF
%     tmp_q = rbt.ikine(tmp_frame, 'mask', [1, 1, 1, 1, 0, 1], 'quiet');
    sim_q = [sim_q; tmp_q];
end
sample_time = 0.01;
t = [0:sample_time:sample_time*(size(sim_q,1)-1)]';
sim_q(:,2) = sim_q(:,2) - 0.75;

sim('x_project_g3.slx');

figure(1)
plot2(interp_pos,'r');
grid on
hold on
plot2(sim_cart_pos,'--');
legend('interp\_pos', 'trajectory');
hold off

figure(2)
plot(t,sim_q(:,1),'-', t, sim_q(:,2), '--', t, sim_q(:,3), '-.', t, sim_q(:,4), ':', t, sim_q(:,5), '-');
grid on
legend('q1', 'q2', 'q3', 'q4', 'q5');



