clear
close all
clc

sample_time =0.01;
step = 1*pi/180;
sphere_origin = [0; 0.5; 0.5];
radius = 0.3;
pos = [0; sphere_origin(2)+radius; sphere_origin(3)];
interp_phi = [0:-15:-90]*pi/180;

for idx=1:length(interp_phi)-1
    pos_z = pos(3,end);
    alpha = 0:pi/180:2*pi;
    r = sqrt(radius^2-(pos_z-sphere_origin(3))^2);
    tmp_pos = [-sin(alpha)*r; 0.5+cos(alpha)*r; pos_z*ones(1,length(alpha))];
    pos = [pos, tmp_pos];
    
    phi = interp_phi(idx):-step:interp_phi(idx+1);
    tmp_pos = [zeros(1,length(phi)); 0.5+0.3*cos(phi); 0.5+0.3*sin(phi)];
    pos = [pos, tmp_pos];
end


plot2(pos');
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');