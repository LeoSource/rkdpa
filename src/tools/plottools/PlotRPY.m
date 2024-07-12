function PlotRPY(pos_rpy,step_size,arrow_size)
% quiver3(0,0,0,1,0,0,0.5,'r','filled','LineWidth',2);
% hold on
% quiver3(0,0,0,0,1,0,1,'g','filled','LineWidth',2);
% quiver3(0,0,0,0,0,1,2,'b','filled','LineWidth',2);
% hold off
% isa(pose_tool, 'SE3')
    np = size(pos_rpy,2);
    nrot = floor(np/step_size);
    arrow_scale = arrow_size;
%     figure
    PlotRPYAxis(pos_rpy(:,1), arrow_scale);
    PlotRPYAxis(pos_rpy(:,end), arrow_scale);
    for idx=1:nrot
        rpy_tmp = pos_rpy(:,step_size*idx);
        PlotRPYAxis(rpy_tmp, arrow_scale);
    end
    hold off;
end

function PlotRPYAxis(pos_rpy,scale)
    rot_plot = rpy2r(180/pi*pos_rpy(4:6)', 'xyz');
    x = pos_rpy(1); y = pos_rpy(2); z = pos_rpy(3);
    quiver3(x,y,z,rot_plot(1,1),rot_plot(2,1),rot_plot(3,1), scale, 'r');
    hold on
    quiver3(x,y,z,rot_plot(1,2),rot_plot(2,2),rot_plot(3,2), scale, 'g');
    quiver3(x,y,z,rot_plot(1,3),rot_plot(2,3),rot_plot(3,3), scale, 'b');
end