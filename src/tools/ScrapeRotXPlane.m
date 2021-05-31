function [sim_pos, sim_q] = ScrapeRotXPlane(rbt,via_pos,pitch_x,q0,dt)

    global g_cvmax g_camax g_jvmax g_jamax
    sim_pos = []; pos = [];
    sim_q = [];
    pitch = []; yaw = [];
    inc_angle = [20, 50]*pi/180;
    pitch_high = pitch_x-inc_angle(1);
    pitch_low = pitch_x-inc_angle(2);
    rbt.SetPitchRange(pitch_high, pitch_low);
    ang_vmax = [1,g_jvmax(3),g_jvmax(1)];
    ang_amax = [1,g_jamax(3),g_jamax(1)];

    %% pre-clean action
    pos0 = rbt.FKSolveTool(q0).t;
    pitch0 = q0(3)+rbt.tool_pitch;
    yaw0 = q0(1)+q0(end);
    rpy0 = [0;pitch0;yaw0];
    ctraj = CTrajPlanner(pos0,rpy0);
    
%     segpos{1} = CalcSplineTransPos([pos0,via_pos],0.05,'arc');
%     arc_traj = ArcPathPlanner(segpos{1}(:,1),via_pos(:,1),segpos{1}(:,2),'arctrans');
%     varc = sqrt(g_camax*arc_traj.radius);
%     line_traj = LineTrajPlanner(pos0,segpos{1}(:,1),g_cvmax,g_camax,[],[0,varc],...
%                                             rpy0,[0;rbt.pitch_high;0],g_jvmax(3),g_jamax(3),[],[0,0],'both');
%     arc_traj = ArcTrajPlanner(segpos{1}(:,1),via_pos(:,1),segpos{1}(:,2),g_cvmax,g_camax,[],[varc,varc],...
%                                             [0;rbt.pitch_high;0],[],[],[],[],[],'pos');                                    
%     ctraj.AddSegment(line_traj);
%     ctraj.AddSegment(arc_traj);
%     
%     posn = via_pos(:,2)+[0;-0.1;0];
%     segpos{2} = CalcSplineTransPos([via_pos, posn],0.05,'arc');
%     arc_traj = ArcPathPlanner(segpos{2}(:,1),via_pos(:,2),segpos{2}(:,2),'arctrans');
%     varc = sqrt(g_camax*arc_traj.radius);
%     line_traj = LineTrajPlanner(segpos{1}(:,2),segpos{2}(:,1),g_cvmax,g_camax,[],[varc,varc],...
%                                             [0;rbt.pitch_high;0],[0;rbt.pitch_low;0],g_jvmax(3),g_jamax(3),[],[0,0],'both');
%     arc_traj = ArcTrajPlanner(segpos{2}(:,1),via_pos(:,2),segpos{2}(:,2),g_cvmax,g_camax,[],[varc,varc],...
%                                             [0;rbt.pitch_low;0],[],[],[],[],[],'pos');
%     ctraj.AddSegment(line_traj);
%     ctraj.AddSegment(arc_traj);
%                                         
%     line_traj = LineTrajPlanner(segpos{2}(:,2),posn,g_cvmax,g_camax,[],[varc,0],...
%                                             [0;rbt.pitch_low;0],[],[],[],[],[],'pos');
%     ctraj.AddSegment(line_traj);
                                        
                                        
    line_traj = LineTrajPlanner(pos0,via_pos(:,1),g_cvmax,g_camax,[],[0,0],...
                                            rpy0,[0;rbt.pitch_high;0],g_jvmax(3),g_jamax(3),[],[0,0],'both');
    ctraj.AddSegment(line_traj);
    
    rpyn = [0;rbt.pitch_low;0];
    line_traj = LineTrajPlanner(via_pos(:,1),via_pos(:,2),g_cvmax,g_camax,[],[0,0],...
                                            [0;rbt.pitch_high;0],rpyn,g_jvmax(3),g_jamax(3),[],[0,0],'both');
    ctraj.AddSegment(line_traj);
    
    posn = via_pos(:,2)+[0;-0.1;0];
    line_traj = LineTrajPlanner(via_pos(:,2),posn,g_cvmax,g_camax,[],[0,0],...
                                            rpyn,[],[],[],[],[],'pos');
    ctraj.AddSegment(line_traj);
    [pos, rpy] = ctraj.GeneratePath(dt);

    %% robot inverse kinematics
    pre_q = q0;
    for idx=1:size(pos,2)
        tmp_q = rbt.IKSolvePitchYaw(pos(:,idx), rpy(2,idx), rpy(3,idx), pre_q);
        sim_q = [sim_q, tmp_q];
        pre_q = tmp_q;
        pose_tmp = rbt.FKSolveTool(tmp_q);
        sim_pos = [sim_pos, pose_tmp.t];
    end


end