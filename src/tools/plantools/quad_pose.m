function pose = quad_pose(pq,u)
% Inputs -----------------------------------------------------------------
%   o p:  7 x N array representing N datapoints of D dimensions.[px,py,pz,qs,qvx,qvy,qvz]
%   o u:  float scalar representing normalized path variable.
% Output -----------------------------------------------------------------
%   o pi:  7 x 1 array representing datapoints with variable u.

    np = size(pq,2);
    ulen = gen_seg_length(pq);
    uidx = discretize(u,ulen);
    s = (u-ulen(uidx))/(ulen(uidx+1)-ulen(uidx));
    
    pos = pq(1:3,:);
    quat = pq(4:7,:);
    pc1 = get_ctrl_pos(pos,uidx);
    pc2 = get_ctrl_pos(pos,uidx+1);
    qc1 = get_ctrl_quat(quat,uidx);
    qc2 = get_ctrl_quat(quat,uidx+1);
    p1 = lerp(pos(:,uidx),pos(:,uidx+1),s);
    p2 = lerp(pc1,pc2,s);
    p = lerp(p1,p2,2*s*(1-s));
    q1 = slerp(quat(:,uidx)',quat(:,uidx+1)',s);
    q2 = slerp(qc1,qc2,s);
    q = slerp(q1,q2,2*s*(1-s));
    pose = [p;q'];

end

function p = lerp(p1,p2,t)
    p = (1-t)*p1+t*p2;
end

function q = slerp(q1,q2,t)
    qr = quatmultiply(quatinv(q1),q2);
    q = quatmultiply(q1,quatexp(t*quatlog(qr)));
end

function pc = get_ctrl_pos(p,idx)
    if idx==1
        pc = p(:,1);
    elseif idx==size(p,2)
        pc = p(:,end);
    else
        pc = (6*p(:,idx)-p(:,idx-1)-p(:,idx+1))/4;
    end
end

function qc = get_ctrl_quat(q,idx)
    if idx==1
        qc = q(:,1)';
    elseif idx==size(q,2)
        qc = q(:,end)';
    else
        qtmp1 = quatmultiply(quatinv(q(:,idx-1)'),q(:,idx)');
        qtmp2 = quatmultiply(quatinv(q(:,idx)'),q(:,idx+1)');
        qtmp = quatlog(qtmp1)-quatlog(qtmp2);
        qc = quatmultiply(q(:,idx)',quatexp(qtmp/4));
    end
end

function ulen = gen_seg_length(pq)
    pos = pq(1:3,:);
    qrot = pq(4:7,:);
    np = size(pq,2);
    len = zeros(1,np);
    for idx=1:np-1
        len(idx+1) = len(idx)+pose_len(pq(:,idx),pq(:,idx+1));
    end
    ulen = len/norm(len(end));
end

function pl = pose_len(pose1,pose2)
    pdelta = norm(pose1(1:3)-pose2(1:3));
    qdelta = norm(rotvec(quaternion(pose1(4:7)').conj()*quaternion(pose2(4:7)')));
%     pl = norm([pdelta;qdelta]);
    pl = 0.1*pdelta+qdelta;
end