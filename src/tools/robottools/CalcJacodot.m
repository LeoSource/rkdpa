%Calculate robot differential jacobian matrix
%This matrix usually used for operational control XDD = J(Q)QDD+JDOT(Q)QD
%reference: https://zhuanlan.zhihu.com/p/205342861
%article: Symbolic Differentiation of the Velocity Mapping for a Serial Kinematic Chain

function jaco_dot = CalcJacodot(robot, q, qd)

    n = robot.n;
    z0 = [0,0,1]';
    p0t = robot.fkine(q).t;
    w_tmp = zeros(3,1);
    for idx=1:n
        p0i = robot.A(1:idx,q').t;
        r0i = robot.A(1:idx,q').R;
        e{idx} = r0i*z0;
        pit{idx} = p0t-p0i;
        w_tmp = w_tmp + e{idx}*qd(idx);
        w{idx} = w_tmp;
    end
    
    for idx=1:n
        ed = cross(w{idx}, e{idx});
        if robot.links(idx).isrevolute
            jw_dot(:,idx) = ed;
            jv_tmp1 = cross(pit{idx}, ed);
            jv_tmp2 = cross(CalcVelit(robot, q, qd, idx), e{idx});
            jv_dot(:,idx) = -(jv_tmp1+jv_tmp2);
        else
            jw_dot(:,idx) = zeros(3,1);
            jv_dot(:,idx) = ed;
        end
    end
   
    jaco_dot = [jv_dot;jw_dot];
        
end


%Calculate the velocity(origin i ->origin tool) in frame0(base frame)
%p0t = p0i+pit  ====>dot(pit) = dot(p0t)-dot(p0i)
function vit = CalcVelit(robot, q, qd, nidx)
    v0t = robot.jacob0(q', 'trans')*qd;
    w = zeros(3,1);
    v = zeros(3,1);
    pre_rot = eye(3);
    for idx=1:nidx
        r0i = robot.A(1:idx,q').R;
        e0i = r0i(:,3);
        pi1_i = pre_rot*robot.A(idx,q').t;
        if robot.links(idx).isrevolute
            v = v+cross(w,pi1_i);
            w = w+e0i*qd(idx);
        else
            v = v+cross(w,pi1_i)+e0i*qd(idx);
%             w = w;
        end
        pre_rot = r0i;
    end
    vit = v0t-v;

end
