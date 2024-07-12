function pquad = quad_pos(p,u)
% Inputs -----------------------------------------------------------------
%   o p:  D x N array representing N datapoints of D dimensions.
%   o u:  float scalar representing normalized path variable.
% Output -----------------------------------------------------------------
%   o pi:  D x 1 array representing datapoints with variable u.
    
    
    np = size(p,2);
    len = zeros(1,np);
    for idx=1:np-1
        len(idx+1) = len(idx)+norm(p(:,idx+1)-p(:,idx));
    end
    ulen = len/norm(len(end));
    uidx = discretize(u,ulen);
    s = (u-ulen(uidx))/(ulen(uidx+1)-ulen(uidx));

    pc1 = get_ctrl_pos(p,uidx);
    pc2 = get_ctrl_pos(p,uidx+1);
    p1 = lerp(p(:,uidx),p(:,uidx+1),s);
    p2 = lerp(pc1,pc2,s);
    pquad = lerp(p1,p2,2*s*(1-s));

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

function p = lerp(p1,p2,t)
    p = (1-t)*p1+t*p2;
end