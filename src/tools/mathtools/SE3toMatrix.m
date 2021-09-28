function mat = SE3toMatrix(pose)
    mat = eye(4);
    mat(1:3,1:3) = pose.R;
    mat(1:3,end) = pose.t;
end