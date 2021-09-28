function rot_err = CalcRotErr(rot_fdb, rot_cmd)

    rot_err = cross(rot_fdb(:,1), rot_cmd(:,1))...
                +cross(rot_fdb(:,2), rot_cmd(:,2))...
                +cross(rot_fdb(:,3), rot_cmd(:,3));
    rot_err = 0.5*rot_err;

end