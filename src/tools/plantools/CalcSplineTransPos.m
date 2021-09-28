function ctrlpos = CalcSplineTransPos(via_pos, zone_radius, option)

    if strcmp(option, 'spline')
        ctrlpos = zeros(3,5);
        ctrlpos(:,3) = via_pos(:,2);

        line_len(1) = norm(via_pos(:,2)-via_pos(:,1));
        line_len(2) = norm(via_pos(:,3)-via_pos(:,2));
        ctrlpos(:,1) = via_pos(:,2)+zone_radius/line_len(1)*(via_pos(:,1)-via_pos(:,2));
        ctrlpos(:,5) = via_pos(:,2)+zone_radius/line_len(2)*(via_pos(:,3)-via_pos(:,2));

        ratio = 0.5;
        ctrlpos(:,2) = ctrlpos(:,3)+ratio*(ctrlpos(:,1)-ctrlpos(:,3));
        ctrlpos(:,4) = ctrlpos(:,3)+ratio*(ctrlpos(:,5)-ctrlpos(:,3));
    elseif strcmp(option, 'arc')
        ctrlpos = zeros(3,2);
        line_len(1) = norm(via_pos(:,2)-via_pos(:,1));
        line_len(2) = norm(via_pos(:,3)-via_pos(:,2));
        ctrlpos(:,1) = via_pos(:,2)+zone_radius/line_len(1)*(via_pos(:,1)-via_pos(:,2));
        ctrlpos(:,2) = via_pos(:,2)+zone_radius/line_len(2)*(via_pos(:,3)-via_pos(:,2));
    end

end