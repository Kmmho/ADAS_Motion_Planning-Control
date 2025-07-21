%% ===== Pure Pursuit 함수 (변경 없음) =====
function [steering_angle, start_idx] = pure_pursuit(waypoints_x, waypoints_y, vehicle_position, head)
    persistent last_target_idx;
    if isempty(last_target_idx), last_target_idx = 1; end

    WHEEL_BASE = 20.0;
    lookahead_dist = 5.5;
    num_points = length(waypoints_x);

    x_ego = vehicle_position(1);
    y_ego = vehicle_position(2);
    local_waypoints = zeros(num_points, 2);

    for i = 1:num_points
        dx = waypoints_x(i) - x_ego;
        dy = waypoints_y(i) - y_ego;
        local_waypoints(i,:) = [ dx*cos(-head)-dy*sin(-head), ...
                                 dx*sin(-head)+dy*cos(-head) ];
    end

    d2 = hypot(waypoints_x - x_ego, waypoints_y - y_ego);
    [~, closest_idx] = min(d2);
    start_idx = max(last_target_idx, closest_idx);

    target_idx = -1;
    for i = start_idx:num_points
        if norm(local_waypoints(i,:))>lookahead_dist && local_waypoints(i,1)>0
            target_idx = i; break;
        end
    end
    if target_idx<0
        for i = 1:start_idx-1
            if norm(local_waypoints(i,:))>lookahead_dist && local_waypoints(i,1)>0
                target_idx = i; break;
            end
        end
    end
    if target_idx<0
        steering_angle = 0; return;
    end

    pt = local_waypoints(target_idx,:);
    alpha = wrapToPi(atan2(pt(2),pt(1)));
    ld = min(max(norm(pt),2.0),20.0);
    steering_angle = atan2(2*WHEEL_BASE*sin(alpha), ld);

    last_target_idx = target_idx;
end