function [steering_angle, speed, x_local, y_local] = ...
    global_to_local_waypoints(waypoints_x, waypoints_y, ...
                              X_ego, Y_ego, head, goal_pose)
%#codegen

    %% 1) 파라미터
    stop_threshold          = 4.0;
    heading_align_threshold = 9.0;
    v_base                  = 5.0;   % [m/s]
    v_min                   = 1.5;
    v_max                   = 10.0;
    smooth_window           = 5;     % 웨이포인트 이동평균 윈도우
    alpha_filter            = 0.7;   % 조향각 저역통과 계수 (0~1)

    %% 2) 초기화
    x_local = 0;  y_local = 0;
    steering_angle = 0;
    speed = v_base;

    %% 3) 목표점까지 거리 계산
    goal_dist = norm([X_ego - goal_pose(1), Y_ego - goal_pose(2)]);
    if goal_dist < stop_threshold
        speed = 0; steering_angle = 0; return;
    elseif goal_dist < heading_align_threshold
        heading_error   = wrapToPi(pi/2 - head);
        steering_angle = atan2(2*2.9*sin(heading_error), goal_dist + 1e-3);
        speed           = 3;
        return;
    end

    %% 4) 유효 웨이포인트 추출 및 스무딩
    valid = ~isnan(waypoints_x) & ~isnan(waypoints_y);
    x_wp0 = waypoints_x(valid);
    y_wp0 = waypoints_y(valid);
    if numel(x_wp0)<2
        speed = 0; return;
    end
    % — 이동평균 스무딩 —
    x_wp = smoothdata(x_wp0, 'movmean', smooth_window);
    y_wp = smoothdata(y_wp0, 'movmean', smooth_window);

    %% 5) 가장 가까운 포인트 로컬 좌표
    dists = hypot(x_wp - X_ego, y_wp - Y_ego);
    [~, idx_closest] = min(dists);
    dx = x_wp(idx_closest) - X_ego;
    dy = y_wp(idx_closest) - Y_ego;
    x_local = dx*cos(-head) - dy*sin(-head);
    y_local = dx*sin(-head) + dy*cos(-head);

    %% 6) Pure Pursuit 조향각 계산
    [steer_pp, ~] = pure_pursuit(x_wp, y_wp, [X_ego, Y_ego], head);

    %% 7) 조향각 저역통과 필터
    persistent last_steer
    if isempty(last_steer)
        last_steer = steer_pp;
    end
    steering_angle = alpha_filter * last_steer + (1-alpha_filter) * steer_pp;
    last_steer = steering_angle;

    %% 8) 속도 제한
    speed = max(v_min, min(v_max, v_base));
end

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

%% ===== 각도 wrap 함수 (변경 없음) =====
function angle = wrapToPi(theta)
    angle = mod(theta + pi, 2*pi) - pi;
end