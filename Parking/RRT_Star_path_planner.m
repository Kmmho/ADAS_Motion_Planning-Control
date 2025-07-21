function [x_path, y_path] = rrt_path_planner(obstacle_info, start_point, goal_point)
%#codegen
    persistent success_cached x_path_cached y_path_cached
    MAX_PATH_LEN = 500;
    
    %--- 캐시 초기화 및 반환 로직 ---
    if isempty(success_cached)
        % 처음 호출될 때
        success_cached  = false;
        x_path_cached   = zeros(MAX_PATH_LEN,1);
        y_path_cached   = zeros(MAX_PATH_LEN,1);
    elseif success_cached
        % 이미 성공적으로 계산된 경로가 있으면 바로 반환
        x_path = x_path_cached;
        y_path = y_path_cached;
        return;
    end

    %%RRT*
    MAX_NODES    = 2000;
    goal_radius  = 3.0;
    goal_sample_rate = 0.15;
    eta          = 2.0;

    start = [start_point(1), start_point(2)];
    finish_point  = [goal_point(1),  goal_point(2)];
    vehicle_h_w = [1.47, 3.1];
    %vehicle_h_w = [2.1, 4.6];
    map_boundary = [5.5, -4, 47.5, -3.9, 47.5, -44.9, 5.5, -44.9];
    x_vals = map_boundary(1:2:end);
    y_vals = map_boundary(2:2:end);
    x_min = min(x_vals); x_max = max(x_vals);
    y_min = min(y_vals); y_max = max(y_vals);

    % 장애물 정보 파싱 (5개씩)
    if mod(length(obstacle_info),5) ~= 0
        obstacles = zeros(0,5);
    else
        num_obj = int32(floor(length(obstacle_info)/5));
        data = reshape(obstacle_info,[5, num_obj])';
        obstacles = [data(:,1), data(:,2), data(:,5), data(:,4), data(:,3)];
    end

    empty_node = struct('pos',zeros(1,2),'cost',0,'parent',int32(0));
    nodes = repmat(empty_node, MAX_NODES, 1);
    nodes(1).pos = start;
    node_count = int32(1);
    goal_node  = int32(0);

    % 샘플링 반복
    for iter = 1:2000
        if rand < goal_sample_rate
            sample = finish_point;
        else
            sample = [x_min + rand*(x_max-x_min), y_min + rand*(y_max-y_min)];
        end
        % 최근접 노드 찾기
        nearest_idx = int32(1);
        min_dist = norm(nodes(1).pos - sample);
        for i = 2:node_count
            d = norm(nodes(i).pos - sample);
            if d < min_dist
                min_dist = d;
                nearest_idx = int32(i);
            end
        end
        % 새 노드 생성
        dir_vec = sample - nodes(nearest_idx).pos;
        dist = norm(dir_vec);
        if dist > eta
            dir_vec = dir_vec / dist * eta;
        end
        new_pos = nodes(nearest_idx).pos + dir_vec;
        % 충돌 체크
        if check_collision(nodes(nearest_idx).pos, new_pos, vehicle_h_w, obstacles)
            continue;
        end
        % 노드 추가
        node_count = node_count + 1;
        if node_count > MAX_NODES, break; end
        nodes(node_count).pos    = new_pos;
        nodes(node_count).cost   = nodes(nearest_idx).cost + dist;
        nodes(node_count).parent = nearest_idx;
        % 목표 반경 체크
        if norm(new_pos - finish_point) < goal_radius
            goal_node = node_count;
            break;
        end
    end

    if goal_node > 0
        idx = goal_node; i = 0;
        temp_path = zeros(MAX_PATH_LEN,2);
        while idx>0 && i<MAX_PATH_LEN
            i = i + 1;
            temp_path(i,:) = nodes(idx).pos;
            idx = nodes(idx).parent;
        end
        flipped = flipud(temp_path(1:i,:));
        pre_steps  = 10; post_steps = 8; step_dist = 0.5;
        last_pos = flipped(end,:);
        if last_pos(1) < finish_point(1)
            hybrid_x = finish_point(1) - 0.9;
        else
            hybrid_x = finish_point(1) + 0.9;
        end
        if finish_point(2) > -36.5
            pre_path  = [ repmat(hybrid_x,pre_steps,1), finish_point(2)-step_dist*(pre_steps:-1:1)'];
            post_path = [ repmat(hybrid_x,post_steps,1), finish_point(2)+step_dist*(1:post_steps)' ];
        else
            pre_path  = [ repmat(hybrid_x,pre_steps,1), finish_point(2)+step_dist*(pre_steps:-1:1)'];
            post_path = [ repmat(hybrid_x,post_steps,1), finish_point(2)-step_dist*(1:post_steps)' ];
        end
        hybrid_path = [pre_path; finish_point; post_path];

        raw_path   = [flipped; hybrid_path];
        path_interp = smooth_path(raw_path, 0.5);
        path_interp = smooth_moving_average(path_interp, 20);

        % 최종 경로
        path_len = size(path_interp,1);
        path_out = zeros(MAX_PATH_LEN,2);
        path_out(1:path_len,:) = path_interp;

        % x,y 분리 및 캐싱
        x_path = path_out(:,1);
        y_path = path_out(:,2);
        x_path_cached  = x_path;
        y_path_cached  = y_path;
        success_cached = true;
    else
        % 실패 시 빈 벡터
        x_path = zeros(MAX_PATH_LEN,1);
        y_path = zeros(MAX_PATH_LEN,1);
    end
end

%% ===== 보조 함수
function collision = check_collision(p1, p2, vehicle_size, obs_list)
    collision = false;
    steps = 10;
    exception_obs = [26.5, -4, 0.1, 42.0, 0];
    for t = linspace(0,1,steps)
        pt = (1-t)*p1 + t*p2;
        yaw = atan2(p2(2)-p1(2), p2(1)-p1(1));
        veh = get_bbox(pt, yaw, vehicle_size);
        for i = 1:size(obs_list,1)
            if check_single_collision(veh, obs_list(i,:))
                collision = true; return;
            end
        end
        if check_single_collision(veh, exception_obs)
            collision = true; return;
        end
    end
end

function smoothed = smooth_moving_average(path, windowSize)
    N = size(path,1);
    half = floor(windowSize/2);
    smoothed = zeros(N,2);

    for i = 1:N
        if i == 1 || i == N
            smoothed(i,:) = path(i,:);  % 시작점과 끝점은 고정
        else
            i0 = max(1, i-half);
            i1 = min(N, i+half);
            smoothed(i,:) = mean(path(i0:i1,:), 1);
        end
    end
end

function is_collide = check_single_collision(veh_box, obs)
    yaw_obs = obs(5);
    rear_center = obs(1:2);
    length_obs = obs(4);
    width_obs = obs(3);
    fwd_vec = [cos(yaw_obs), sin(yaw_obs)];
    center = rear_center + (length_obs/2) * fwd_vec;
    obs_box = get_bbox(center, yaw_obs, [width_obs, length_obs]);
    is_collide = box_overlap(veh_box, obs_box);
end

function box = get_bbox(center, yaw, size)
    w = size(1); h = size(2);
    dx = [ w/2, -w/2, -w/2,  w/2];
    dy = [ h/2,  h/2, -h/2, -h/2];
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    temp = R * [dx; dy];
    box = temp + center';
end

function overlap = box_overlap(b1, b2)
    overlap = true;
    axes = get_axes(b1, b2);
    for i = 1:size(axes,2)
        proj1 = axes(:,i)' * b1;
        proj2 = axes(:,i)' * b2;
        if max(proj1) < min(proj2) || max(proj2) < min(proj1)
            overlap = false;
            return;
        end
    end
end

function axes = get_axes(b1, b2)
    edges = [b1(:,2)-b1(:,1), b1(:,3)-b1(:,2)];
    normals = [-edges(2,:); edges(1,:)];
    axes = normals ./ vecnorm(normals);
end

function path_smooth = smooth_path(path_raw, step_size)
    if size(path_raw, 1) < 2
        path_smooth = path_raw;
        return;
    end
    dist = [0; cumsum(vecnorm(diff(path_raw), 2, 2))];
    query = 0:step_size:dist(end);
    if query(end) < dist(end)
        query(end+1) = dist(end);
    end
    x_interp = spline(dist, path_raw(:,1), query);
    y_interp = spline(dist, path_raw(:,2), query);
    path_smooth = [x_interp', y_interp'];
end
