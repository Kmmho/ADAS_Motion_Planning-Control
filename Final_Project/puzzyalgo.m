

% safe_margin = 2.0;        % 차량-장애물 최소 거리
% 장애물 파라미터
obstacle_length = 4;
obstacle_width = 2;

s_obs1_init = 10;      % 차량 경로 시작점과 유사한 위치에서 시작
s_obs2_init = 1000;        % 출발점 s좌표 (둘 다 같게 or 다르게)
s_obs3_init = 100;

v_obs1 = 60.0;        % 속도 (둘 다 같게 or 다르게)
v_obs2 = 50.0;        % 속도 (둘 다 같게 or 다르게)
v_obs3 = 40.0;

d_obs1 = 0;         % 중앙 차선
d_obs2 = 6; % 왼쪽 차선 (+3)
d_obs3 = -6;
yaw_obs_prev = 0;
%% 


% 차량 시각화 파라미터
vehicle_length = 4;
vehicle_width = 2;
LANE_WIDTH = 3;

x_path = route2_waypoint_raw(:,1);
y_path = route2_waypoint_raw(:,2);

s_ref = zeros(size(x_path));
for i = 2:length(x_path)
    dx = x_path(i) - x_path(i-1);
    dy = y_path(i) - y_path(i-1);
    s_ref(i) = s_ref(i-1) + sqrt(dx^2 + dy^2);
end

% 비용 함수 가중치
K_J = 0.1; K_T = 0.1; K_D = 1.0; K_V = 1.0; K_LAT = 1.0; K_LON = 1.0;

figure;
grid on; axis equal;
xlabel('X 좌표'); ylabel('Y 좌표');
title('Frenet 기반 궤적 후보 + 최적 경로 선택');
set(gca, 'FontSize', 12);
hold on;
% 거시적 관점, 인피니트 관점 코스트,
plot(route1_waypoint_raw(:,1), route1_waypoint_raw(:,2), 'b-', 'LineWidth', 1.5);
plot(route2_waypoint_raw(:,1), route2_waypoint_raw(:,2), 'r-', 'LineWidth', 2);
plot(route3_waypoint_raw(:,1), route3_waypoint_raw(:,2), 'y-', 'LineWidth', 1.5);

car_plot = []; dot_plot = []; traj_plots = []; best_traj_plot = [];

for i = 1:3:length(x_path)
    cx = x_path(i);
    cy = y_path(i);
    T = 2;
    t_sample = linspace(0, T, 60);


     % ==== 장애물 위치 계산 및 plot ====
    t_now = (i-1)/3 * 0.05; %현재 시간

    s_obs1 = s_obs1_init + v_obs1 * t_now;
    s_obs2 = s_obs2_init + v_obs2 * t_now;
    s_obs3 = s_obs3_init + v_obs3 * t_now;

    [x_obs1, y_obs1, yaw_obs1] = get_cartesian(s_obs1, d_obs1, x_path, y_path, s_ref);
    [x_obs2, y_obs2, yaw_obs2] = get_cartesian(s_obs2, d_obs2, x_path, y_path, s_ref);
    [x_obs3, y_obs3, yaw_obs3] = get_cartesian(s_obs3, d_obs3, x_path, y_path, s_ref);
    % fprintf("yaw_obs1 = %.2f\n", yaw_obs1);
     % 중앙 차선 장애물 사각형
    R1 = [cos(yaw_obs1), -sin(yaw_obs1); sin(yaw_obs1), cos(yaw_obs1)];
    rect1 = [-obstacle_length/2, obstacle_length/2, obstacle_length/2, -obstacle_length/2;
             -obstacle_width/2, -obstacle_width/2, obstacle_width/2, obstacle_width/2];
    rect_rot1 = R1 * rect1;
    rect_rot1(1,:) = rect_rot1(1,:) + x_obs1;
    rect_rot1(2,:) = rect_rot1(2,:) + y_obs1;
    
    % 왼쪽 차선 장애물 사각형
    R2 = [cos(yaw_obs2), -sin(yaw_obs2); sin(yaw_obs2), cos(yaw_obs2)];
    rect2 = [-obstacle_length/2, obstacle_length/2, obstacle_length/2, -obstacle_length/2;
             -obstacle_width/2, -obstacle_width/2, obstacle_width/2, obstacle_width/2];
    rect_rot2 = R2 * rect2;
    rect_rot2(1,:) = rect_rot2(1,:) + x_obs2;
    rect_rot2(2,:) = rect_rot2(2,:) + y_obs2;

    R3 = [cos(yaw_obs3), -sin(yaw_obs3); sin(yaw_obs3), cos(yaw_obs3)];
    rect3 = [-obstacle_length/2, obstacle_length/2, obstacle_length/2, -obstacle_length/2;
             -obstacle_width/2, -obstacle_width/2, obstacle_width/2, obstacle_width/2];
    rect_rot3 = R3 * rect3;
    rect_rot3(1,:) = rect_rot3(1,:) + x_obs3;
    rect_rot3(2,:) = rect_rot3(2,:) + y_obs3;
    
    % 이전 장애물 plot 삭제
    if exist('obstacle_plot1','var')
        if ishandle(obstacle_plot1)
            delete(obstacle_plot1);
        end
    end
    if exist('obstacle_plot2','var')
        if ishandle(obstacle_plot2)
            delete(obstacle_plot2);
        end
    end
    if exist('obstacle_plot3','var')
        if ishandle(obstacle_plot3)
            delete(obstacle_plot3);
        end
    end
    if exist('pred_plot1','var')
        if ishandle(pred_plot1)
            delete(pred_plot1);
        end
    end
    if exist('pred_plot2','var')
        if ishandle(pred_plot2)
            delete(pred_plot2);
        end
    end
    if exist('pred_plot3','var')
        if ishandle(pred_plot3)
            delete(pred_plot3);
        end
    end
    % 장애물 시각화 (색깔 다르게)
    obstacle_plot1 = fill(rect_rot1(1,:), rect_rot1(2,:), 'blue');
    obstacle_plot2 = fill(rect_rot2(1,:), rect_rot2(2,:), 'magenta');
    obstacle_plot3 = fill(rect_rot3(1,:), rect_rot3(2,:), 'black');

    % === 장애물 1 예측 궤적 (CV 모델) ===
    vx_obs1 = v_obs1 * cos(yaw_obs1);  % 현재 진행방향 기준 속도 벡터
    vy_obs1 = v_obs1 * sin(yaw_obs1);
    % 3. CV 예측 (x, y = x0 + vx * t, y0 + vy * t)
    x_cv_pred1 = x_obs1 + vx_obs1 * t_sample;
    y_cv_pred1 = y_obs1 + vy_obs1 * t_sample;
    
    pred_plot1=plot(x_cv_pred1, y_cv_pred1, 'k--', 'LineWidth', 2);
    % 2번 장애물 예측궤적
    vx_obs2 = v_obs2 * cos(yaw_obs2);  % 현재 진행방향 기준 속도 벡터
    vy_obs2 = v_obs2 * sin(yaw_obs2);
    % 3. CV 예측 (x, y = x0 + vx * t, y0 + vy * t)
    x_cv_pred2 = x_obs2 + vx_obs2 * t_sample;
    y_cv_pred2 = y_obs2 + vy_obs2 * t_sample;
    
    pred_plot2=plot(x_cv_pred2, y_cv_pred2, 'k--', 'LineWidth', 2);

    
    % 예측 시간축
    dt = t_sample(2) - t_sample(1);
    N = length(t_sample);
    
    % 상태 변수 저장

    yaw_obs_now = yaw_obs1;
    yaw_rate_obs = (yaw_obs_now - yaw_obs_prev) / dt;
    fprintf("yaw_rate_obs : %.2f\n", yaw_rate_obs);
    yaw_obs_prev = yaw_obs_now;  % 다음 프레임에 갱신
    x_ctrv = zeros(1, N+1);
    x_ctrv(1) = x_obs1;
    
    y_ctrv = zeros(1, N+1);
    y_ctrv(1) = y_obs1;
    yaw_ctrv = zeros(1, N+1);
    yaw_ctrv(1) = yaw_obs1;
    % yaw_rate_damped = yaw_rate_obs * exp(-3 * dt);
    for k = 1:N
        t_k = k * dt;
        yaw_rate_damped = yaw_rate_obs * exp(-5 * t_k);

        if abs(yaw_rate_damped) > 1  % yaw_rate가 충분히 크면
            x_ctrv(k+1) = x_ctrv(k) + v_obs1/yaw_rate_damped * (sin(yaw_ctrv(k) + yaw_rate_damped*dt) - sin(yaw_ctrv(k)));
            y_ctrv(k+1) = y_ctrv(k) + v_obs1/yaw_rate_damped * (-cos(yaw_ctrv(k) + yaw_rate_damped*dt) + cos(yaw_ctrv(k)));
        else
            % yaw_rate가 거의 0이면 CV 모델로 근사
            x_ctrv(k+1) = x_ctrv(k) + v_obs1 * cos(yaw_ctrv(k)) * dt;
            y_ctrv(k+1) = y_ctrv(k) + v_obs1 * sin(yaw_ctrv(k)) * dt;
        end
        yaw_ctrv(k+1) = yaw_ctrv(k) + yaw_rate_damped * dt;
    end
    
    pred_plot3 = plot(x_ctrv, y_ctrv, 'b--', 'LineWidth', 2);
    
    % 이전 궤적/차량 삭제
    if exist('traj_plots', 'var')
        for p = traj_plots
            if ishandle(p)
                delete(p);
            end
        end
        traj_plots = [];
    end
    if exist('best_traj_plot', 'var') & ishandle(best_traj_plot)
        delete(best_traj_plot);
    end
    if exist('car_plot', 'var') & ishandle(car_plot)
        delete(car_plot);
    end
    if exist('dot_plot', 'var') & ishandle(dot_plot)
        delete(dot_plot);
    end

    [s0, d0] = get_frenet(cx, cy, x_path, y_path);
    [~, ~, yaw] = get_cartesian(s0, d0, x_path, y_path, s_ref);
    % fprintf("yaw : %.2f\n", yaw);
    lateral_offsets = linspace(-2*LANE_WIDTH, 2*LANE_WIDTH, 11);


    % s(t): 종방향 quartic
    s0_d = 6.0; s0_dd = 0.0;
    sT_d = 6.0; sT_dd = 0.0;
    A_s = [3*T^2, 4*T^3; 6*T, 12*T^2];
    b_s = [sT_d - (s0_d + s0_dd*T); sT_dd - s0_dd];
    x_s = A_s \ b_s;
    a0_s = s0; a1_s = s0_d; a2_s = 0.5*s0_dd;
    a3_s = x_s(1); a4_s = x_s(2);
    s_target = a0_s + a1_s*t_sample + a2_s*t_sample.^2 + a3_s*t_sample.^3 + a4_s*t_sample.^4;
    s_jerk = 6*a3_s + 24*a4_s*t_sample;

    best_cost = inf;
    best_traj = [];

    for d = lateral_offsets
        % d(t): 횡방향 quintic
        di = d0; di_d = 0; di_dd = 0;
        df = d;  df_d = 0; df_dd = 0;

        A_d = [T^3, T^4, T^5; 3*T^2, 4*T^3, 5*T^4; 6*T, 12*T^2, 20*T^3];
        b_d = [df - (di + di_d*T + 0.5*di_dd*T^2);
               df_d - (di_d + di_dd*T);
               df_dd - di_dd];
        x_d = A_d \ b_d;
        a0_d = di; a1_d = di_d; a2_d = 0.5*di_dd;
        a3_d = x_d(1); a4_d = x_d(2); a5_d = x_d(3);
        d_target = a0_d + a1_d*t_sample + a2_d*t_sample.^2 + ...
                   a3_d*t_sample.^3 + a4_d*t_sample.^4 + a5_d*t_sample.^5;
        d_jerk = 6*a3_d + 24*a4_d*t_sample + 60*a5_d*t_sample.^2;

        % 비용 계산
        J_lat = sum(d_jerk.^2);
        J_lon = sum(s_jerk.^2);
        d_diff = (d_target(end) - df)^2;
        v_diff = (sT_d - s0_d)^2;
        cost = K_LAT*(K_J*J_lat + K_T*T + K_D*d_diff) + ...
               K_LON*(K_J*J_lon + K_T*T + K_V*v_diff);

        % Frenet → Global 변환
        x_traj = zeros(size(s_target)); y_traj = zeros(size(s_target));
        for k = 1:length(s_target)
            [x_traj(k), y_traj(k), ~] = get_cartesian(s_target(k), d_target(k), x_path, y_path, s_ref);
        end

        traj_plots(end+1) = plot(x_traj, y_traj, '--', 'Color', [0.6 0.6 0.6]);

        if cost < best_cost
            best_cost = cost;
            best_traj.x = x_traj;
            best_traj.y = y_traj;
        end
    end

    best_traj_plot = plot(best_traj.x, best_traj.y, 'b-', 'LineWidth', 2.5);

    % 차량 시각화
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    rect = [-vehicle_length/2, vehicle_length/2, vehicle_length/2, -vehicle_length/2;
            -vehicle_width/2, -vehicle_width/2, vehicle_width/2, vehicle_width/2];
    rect_rotated = R * rect;
    rect_rotated(1,:) = rect_rotated(1,:) + cx;
    rect_rotated(2,:) = rect_rotated(2,:) + cy;

    car_plot = fill(rect_rotated(1,:), rect_rotated(2,:), 'cyan');
    dot_plot = plot(cx, cy, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');

    pause(0.05);
end

%% === 보조 함수들 ===

function [s, d] = get_frenet(x, y, x_ref, y_ref)
    idx = get_closest_waypoint(x, y, x_ref, y_ref);
    if idx == 1
        prev_idx = 1; next_idx = 2;
    elseif idx == length(x_ref)
        prev_idx = length(x_ref) - 1; next_idx = length(x_ref);
    else
        prev_idx = idx - 1; next_idx = idx;
    end
    vec_path = [x_ref(next_idx) - x_ref(prev_idx); y_ref(next_idx) - y_ref(prev_idx)];
    vec_ego  = [x - x_ref(prev_idx); y - y_ref(prev_idx)];
    proj = dot(vec_path, vec_ego) / norm(vec_path)^2 * vec_path;
    perp = vec_ego - proj;
    d = norm(perp);
    if det([vec_path, vec_ego]) < 0
        d = -d;
    end
    s = 0;
    for i = 1:(prev_idx - 1)
        s = s + norm([x_ref(i+1) - x_ref(i), y_ref(i+1) - y_ref(i)]);
    end
    s = s + norm(proj);
end

function [x, y, heading] = get_cartesian(s, d, x_ref, y_ref, s_ref)
    s = mod(s, s_ref(end));
    idx = find(s_ref <= s, 1, 'last');
    if idx >= length(s_ref), idx = length(s_ref) - 1; end
    dx = x_ref(idx+1) - x_ref(idx);
    dy = y_ref(idx+1) - y_ref(idx);
    heading = atan2(dy, dx);
    ds = s - s_ref(idx);
    x_base = x_ref(idx) + ds * cos(heading);
    y_base = y_ref(idx) + ds * sin(heading);
    x = x_base + d * cos(heading + pi/2);
    y = y_base + d * sin(heading + pi/2);
end

function idx = get_closest_waypoint(x, y, x_ref, y_ref)
    dists = sqrt((x_ref - x).^2 + (y_ref - y).^2);
    [~, idx] = min(dists);
end