function [target_speed, debug_time_gap] = acc_timegap_based( ...
    ego_pos, ego_yaw, ego_vx, ego_lane_idx, ...
    traffic_info, traffic_lane_idx)

    % 기본 설정
    default_speed = 2.8;      % m/s (기본 크루즈 속도)
    safe_gap_time = 3.0;       % 안전 시간 간격 (초)
    emergency_gap_time = 2.0;  % 비상 시간 간격 (초)

    % 전방 차량 정보 분리
    traffic_pos = traffic_info(:, 1:2);   % 위치 [8x2]
    traffic_vx = traffic_info(:, 3);      % 속도 [8x1]
    L = size(traffic_info, 1);            % 차량 수

    % 초기값 설정
    min_gap_time = inf;
    front_vx = 0;
    found = false;

    ego_dir = [cos(ego_yaw); sin(ego_yaw)];  % 주행 방향 단위 벡터

    for i = 1:L
        if traffic_lane_idx(i) == ego_lane_idx
            dx = traffic_pos(i,1) - ego_pos(1);
            dy = traffic_pos(i,2) - ego_pos(2);
            rel_vec = [dx; dy];

            if dot(rel_vec, ego_dir) > 0  % 전방 차량인지 확인
                dist = norm(rel_vec);

                % ego 속도가 0이면 분모 0 방지
                if ego_vx > 0
                    time_gap = dist / ego_vx;
                else
                    time_gap = inf;
                end

                if time_gap < min_gap_time
                    min_gap_time = time_gap;
                    front_vx = traffic_vx(i);
                    found = true;
                end
            end
        end
    end

    % 속도 결정 로직
    target_speed = default_speed;

    if found
        if min_gap_time < emergency_gap_time
            target_speed = 0;  % 매우 위험 → 정지
        elseif min_gap_time < safe_gap_time
            ratio = (min_gap_time - emergency_gap_time) / (safe_gap_time - emergency_gap_time);
            target_speed = front_vx * ratio;
        else
            target_speed = default_speed;
        end
    end

    % 디버깅 정보 출력용
    debug_time_gap = min_gap_time;
end


