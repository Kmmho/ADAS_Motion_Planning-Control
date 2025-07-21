function global_path  = trajectory_path(s_ref, global_waypoint_2, Ego_x, Ego_y, Ego_Yaw, Ego_Vx, Ego_Ax, target_speed, obs_pre, TargetTolgateLane, gate_flag)
    LANE_WIDTH = 3.0;
    CENTER_LANE = 2;
    TargetTolgateLane_d = (TargetTolgateLane - CENTER_LANE) * LANE_WIDTH;



    mapx= global_waypoint_2(:,1);
    mapy= global_waypoint_2(:,2);
    
    % === 초기 자차 위치 및 상태 === %
    
    obs=obs_pre(:,[1:2]);

    x0 = Ego_x; %
    y0 = Ego_y; %
    yaw0 = Ego_Yaw; % 
    v0 = Ego_Vx; a0 = Ego_Ax; %% 초기값 

    % 1. Frenet 좌표계 위치 (s0, d0) 계산
    [s0, d0] = get_frenet(x0, y0, mapx, mapy);

    % 2. 기준 경로에서의 진행 방향 (yaw_path) 계산
    [~, ~, yaw_path] = get_cartesian(s0, d0, mapx, mapy, s_ref);

    % 3. 경로 기준 상대 진행 방향 계산
    yawi = yaw0 - yaw_path;

    % 4. Frenet 상태 초기값
    si    = s0;
    si_d  = v0 * cos(yawi);
    si_dd = a0 * cos(yawi);
    sf_d  = target_speed; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sf_dd = 0;

    di    = d0;
    di_d  = v0 * sin(yawi);
    di_dd = a0 * sin(yawi);
    df_d  = 0;
    df_dd = 0;

    opt_d = di;  % 횡 방향 목표 유지
    
    %% 
   
    [path, opt_ind] = frenet_optimal_planning(...
                                              si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, ...
                                              df_d, df_dd, obs, mapx, mapy, s_ref, opt_d, gate_flag, TargetTolgateLane_d); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%obs 만처리하면됨 
   
    %% 출력
    global_path = [path(opt_ind).x(1:path(opt_ind).len)', path(opt_ind).y(1:path(opt_ind).len)'];  % N×2
    
    

    % % === 디버깅용 시각화 ===
    % figure(1); clf; hold on; axis equal;
    % plot(mapx, mapy, 'k--'); % 기준 경로
    % plot(global_path(:,1), global_path(:,2), 'b-', 'LineWidth', 2); % 최적 궤적
    % plot(Ego_x, Ego_y, 'ro'); % 자차 현재 위치
    % title('Selected Optimal Trajectory');
    % grid on;

    
end

function [path, opt_ind] = frenet_optimal_planning(...
                                                  si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, ...
                                                  df_d, df_dd, obs, mapx, mapy, s_ref, opt_d, gate_flag, TargetTolgateLane_d)
    % 1. Frenet 궤적 후보 생성
    [fplist, MAX_len, LENG_path] = calc_frenet_paths(...
                                                    si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, ...
                                                    df_d, df_dd, opt_d, gate_flag, TargetTolgateLane_d);


    % 2. Global 궤적 변환
    fplist = calc_global_paths(fplist, mapx, mapy, s_ref);

    % 3. 제약조건 및 충돌 검사 → 유효 trajectory만 반환
    [valid_paths, num_valid] = check_path(fplist, obs, MAX_len, LENG_path);

    % 4. 최적 경로 선택
    min_cost = inf;
    opt_ind = 1;  % 초기값

    for i = 1:num_valid
        if valid_paths(i).c_tot < min_cost
            min_cost = valid_paths(i).c_tot;
            opt_ind = i;
        end
    end

    % 5. 출력 (최종 유효 경로들)
    path = valid_paths;
end

    

function [fplist, MAX_len, LENG_path] = calc_frenet_paths(...
                                                            si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, ...
                                                            df_d, df_dd, opt_d, gate_flag, TargetTolgateLane_d)
  
    % % 차선 번호(1,2,3)를 실제 Frenet d 좌표로 변환
    % LANE_WIDTH = 3;     % 실제 차선 폭 [m]
    % CENTER_LANE = 2;      % 중앙 차선 번호
    % TargetTolgateLane_d = (TargetTolgateLane - CENTER_LANE) * LANE_WIDTH;


    
    TARGET_SPEED = sf_d;
    DF_SET = linspace(-3, 3, 3); 
    DT = 0.05; 
    DT_T = 0.2; 
    MIN_T = 2; 
    MAX_T = 4; 
     % K_J Jerk 웨이트 s,d 둘다 , K_T t_f에 대한 항, K_D Diff d에대한 diff, K_V s에 대한 diff(속도 s_d에 대한 diff)
    MAX_len= (MAX_T/DT)+1;

    K_J = 0.1; K_T = 1; K_D = 1; K_V = 1;

    W_LAT = 1.0; W_LON = 1.0; %종방향 횡방향 웨이트
    
    LENG_path = 3*(MAX_T-MIN_T)/DT_T; 
    fp_templet = struct( ...
    't', zeros(1,MAX_len), ...
    'd', zeros(1,MAX_len), 'd_d', zeros(1,MAX_len), 'd_dd', zeros(1,MAX_len), 'd_ddd', zeros(1,MAX_len), ...
    's', zeros(1,MAX_len), 's_d', zeros(1,MAX_len), 's_dd', zeros(1,MAX_len), 's_ddd', zeros(1,MAX_len), ...
    'c_lat', 0, 'c_lon', 0, 'c_tot', 0, ...
    'x', zeros(1,MAX_len), 'y', zeros(1,MAX_len), 'yaw', zeros(1,MAX_len), ...
    'ds', zeros(1,MAX_len), 'kappa', zeros(1,MAX_len), ...
    'len', 0);

    fplist = repmat(fp_templet, 1, LENG_path);  % 빈 구조체 배열로 초기화
    index=1;
    for df = DF_SET
        for T = MIN_T:DT_T:MAX_T
            
            % Lateral trajectory (Quintic)
            [a0_d, a1_d, a2_d, a3_d, a4_d, a5_d] = calc_quintic(di, di_d, di_dd, df, df_d, df_dd, T);
            
            t = 0:DT:T;
            
            d = a0_d + a1_d*t + a2_d*t.^2 + a3_d*t.^3 + a4_d*t.^4 + a5_d*t.^5;
            d_d = a1_d + 2*a2_d*t + 3*a3_d*t.^2 + 4*a4_d*t.^3 + 5*a5_d*t.^4;
            d_dd = 2*a2_d + 6*a3_d*t + 12*a4_d*t.^2 + 20*a5_d*t.^3;
            d_ddd = 6*a3_d + 24*a4_d*t + 60*a5_d*t.^2;
            
            
            % Longitudinal trajectory (Quartic)
            %sf_d에 Target Speed를 기준으로 여러개 생성해야된다
            %sf 인자는 어디로 갔나
            [a0_s, a1_s, a2_s, a3_s, a4_s] = calc_quartic(si, si_d, si_dd, sf_d, sf_dd, T);
            s = a0_s + a1_s*t + a2_s*t.^2 + a3_s*t.^3 + a4_s*t.^4;
            s_d = a1_s + 2*a2_s*t + 3*a3_s*t.^2 + 4*a4_s*t.^3;
            s_dd = 2*a2_s + 6*a3_s*t + 12*a4_s*t.^2;
            s_ddd = 6*a3_s + 24*a4_s*t;
            
           
            %% 비용 계산
            J_lat = sum(d_ddd.^2); %적분항 인데 왜 T로 안나눠 임마 
            J_lon = sum(s_ddd.^2); %적분항 
            if gate_flag
                d_gate_weight = 10000000000;
                d_diff = (d(end) - TargetTolgateLane_d)^2 * d_gate_weight;
            else
                d_diff = (d(end) - opt_d)^2;
            end

            % if gate_flag
            %     opt_d = TargetTolgateLane_d;
            % else
            %     opt_d = di;
            % end


            s_diff = (TARGET_SPEED - s_d(end))^2; %ACC랑 최대한 유지하기 위함

            
            % if gate_flag
            %     d_gate_weight = 10000000000;
            %     d_diff = (d(end) - TargetTolgateLane_d)^2 * d_gate_weight;
            %     fprintf('[DEBUG] df = %.2f | d(end) = %.2f | target_d = %.2f | cost = %.2e\n', ...
            %             df, d(end), TargetTolgateLane_d, d_diff);
            % else
            %     d_diff = (d(end) - opt_d)^2;
            % end

            %% 비용함수
            c_lat = K_J * J_lat  + K_D * d_diff + K_T * T; %% cost_lateral
            c_lon = K_J * J_lon  + K_V * s_diff + K_T * T; %% cost_longitual
            c_tot = W_LAT * c_lat + W_LON * c_lon; %% total Cost가 낮은것을 선택

            len = length(t);  % ← 이 줄이 꼭 필요!

            % 1. fp 템플릿 기반 새 구조체 생성
            fp = fp_templet;

            % 2. 유효 길이 저장
            fp.len = len;

            % 3. 시간 및 궤적 데이터 복사
            fp.t(1:len)     = t;
            fp.d(1:len)     = d;
            fp.d_d(1:len)   = d_d;
            fp.d_dd(1:len)  = d_dd;
            fp.d_ddd(1:len) = d_ddd;

            fp.s(1:len)     = s;
            fp.s_d(1:len)   = s_d;
            fp.s_dd(1:len)  = s_dd;
            fp.s_ddd(1:len) = s_ddd;

            % 4. 비용도 할당
            fp.c_lat = c_lat;
            fp.c_lon = c_lon;
            fp.c_tot = c_tot;

            % 5. fplist에 대입
            fplist(index) = fp;
            if(index<LENG_path)
            index = index +1;
            end
        end
    end
end

function fplist = calc_global_paths(fplist, mapx, mapy, s_ref)
    for i = 1:length(fplist)
        fp = fplist(i);
        len = fp.len;

        % === Global 좌표 변환 === %
        for j = 1:len
            [fp.x(j), fp.y(j), ~] = get_cartesian(fp.s(j), fp.d(j), mapx, mapy, s_ref);
        end

        % === Yaw, ds 계산 === %
        for j = 1:len-1
            dx = fp.x(j+1) - fp.x(j);
            dy = fp.y(j+1) - fp.y(j);
            fp.yaw(j) = atan2(dy, dx);
            fp.ds(j) = hypot(dx, dy);
        end
        if len >= 2
            fp.yaw(len) = fp.yaw(len - 1);
            fp.ds(len)  = fp.ds(len - 1);
        else
            fp.yaw(len) = 0;
            fp.ds(len)  = 1e-6;  % 너무 작지 않게 보호
        end

        % === 곡률 계산 === %
        for j = 1:len-1
            dyaw = atan2(sin(fp.yaw(j+1) - fp.yaw(j)), cos(fp.yaw(j+1) - fp.yaw(j)));
            fp.kappa(j) = dyaw / fp.ds(j);
        end
        if len >= 2
            fp.kappa(len) = fp.kappa(len - 1);
        else
            fp.kappa(len) = 0;
        end

        % 업데이트된 fp 구조체 다시 저장
        fplist(i) = fp;
    end
end

function [valid_paths, numValid] = check_path(fplist, obs, MAX_len, LENG_path)
    % 정적 템플릿 구조체 선언
    fp_templet = struct( ...
        't', zeros(1,MAX_len), ...
        'd', zeros(1,MAX_len), 'd_d', zeros(1,MAX_len), 'd_dd', zeros(1,MAX_len), 'd_ddd', zeros(1,MAX_len), ...
        's', zeros(1,MAX_len), 's_d', zeros(1,MAX_len), 's_dd', zeros(1,MAX_len), 's_ddd', zeros(1,MAX_len), ...
        'c_lat', 0, 'c_lon', 0, 'c_tot', 0, ...
        'x', zeros(1,MAX_len), 'y', zeros(1,MAX_len), 'yaw', zeros(1,MAX_len), ...
        'ds', zeros(1,MAX_len), 'kappa', zeros(1,MAX_len), ...
        'len', 0);

    % 제약 조건
    V_MAX = 30;
    ACC_MAX = 5;
    K_MAX = 1.5;

    % 고정 크기 구조체 배열 선언
    valid_paths = repmat(fp_templet, 1, LENG_path);
    index = 1;
    numValid = 0;

    for i = 1:length(fplist)
        fp = fplist(i);
        len = fp.len;

        acc_squared = fp.s_dd(1:len).^2;  % s 방향에 대한 가속도만 고려
        

        if all(fp.s_d(1:len) > V_MAX)

            continue;
        % elseif any(acc_squared > ACC_MAX^2)
        %     continue;
        elseif any(abs(fp.kappa(1:len)) > K_MAX)
            continue;
        elseif collision_check(fp, obs)
            continue;
        elseif any(abs(fp.d(1:len)) > 3.5)
            continue;
        else
            valid_paths(index) = fp;
            index = index + 1;
            numValid = numValid + 1;
        end
    end

end


function collision = collision_check(fp, obs_global)
    CAR_RADIUS = 2;
    OBSTACLE_RADIUS = 0.5;
    SAFE_DIST_SQ = (CAR_RADIUS + OBSTACLE_RADIUS)^2;

    len = fp.len;
    collision = false;

    for i = 1:size(obs_global, 1)
        dx = fp.x(1:len) - obs_global(i,1);
        dy = fp.y(1:len) - obs_global(i,2);
        dists_sq = dx.^2 + dy.^2;

        if any(dists_sq <= SAFE_DIST_SQ)
            collision = true;
            return;
        end
    end
end



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

function [a0, a1, a2, a3, a4, a5] = calc_quintic(xi, vi, ai, xf, vf, af, T)
    a0 = xi;
    a1 = vi;
    a2 = 0.5 * ai;
    A = [T^3, T^4, T^5;
         3*T^2, 4*T^3, 5*T^4;
         6*T, 12*T^2, 20*T^3];
    b = [xf - a0 - a1*T - a2*T^2;
         vf - a1 - 2*a2*T;
         af - 2*a2];
    x = A\b;
    a3 = x(1); a4 = x(2); a5 = x(3);
end

function [a0, a1, a2, a3, a4] = calc_quartic(xi, vi, ai, vf, af, T)
    a0 = xi;
    a1 = vi;
    a2 = 0.5 * ai;
    A = [3*T^2, 4*T^3;
         6*T, 12*T^2];
    b = [vf - a1 - 2*a2*T;
         af - 2*a2];
    x = A\b;
    a3 = x(1); a4 = x(2);
end