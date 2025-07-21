% --- 데이터 로드 ---
data = load('Global_Map.mat');
waypoints = data.waypoints; % Nx2

N = size(waypoints,1);


% === 필수 경유 노드 지정 (여기만 바꾸면 됨!) === 
startIdx = 562; % Simulink에서 input
must1    = 0 ;   % Simulink에서 input
must2    = 0 ;  % Simulink에서 input
goalIdx  = 877; % Simulink에서 input

via_nodes = [startIdx];

if ~isempty(must1) && must1 ~= 0
    via_nodes = [via_nodes, must1];
end
if ~isempty(must2) && must2 ~= 0
    via_nodes = [via_nodes, must2];
end

via_nodes = [via_nodes, goalIdx];  % 마지막에 goal 추가



% --- 진행방향 단위 벡터 계산 ---
forward_vecs = zeros(N,2);
for i = 1:N-1
    v = waypoints(i+1,:) - waypoints(i,:);
    forward_vecs(i,:) = v / norm(v);
end
forward_vecs(N,:) = forward_vecs(N-1,:);

% --- 이웃 리스트 생성 (진행방향 기준) ---
neighborList = cell(N,1);
D_thresh = 5; % [m], 거리 조정

for i = 1:N
    dists = sqrt(sum((waypoints - waypoints(i,:)).^2, 2));
    candidateIdx = find(dists > 0 & dists < D_thresh);
    neighborIdx = [];
    for idx = candidateIdx'
        vec = waypoints(idx,:) - waypoints(i,:);
        vec = vec / norm(vec);
        if dot(forward_vecs(i,:), vec) > 0
            neighborIdx(end+1) = idx;
        end
    end
    neighborList{i} = neighborIdx;
end

% --- A* 함수 (start→goal) ---
function path = run_astar(startIdx, goalIdx, waypoints, neighborList)
    N = size(waypoints,1);
    cameFrom = zeros(N,1);
    gScore   = inf(N,1);
    fScore   = inf(N,1);

    gScore(startIdx) = 0;
    fScore(startIdx) = norm(waypoints(startIdx,:) - waypoints(goalIdx,:));
    openSet = startIdx;

    while ~isempty(openSet)
        [~, minIdx] = min(fScore(openSet));
        current = openSet(minIdx);
        if current == goalIdx
            break;
        end
        openSet(minIdx) = [];
        for neighbor = neighborList{current}
            tentative_gScore = gScore(current) + norm(waypoints(neighbor,:) - waypoints(current,:));
            if tentative_gScore < gScore(neighbor)
                cameFrom(neighbor) = current;
                gScore(neighbor)   = tentative_gScore;
                fScore(neighbor)   = tentative_gScore + norm(waypoints(neighbor,:) - waypoints(goalIdx,:));
                if ~ismember(neighbor, openSet)
                    openSet(end+1) = neighbor;
                end
            end
        end
    end

    if cameFrom(goalIdx) == 0
        error('경로를 찾지 못했습니다.');
    end

    path = goalIdx;
    while path(1) ~= startIdx
        path = [cameFrom(path(1)); path];
    end
end

% --- 전체 경로 연결 (via_nodes 순서대로) ---
full_path = [];
for k = 1:(length(via_nodes)-1)
    p = run_astar(via_nodes(k), via_nodes(k+1), waypoints, neighborList);
    if k > 1
        p = p(2:end); % 앞 path의 마지막 노드와 중복 방지
    end
    full_path = [full_path; p];
end

% --- 경로 좌표 추출 ---
global_path_xy = waypoints(full_path, :);  % [N_path x 2]

% --- 보간을 통한 촘촘한 경로 생성 ---
diffs = diff(global_path_xy);
seg_len = sqrt(sum(diffs.^2, 2));
s = [0; cumsum(seg_len)];
ds_interp = 1; % [m]
s_interp = (0:ds_interp:s(end))';
x_interp = interp1(s, global_path_xy(:,1), s_interp, 'spline');
y_interp = interp1(s, global_path_xy(:,2), s_interp, 'spline');
global_path_xy_dense = [x_interp, y_interp]; %simulink 에서 output

%--- 저장 ---
save('gotopark.mat', 'global_path_xy_dense');
save('global_path_xy.mat', 'global_path_xy');

% --- 시각화 ---
figure;
plot(waypoints(:,1), waypoints(:,2), 'k.'); hold on
plot(global_path_xy(:,1), global_path_xy(:,2), 'r-', 'LineWidth', 2);
plot(global_path_xy_dense(:,1), global_path_xy_dense(:,2), 'b.', 'MarkerSize', 6);
plot(waypoints(via_nodes,1), waypoints(via_nodes,2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);
legend('Waypoints', 'A* Path', 'Dense Interpolated Path', 'Must-Visit Nodes');
title('A* Global Path with Must-Visit Nodes');
axis equal;
