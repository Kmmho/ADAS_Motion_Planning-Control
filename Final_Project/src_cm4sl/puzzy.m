
% 퍼지 모델 기반 예측 경로 애니메이션 시각화
% a와 yaw를 동적으로 변화시키며 경로 시각화

function y = trapmf(x, params)
    a = params(1); b = params(2); c = params(3); d = params(4);

    if x <= a || x >= d
        y = 0;
    elseif x >= b && x <= c
        y = 1;
    elseif x > a && x < b
        y = (x - a) / (b - a);
    elseif x > c && x < d
        y = (d - x) / (d - c);
    else
        y = 0;
    end
end

% 시간 설정
t = 0:0.1:5;

% 시뮬레이션 설정
figure;
for k = 1:30
    % 입력값 변화 (sine/cosine 기반 변화)
    a = 1 + 2 * abs(sin(k/10));
    yaw = 0.2 + 1.5 * abs(cos(k/15));

    % 멤버십 함수
    low_a = trapmf(a, [0 0 1 2]);
    high_a = trapmf(a, [1.5 2.5 5 5]);
    low_yaw = trapmf(yaw, [0 0 0.5 1]);
    high_yaw = trapmf(yaw, [0.8 1.2 2 2]);

    % 규칙 적용
    w1 = min(low_a, low_yaw);  % CV=0.5, CA=0.5
    w2 = min(high_a, low_yaw); % CA=0.7, CTRA=0.3
    w3 = min(low_a, high_yaw); % CTRV=0.6, CTRA=0.4
    w4 = min(high_a, high_yaw);% CTRV=0.5, CTRA=0.5

    % 가중치 계산
    cv = 0.5 * w1;
    ca = 0.5 * w1 + 0.7 * w2;
    ctrv = 0.6 * w3 + 0.5 * w4;
    ctra = 0.3 * w2 + 0.4 * w3 + 0.5 * w4;
    sum_w = cv + ca + ctrv + ctra;
    if sum_w == 0
        cv = 0.25; ca = 0.25; ctrv = 0.25; ctra = 0.25;
    else
        cv = cv / sum_w;
        ca = ca / sum_w;
        ctrv = ctrv / sum_w;
        ctra = ctra / sum_w;
    end

    % 각 모델 경로
    x_cv = t; y_cv = zeros(size(t));
    x_ca = t; y_ca = 0.5 * (a * t.^2);
    x_ctrv = sin(0.4 * t) * 10;
    y_ctrv = cos(0.4 * t) * 10;
    x_ctra = sin(0.4 * t) .* (1 + 0.2 * t);
    y_ctra = cos(0.4 * t) .* (1 + 0.2 * t);

    % 최종 예측 경로
    x_pred = cv * x_cv + ca * x_ca + ctrv * x_ctrv + ctra * x_ctra;
    y_pred = cv * y_cv + ca * y_ca + ctrv * y_ctrv + ctra * y_ctra;

    % 시각화
    clf;
    plot(x_cv, y_cv, '--', 'DisplayName', 'CV'); hold on;
    plot(x_ca, y_ca, '--', 'DisplayName', 'CA');
    plot(x_ctrv, y_ctrv, '--', 'DisplayName', 'CTRV');
    plot(x_ctra, y_ctra, '--', 'DisplayName', 'CTRA');
    plot(x_pred, y_pred, 'k-', 'LineWidth', 2, 'DisplayName', 'Fuzzy Prediction');
    legend;
    title(sprintf('a = %.2f, yaw = %.2f', a, yaw));
    xlabel('X'); ylabel('Y'); axis equal; grid on;
    pause(0.3);
end
