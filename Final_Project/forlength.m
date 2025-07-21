% === map.mat 파일 불러오기 ===
load('final.mat');

% === 첫 번째 시간대의 2D 맵 데이터 추출 ===
map_array = map_data.Data{1};  % 0초 시점의 41x42 double 행렬

% === 데이터 시각화 (선택) ===
figure;
imagesc(map_array);
axis equal tight;
colormap(gray);
title('Extracted Map at 0초');

% === .mat 파일로 저장 ===
save('map_simple.mat', 'map_array');
disp('map_simple.mat 파일로 저장 완료!');