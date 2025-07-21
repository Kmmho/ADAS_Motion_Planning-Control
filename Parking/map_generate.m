%% ======== generate_map_ ======== Ver 2025-05-17 10:40
function mapMatrix  = generate_map_(map_boundary, traffic_info)
    %변수설정
    resolution = 0.05;
    traffic_info_mat = reshape(traffic_info(1:5*21), 5, []).';
    x_values = map_boundary(1:2:end);
    y_values = map_boundary(2:2:end);
    N = size(traffic_info_mat,1);
    x_min = min(x_values);  x_max = max(x_values);
    y_min = min(y_values);  y_max = max(y_values);
    x_size = ceil((x_max-x_min)/resolution);
    y_size = ceil((y_max-y_min)/resolution);
    
    
    mapMatrix = zeros(y_size, x_size); %출력코드
    
    for i = 1:N
        x_c = traffic_info_mat(i, 1);
        y_c = traffic_info_mat(i, 2);
        yaw = traffic_info_mat(i, 3);
        length = traffic_info_mat(i, 4);
        width = traffic_info_mat(i, 5);
        
        if yaw == -1.57079628
            yaw = -pi/2;
        elseif yaw == 1.57079628
            yaw = pi/2;
        end

        dx = length/2;  dy = width/2;
        local = [-dx,-dy; dx,-dy; dx,dy; -dx,dy];
        R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
        rotated = (R*local')';
        forward_vec = [cos(yaw), sin(yaw)];
        side_vec    = [-sin(yaw), cos(yaw)];
        if yaw == -pi/2
            center = [x_c, y_c] + (length/2)*forward_vec + (width/2)*side_vec;
        else
            center = [x_c, y_c] + (length/2)*forward_vec - (width/2)*side_vec;
        end
        corners = rotated + center;
        x_idx = (corners(:,1)-x_min)/resolution + 1;
        y_idx = (y_max-corners(:,2))/resolution + 1;
        mask = poly2mask(x_idx, y_idx, y_size, x_size);
        mapMatrix(mask) = 1;
    end
end