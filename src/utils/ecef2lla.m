function [latitude, longitude, height] = ecef2lla(x, y, z)
    a = 6378137.0;
    f = 1/298.257223563;
    e2 = 2*f - f^2;
    
    % 计算经度
    longitude = atan2(y, x) * 180/pi;
    rho = sqrt(x^2 + y^2);
    
    % 初始纬度估计
    latitude = atan2(z, rho) * 180/pi;
    
    % 迭代计算精确纬度
    for i = 1:10
        sin_lat = sin(latitude * pi/180);
        N = a / sqrt(1 - e2 * sin_lat^2);
        height = rho / cos(latitude * pi/180) - N;
        lat_new = atan2(z + N*e2*sin_lat, rho) * 180/pi;
        
        if abs(lat_new - latitude) < 1e-8
            break;
        end
        latitude = lat_new;
    end
end