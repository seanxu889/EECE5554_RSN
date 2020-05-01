% expand the fwd_vel_from_gps data points number from 847 to 34084 for convenience,
% which is equal to the number of IMU data.
% Add 39 points between each two data points of fwd_vel_from_gps.

function new_gps_vel = Get_New_GPS_Vel(fwd_vel_from_gps)

    original = fwd_vel_from_gps;
    new_gps_vel = fwd_vel_from_gps;

    for k = 1:length(original)-1
        flag = (k-1) * 39;
        append_points = linspace(original(k), original(k+1), 41);
        new_gps_vel = [new_gps_vel(1 : k+flag); append_points(2 : 40)'; new_gps_vel(k+flag+1 : end)]; 
    end
    new_gps_vel = [new_gps_vel; linspace(0, 0, 243)'];

end

% test dimension:
% figure(1);
% plot(fwd_vel_from_gps);
% hold on
% t2 = linspace(1, 852, length(new_gps_vel));
% plot(t2, new_gps_vel);
% hold off
