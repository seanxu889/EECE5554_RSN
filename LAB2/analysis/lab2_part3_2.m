% imu part3_2 Estimate the forward velocity
clear;
clc;

% data using:
% part2_trajectory_imu_out.csv
% part2_trajectory_gps_out.csv

%% Integrate the forward acceleration to estimate the forward velocity
imu_data = readtable('part2_trajectory_imu_out.csv', 'HeaderLines',1);
imu_acc = table2array(imu_data(:, 30:32));
t = linspace(1, 852, 34084);
fwd_vel_from_acc = cumtrapz(t, imu_acc(:, 1));

figure(1);
plot(t, fwd_vel_from_acc, 'linewidth', 2.0)
xlabel('time series (second)'); 
ylabel('forward velocity (m/s)');
title('Forward Velocity from acceleration w/o adjustments');
grid on

%% Make adjustments to the acceleration measurements
% initial_drift = mean(imu_acc(1:700, 1:2)); % data 1:700 is the stationary
% imu_acc_adj = imu_acc(:, 1:2) - initial_drift; % adjust by abstruct the stationary initial_drift
% fwd_vel_from_acc_adj = cumtrapz(t, imu_acc_adj(:, 1));
% figure(2);
% plot(t, abs(fwd_vel_from_acc_adj), 'linewidth', 2.0) % adjust by abs()
% xlabel('time series (second)'); 
% ylabel('forward velocity (m/s)');
% title('Forward Velocity from acceleration after adjustments');
% grid on

% apply Moving Average Filter (low pass filter)
windowSize = 75;
imu_acc_mv = filter(ones(1, windowSize) / windowSize, 1, imu_acc(:, 1));
imu_acc_mv2 = imu_acc_mv;

% find each stationary measurement region, 
% and adjust by abstructing each drift for the following measurement values
len = 1;
index_start = [];
index_end = [];

% test:
% Delta = [];
% for mm = 2 : length(imu_acc_mv)
%     Delta = [Delta, abs(imu_acc_mv(mm) - imu_acc_mv(mm-1))];
% end

for mm = 2 : length(imu_acc_mv)-1
    delta_vel = abs(imu_acc_mv(mm) - imu_acc_mv(mm-1));
    delta_vel_next = abs(imu_acc_mv(mm+1) - imu_acc_mv(mm));
    
    if delta_vel <= 0.0075
        len = len + 1;
        if delta_vel_next > 0.0075 && len > 500
            index_start = [index_start, mm-len+1];
            index_end = [index_end, mm];
            len = 1;
        end        
        if delta_vel_next >= 0.0075
            len = 1;
        end
    end   
end

for kk = 1 : length(index_start)
    drift_mean = mean(imu_acc_mv(index_start(kk): index_end(kk)));
    if kk == length(index_start)
        imu_acc_mv(index_start(kk): end) = imu_acc_mv(index_start(kk): end) - drift_mean;
    else
        imu_acc_mv(index_start(kk): index_start(kk+1)-1) = imu_acc_mv(index_start(kk): index_start(kk+1)-1) - drift_mean;
    end
end
%imu_acc_mv(1:4000) = 0; % only consider the data after circle-drive, start from the straight line

% create a dynamic scaling factor for acceleration adjustment 17300 : 22180
% dynamic_factor1 = linspace(3, 0.2, 34084);
% dynamic_factor2 = linspace(1, 2, 34084);
% for rr = 17300:22180%length(imu_acc_mv)
%     if imu_acc_mv(rr) > 0
%         imu_acc_mv(rr) = imu_acc_mv(rr) * 0.9;%dynamic_factor1(rr);
%     end
%     if imu_acc_mv(rr) < 0
%         imu_acc_mv(rr) = imu_acc_mv(rr) * 1.5;%dynamic_factor2(rr);
%     end
% end

figure(2);
plot(imu_acc_mv2, 'linewidth', 1.5, 'color', [1 0.7 0]);
xlabel('time series (second)'); 
ylabel('adjusted acceleration (m/s^2)');
title('adjusted acceleration');
grid on
hold on
plot(imu_acc_mv, 'linewidth', 1.5, 'color', 'k');
legend('acc after moving agerage filter', 'compensation drift for filtered acc');
hold off

% integrate the adjusted acceleration section by section
fwd_vel_from_acc_adj = [];
for nn = 1 : length(index_start)
    if nn == 1 && index_start(1) ~= 1
        fwd_vel_from_acc_adj = [fwd_vel_from_acc_adj; cumtrapz(imu_acc_mv(1: index_start(1)-1))];
    end
    if nn == length(index_start)
        fwd_vel_from_acc_adj = [fwd_vel_from_acc_adj; cumtrapz(imu_acc_mv(index_start(nn):end))];
    else
        fwd_vel_from_acc_adj = [fwd_vel_from_acc_adj; cumtrapz(imu_acc_mv(index_start(nn):index_start(nn+1)-1))];
    end
end
fwd_vel_from_acc_adj = abs(fwd_vel_from_acc_adj) / 100;

save('imu_acc_mv', 'imu_acc_mv'); % save for future use
save('fwd_vel_from_acc_adj', 'fwd_vel_from_acc_adj'); % save for future use

figure(3);
plot(fwd_vel_from_acc_adj, 'linewidth', 2.0);
xlabel('time series (second)'); 
ylabel('adjusted forward velocity (m/s)');
title('Forward Velocity from acceleration after adjustments');
grid on

%% Calculate an estimate of the velocity from GPS measurements
gps_data = readtable('part2_trajectory_gps_out.csv', 'HeaderLines',1);
gps_data_utm = table2array(gps_data(:, 8:9));
gps_data_utm(:, 1) = gps_data_utm(:, 1) - min(gps_data_utm(:, 1));
gps_data_utm(:, 2) = gps_data_utm(:, 2) - min(gps_data_utm(:, 2));
gps_data_degree = table2array(gps_data(:, 5:6));
save('gps_data_utm', 'gps_data_utm'); % save for future use

% extract the GPS time info
gps_data_time = table2array(gps_data(:, 1));
time = [];
for jj = 1:length(gps_data_time)
    tt = sprintf('%f',gps_data_time(jj));
    time = [time; tt(7:11)];
end
time = str2num(time);
time(802:end) = time(802:end) + 100000;
time = (time - time(1)) / 10; % total recording time = 846.9 sec

fwd_vel_from_gps = [];
for jj = 2:length(gps_data_degree)
    % Length in meters of 1° of latitude = 111.32 km
    % Length in meters of 1° of longitude = 111.32 km * cos(latitude) 
    % = 111.32 km * cos(deg2rad(42.33)) = 82.2965 km
    
    delta_lat = (gps_data_degree(jj, 1) - gps_data_degree(jj-1, 1)) * 111320;
    delta_lon = (gps_data_degree(jj, 2) - gps_data_degree(jj-1, 2)) * 82296.5;
    delta_distance = sqrt(delta_lat^2 + delta_lon^2);
    dt = time(jj) - time(jj-1);
    vel = delta_distance / dt;
    fwd_vel_from_gps = [fwd_vel_from_gps; vel];
end
save('fwd_vel_from_gps', 'fwd_vel_from_gps'); % save fwd_vel_from_gps for future comparation

figure(4);
plot(fwd_vel_from_gps, 'linewidth', 2.0) 
xlabel('time series (second)'); 
ylabel('forward velocity (m/s)');
title('Forward Velocity from GPS');
grid on

% plot trajectory
figure(5);
plot(gps_data_utm(:, 1), gps_data_utm(:, 2), 'linewidth', 0.5);
xlabel('UTM easting'); 
ylabel('UTM northing');
title('GPS UTM data trajectory');
grid on

figure(5);
plot(gps_data_degree(:,2),gps_data_degree(:,1), 'linewidth', 0.5);
xlabel('lon'); 
ylabel('lat');
title('GPS degree data trajectory');
grid on
