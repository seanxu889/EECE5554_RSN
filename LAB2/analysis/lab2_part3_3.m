% imu part3_3 Dead Reckoning with IMU - Integrate IMU data to obtain displacement and compare with GPS
clear;
clc;

% data using:
% part2_trajectory_imu_out.csv
% part2_trajectory_mag_out.csv
% part2_trajectory_gps_out.csv

%% 1. Compare w * X_dot AND acc_car_y
imu_data = readtable('part2_trajectory_imu_out.csv', 'HeaderLines',1);
imu_acc = table2array(imu_data(:, 30:32));
% initial_drift = mean(imu_acc(1:700, 2)); % data 1:700 is the stationary
% imu_acc = imu_acc(:, 1:2) - initial_drift; % adjust by abstruct the stationary initial_drift
t = linspace(1, 852, 34084);

struct_acc = load('imu_acc_mv.mat'); % read the adjusted acceleration from lab2_part3_2.m
imu_acc_mv = struct_acc.imu_acc_mv; 

struct_vel_car_x = load('fwd_vel_from_acc_adj.mat'); % read the velovity from lab2_part3_2.m
vel_car_x = struct_vel_car_x.fwd_vel_from_acc_adj; 

% compute w * X_dot
gyro_data = table2array(imu_data(:, 18:20));
w = gyro_data(:, 3) - mean(gyro_data(1:700, 3)); % rotation rate from gyro, subtract the mean
value = w .* vel_car_x;

acc_car_y = imu_acc(:, 2) - mean(imu_acc(1:700, 2)); % the acceleration measured by the inertial sensor

figure(1);
plot(t, value, 'color', 'b');
hold on 
xlabel('time series (second)'); 
title('Compare w * X dot AND acc car y');
grid on
plot(t, acc_car_y);
legend('w * Xdot', 'acc car y');

%% 2. Integrate (v_e, v_n) it to estimate the trajectory of the vehicle (x_e, x_n)
% mag_data = readtable('part2_trajectory_mag_out.csv', 'HeaderLines',1);
% mag_data = table2array(mag_data(:, 5:7));
% yaw_from_mag = 180 * atan2(-mag_data(:,2), mag_data(:,1)) / pi;
% yaw_from_mag = deg2rad(yaw_from_mag);
% yaw_from_mag_shift = unwrap(yaw_from_mag);

struct_yaw_from_mag_shift = load('yaw_from_mag_shift.mat'); 
yaw_from_mag_shift = struct_yaw_from_mag_shift.yaw_from_mag_shift;

struct_gps_vel = load('fwd_vel_from_gps.mat');
fwd_vel_from_gps = struct_gps_vel.fwd_vel_from_gps;
new_gps_vel = Get_New_GPS_Vel(fwd_vel_from_gps);

% use the heading from the magnetometer to rotate the direction
v_n = [];
v_e = [];

% --------------------uncomment one---------------------
% A: try to get trajectory using velocity from integrated acceleration
vel_car_x = vel_car_x;

% B: try to get trajectory using gps velocity, data is from lab2_part3_2.m line:64
%vel_car_x = new_gps_vel; 
% ------------------------------------------------------

struct_gyro_yaw = load('yaw_from_gyro.mat'); 
yaw_from_gyro = struct_gyro_yaw.yaw_from_gyro;
struct_gyro_yaw_ture = load('yaw_from_gyro_ture.mat');
yaw_from_gyro_ture = struct_gyro_yaw_ture.yaw_from_gyro_ture;
struct_filter_yaw = load('theta_z_combine.mat');
yaw_from_filter = struct_filter_yaw.theta_z_combine;
struct_IMU_yaw = load('yaw_from_IMU_shift.mat');
yaw_from_IMU_shift = struct_IMU_yaw.yaw_from_IMU_shift;

for ii = 1:length(yaw_from_mag_shift)
    % --------------------uncomment one----------------------
    angle = yaw_from_mag_shift(ii); % A: yaw from magnetometer
    %angle = yaw_from_gyro(ii); % B: yaw from gyro (data is from lab2_part3_1.m)
    %angle = yaw_from_gyro_ture(ii); % C: yaw from gyro (data is from lab2_part3_1.m)
    %angle = yaw_from_filter(ii); % D: yaw from complementary filter (data is from lab2_part3_1.m)
    %angle = yaw_from_IMU_shift(ii); % E: yaw from IMU direct output (data is from lab2_part3_1.m)
    % -------------------------------------------------------
    
    v_n = [v_n; vel_car_x(ii) * cos(angle)]; 
    v_e = [v_e; vel_car_x(ii) * sin(angle)];
end
v_head = [v_e, v_n];

% integrate [v_e, v_n] to estimate the trajectory of the vehicle (x_e ,x_n)
x_e = cumtrapz(t, v_e);
x_n = cumtrapz(t, v_n);
figure(2);
plot(x_e, x_n);
xlabel('East'); 
ylabel('North');
title('The estimated trajectory before adjustment');
grid on

% plotting the GPS track and estimated trajectory on the same plot
struct_gps_data_utm = load('gps_data_utm.mat'); % read the velovity from lab2_part3_2.m
gps_data_utm = struct_gps_data_utm.gps_data_utm; 

% initially adjust the starting point
x_e = x_e + gps_data_utm(1, 1);
x_n = x_n + gps_data_utm(1, 2);

% find the rotation angle theta by using the first straight line
% fit the straight line function using two points on it
GPS_point_A = [gps_data_utm(105, 1), gps_data_utm(105, 2)];
GPS_point_B = [gps_data_utm(138, 1), gps_data_utm(138, 2)];
slope_A = (GPS_point_B(2) - GPS_point_A(2)) / (GPS_point_B(1) - GPS_point_A(1));
degree_A = rad2deg(atan(slope_A) + pi);

estimated_point_A = [x_e(4076), x_n(4076)];
estimated_point_B = [x_e(5748), x_n(5748)];
slope_B = (estimated_point_B(2) - estimated_point_A(2)) / (estimated_point_B(1) - estimated_point_A(1));
degree_B = rad2deg(atan(slope_B) + pi);
theta = -deg2rad(degree_A - degree_B);
R_matrix = [cos(theta), sin(theta);
            -sin(theta), cos(theta)];
new_x = R_matrix * [x_e, x_n]';
new_x = new_x';
        
figure(3);
plot(gps_data_utm(:, 1), gps_data_utm(:, 2), 'linewidth', 0.5);
xlabel('UTM easting'); 
ylabel('UTM northing');
title('GPS UTM trajectory and estimated trajectory w/o adjust');
grid on
hold on
plot(x_e, x_n);
legend('GPS trajectory', 'estimated trajectory');
hold off

new_x(:, 1) = new_x(:, 1) + (gps_data_utm(117, 1) - new_x(117, 1));
new_x(:, 2) = new_x(:, 2) + (gps_data_utm(87, 2) - new_x(87, 2));

% after adjustment
figure(4);
plot(gps_data_utm(:, 1), gps_data_utm(:, 2), 'linewidth', 0.5);
xlabel('UTM easting'); 
ylabel('UTM northing');
title('GPS UTM trajectory and adjusted estimated trajectory');
grid on
hold on
plot(new_x(:, 1), new_x(:, 2));
legend('GPS trajectory', 'adjusted trajectory');
hold off

%% 3. Estimate x_c
gps_acc = [(new_gps_vel(1)) / t(1)];
for ii = 2:length(new_gps_vel)
    a_temp = (new_gps_vel(ii)-new_gps_vel(ii-1)) / (t(ii)-t(ii-1));
    gps_acc = [gps_acc; a_temp];
end

w_dot = [(w(1)) / t(1)];
for ii = 2:length(w)
    w_dot_temp = (w(ii)-w(ii-1)) / (t(ii)-t(ii-1));
    w_dot = [w_dot; w_dot_temp];
end

% test:
% figure(3)
% plot(vel_car_x(1:end-1))
% hold on
% plot(imu_acc_mv(120:end))
% hold off
% legend('gps', 'imu')

% test:
% figure(5)
% plot(imu_acc(:, 1))
% hold on
% plot(imu_acc_mv)
% hold off

zzz = w_dot + w.*w;
imu_acc_drift = mean(imu_acc(1:400, 1));
x_c = (imu_acc(:, 1)-imu_acc_drift - imu_acc_mv - value) ./ zzz;

figure(6)
plot(t, x_c);
grid on
xlabel('time'); 
ylabel('x c');
title('estimated x c');
text(312,3.736,'o','color','r');
text(323,-1.3,'o','color','r');

