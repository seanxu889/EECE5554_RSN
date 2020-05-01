% imu part3_1 Estimate the heading (yaw)
clear;
clc;

% data using:
% part2_trajectory_mag_out.csv
% part2_trajectory_imu_out.csv

%% Magnetometer Calibration
mag_data = readtable('part2_trajectory_mag_out.csv', 'HeaderLines',1);
mag_data = table2array(mag_data(:, 5:7));

% show original data points
figure(1);
scatter(mag_data(1000:4000, 1), mag_data(1000:4000, 2),'b.'); % the circle driving data points is 1000-4000
xlabel('x (Gauss)'); 
ylabel('y (Gauss)');
title('1. Original data points');
grid on

% fit the data to an ellipse 
% Richard Brown (2020). fitellipse.m (https://www.mathworks.com/matlabcentral/fileexchange/15125-fitellipse-m), MATLAB Central File Exchange.
[z, a, b, alpha] = fitellipse([mag_data(1000:4000, 1), mag_data(1000:4000, 2)]', 'linear');
hF = figure(2);
hAx = axes('Parent', hF);
h = plotellipse(hAx, z, a, b, alpha, 'r.');
grid on
hold on
plotellipse(z, a, b, alpha)
xlabel('x (Gauss)'); 
ylabel('y (Gauss)');
title('2. Fit the data points to an ellipse');
grid on
hold on
scatter(z(1), z(2), 'filled', 'ko')
legend(' ', 'Fitted Ellipse', 'Origin');
hold off

% hard-iron corrections
x_offset = z(1);
y_offset = z(2);

mag_data(:, 1) = mag_data(:, 1) - x_offset;
mag_data(:, 2) = mag_data(:, 2) - y_offset;

figure(3);
subplot(2,2,1);
plot(mag_data(1000: 4000, 1), mag_data(1000:4000, 2), 'b.');
grid on
xlabel('x (Gauss)'); 
ylabel('y (Gauss)');
title('3. Magnetometer data after hard-iron correction');
hold on

% soft-iron corrections
% rotation, align the major axis of the ellipse with the reference frame X axis
theta = alpha;
rotation_matrix = [cos(theta), -sin(theta);
                   sin(theta), cos(theta)];
mag_new_xy = rotation_matrix * mag_data(:, 1:2)';
mag_data(:, 1:2) = mag_new_xy';

subplot(2,2,2);
plot(mag_data(1000:4000, 1), mag_data(1000:4000, 2), 'b.');
grid on
xlabel('x (Gauss)'); 
ylabel('y (Gauss)');
title('4. Magnetometer data after soft-iron rotation');

% scale, scalling the major axis such that the ellipse is converted to a circle
scale_factor = a / b;
mag_data(:, 1) = mag_data(:, 1) / scale_factor;

subplot(2,2,3);
plot(mag_data(1000:4000, 1), mag_data(1000:4000, 2), 'b.');
grid on
xlabel('x (Gauss)'); 
ylabel('y (Gauss)');
title('5. Magnetometer data after soft-iron scalling');

% back-rotation, rotating the data back to their original position
theta_back = -theta;
rotation_matrix_back = [cos(theta_back), -sin(theta_back);
                        sin(theta_back), cos(theta_back)];
mag_new_xy_back = rotation_matrix_back * mag_data(:, 1:2)';
mag_data(:, 1:2) = mag_new_xy_back';

subplot(2,2,4);
plot(mag_data(1000:4000, 1), mag_data(1000:4000, 2), 'b.');
grid on
xlabel('x (Gauss)'); 
ylabel('y (Gauss)');
title('6. Magnetometer data after soft-iron correction');

%% Calculate the yaw angle 
% from the corrected magnetometer readings
yaw_from_mag = 180 * atan2(-mag_data(:,2), mag_data(:,1)) / pi;
yaw_from_mag = deg2rad(yaw_from_mag);
yaw_from_mag_shift = unwrap(yaw_from_mag); % shift phase angles using unwrap()
t = linspace(1, 852, 34084);
save('yaw_from_mag_shift', 'yaw_from_mag_shift'); % save for future use

figure(4);
plot(t(4000:end), yaw_from_mag_shift(4000:end)+21.88, 'linewidth',1.5)
%plot(t, yaw_from_mag, 'linewidth', 0.8)
grid on
hold on

% integrate the yaw rate sensor (gyro) to get yaw angle.
imu_data = readtable('part2_trajectory_imu_out.csv', 'HeaderLines',1);
gyro_data = table2array(imu_data(:, 18:20));
gyro_data = gyro_data - mean(gyro_data(1:400,:)); % zero mean
yaw_from_gyro = cumtrapz(t, gyro_data(:, 3));
save('yaw_from_gyro', 'yaw_from_gyro'); % save fwd_vel_from_gps for future use

% align the yaw angle
initial_drift = yaw_from_mag_shift(1);
yaw_from_gyro_ture = yaw_from_gyro + initial_drift;
save('yaw_from_gyro_ture', 'yaw_from_gyro_ture'); % save fwd_vel_from_gps for future use
plot(t(4000:end), yaw_from_gyro_ture(4000:end)+21.88, 'linewidth', 1.5)
xlabel('time series (second)'); 
ylabel('yaw (rad)');
title('Yaw Angle');
legend('Yaw from corrected magnetometer', 'Yaw from gyro (rate sensor)');
hold off

%% Use complementary filter to combine the magnetometer and yaw rate measurements
% filter the magnetometer estimate using a low pass filter,
% and gyro estimate using a high pass filter

updateRate = 40;  
dt = 1 / updateRate;  
hpf = 0.999;   % 852sec / (852+0.025) = 0.99997
lpf = 1 - hpf;

mag_z = lpf * yaw_from_mag_shift;
gyro_z = gyro_data(:, 3);
theta_z_combine = zeros(size(gyro_z));
theta_z_combine(1) = hpf * theta_z_combine(1) * dt + mag_z(1);% + initial_drift;

for ii = 2:length(gyro_z) 
    theta_z_combine(ii) = hpf * (theta_z_combine(ii-1) + gyro_z(ii) * dt) + mag_z(ii);
end
save('theta_z_combine', 'theta_z_combine'); % save theta_z_combine for future use

figure(5)
plot(t(4000:end), theta_z_combine(4000:end)+21.88, 'linewidth', 2.0);
hold on

%% Yaw angle directly output by the IMU
quaternion = [table2array(imu_data(:, 8)), table2array(imu_data(:, 5:7))]; % wxyz
eul = quat2eul(quaternion); % Z,Y,X = yaw,roll,pitch
yaw_from_IMU = eul(:, 1);
yaw_from_IMU_shift = unwrap(yaw_from_IMU); % shift phase angles using unwrap()
save('yaw_from_IMU_shift', 'yaw_from_IMU_shift'); % save yaw_from_IMU_shift for future use

plot(t(4000:end), yaw_from_IMU_shift(4000:end)+21.88, 'linewidth', 2.0)
%plot(t, yaw_from_IMU, 'linewidth', 2.0)
xlabel('time series (second)'); 
ylabel('yaw (rad)');
title('Yaw Angle');
grid on
legend('Yaw after complementary filter', 'Yaw from IMU output directly');
hold off

