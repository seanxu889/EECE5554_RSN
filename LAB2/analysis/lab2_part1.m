% imu part1 data analysis code
clear;
clc;

% data using:
% part1_imu.csv
% part1_imu_mag.csv

%% stationary data
data_imu = readtable('part1_imu.csv', 'HeaderLines',1);
imu_quaternions = table2array(data_imu(:, 5:8));
imu_acc = table2array(data_imu(:, 30:32));
imu_vel = table2array(data_imu(:, 18:20));

data_imu_mag = readtable('part1_imu_mag.csv', 'HeaderLines',1);
imu_mag = table2array(data_imu_mag(:, 5:7));

%% imu_mag (magnetometer) x,y,z with time respectively
imu_mag_mean_xyz = mean(imu_mag, 1);
imu_mag_std_xyz = std(imu_mag, 0, 1);

figure(1);
subplot(3,2,1)
plot(imu_mag(:, 1))
xlabel('time'); 
ylabel('imu mag');
title('IMU Magnetometer Measurement in X');
subplot(3,2,2)
hist(imu_mag(:,1) - imu_mag_mean_xyz(1), 500)
title('Noise Distribution of IMU Mag in X');

subplot(3,2,3)
plot(imu_mag(:, 2))
xlabel('time'); 
ylabel('imu mag');
title('IMU Magnetometer Measurement in Y');
subplot(3,2,4)
hist(imu_mag(:,2) - imu_mag_mean_xyz(2), 500)
title('Noise Distribution of IMU Mag in Y');

subplot(3,2,5)
plot(imu_mag(:, 3))
xlabel('time'); 
ylabel('imu mag');
title('IMU Magnetometer Measurement in Z');
subplot(3,2,6)
hist(imu_mag(:,3) - imu_mag_mean_xyz(3), 500)
title('Noise Distribution of IMU Mag in Z');

%% imu_acc (acceleration) x,y,z with time respectively
imu_acc_mean_xyz = mean(imu_acc, 1);
imu_acc_std_xyz = std(imu_acc, 0, 1);

figure(2);
subplot(3,2,1)
plot(imu_acc(:, 1))
xlabel('time'); 
ylabel('imu acc');
title('IMU Compensated Accelerometer Measurement in X with time');
subplot(3,2,2)
hist(imu_acc(:,1) - imu_acc_mean_xyz(1), 500)
title('Noise Distribution of IMU Acc in X');

subplot(3,2,3)
plot(imu_acc(:, 2))
xlabel('time'); 
ylabel('imu acc');
title('IMU Compensated Accelerometer Measurement in Y with time');
subplot(3,2,4)
hist(imu_acc(:,2) - imu_acc_mean_xyz(2), 500)
title('Noise Distribution of IMU Acc in Y');

subplot(3,2,5)
plot(imu_acc(:, 3))
xlabel('time'); 
ylabel('imu acc');
title('IMU Compensated Accelerometer Measurement in Z with time');
subplot(3,2,6)
hist(imu_acc(:,3) - imu_acc_mean_xyz(3), 500)
title('Noise Distribution of IMU Acc in Z');

%% imu_vel (angular rate) x,y,z with time respectively
imu_vel_mean_xyz = mean(imu_vel, 1);
imu_vel_std_xyz = std(imu_vel, 0, 1);

figure(3);
subplot(3,2,1)
plot(imu_vel(:, 1))
xlabel('time'); 
ylabel('imu vel');
title('IMU Compensated Angular Rate in X with time');
subplot(3,2,2)
hist(imu_vel(:,1) - imu_vel_mean_xyz(1), 500)
title('Noise Distribution of IMU Vel in X');

subplot(3,2,3)
plot(imu_vel(:, 2))
xlabel('time'); 
ylabel('imu vel');
title('IMU Compensated Angular Rate in Y with time');
subplot(3,2,4)
hist(imu_vel(:,2) - imu_vel_mean_xyz(2), 500)
title('Noise Distribution of IMU Vel in Y');

subplot(3,2,5)
plot(imu_vel(:, 3))
xlabel('time'); 
ylabel('imu vel');
title('IMU Compensated Angular Rate in Z with time');
subplot(3,2,6)
hist(imu_vel(:,3) - imu_vel_mean_xyz(3), 500)
title('Noise Distribution of IMU Vel in Z');
