% rtk gps data analysis code
clear;
clc;

%% Stationary data 
%---------------uncomment one to choose dataset--------------
data_sta = readtable('stationary_clear_data.csv', 'HeaderLines',1); 
%data_sta = readtable('stationary_reflect_data.csv', 'HeaderLines',1);
%------------------------------------------------------------
fix_quality_sta = table2array(data_sta(:, 12));
data_sta = table2array(data_sta(:, 8:9)); 
data_sta(:,1) = data_sta(:,1) - min(data_sta(:, 1));
data_sta(:,2) = data_sta(:,2) - min(data_sta(:, 2));

utm_easting_ave1 = sum(data_sta(:, 1)) / size(data_sta,1);
utm_northing_ave1 = sum(data_sta(:, 2)) / size(data_sta,1);

utm_easting_var1 = std(data_sta(:,1));
utm_northing_var1 = std(data_sta(:,2));

fprintf('----Stationary data standard deviation:---- \n');
fprintf('utm_easting standard deviation = %f\n', utm_easting_var1);
fprintf('utm_northing standard deviation = %f\n', utm_northing_var1);

figure(1);
% normal points plot
subplot(1,2,1);
scatter(data_sta(:, 1), data_sta(:, 2), 5, 'b');
hold on
scatter(utm_easting_ave1, utm_northing_ave1, 10, 'filled', 'r');
xlabel('utm easting'); 
ylabel('utm northing');
title('UTM stationary data');
hold off

% the most recent data will be plotted with larger circles
subplot(1,2,2);
sz = linspace(2, 60, size(data_sta, 1)); 
scatter(data_sta(:, 1), data_sta(:, 2), sz, 'b');
hold on
scatter(utm_easting_ave1, utm_northing_ave1, 30, 'filled', 'r');
xlabel('utm easting'); 
ylabel('utm northing');
title('UTM stationary data (w/ time series)');
hold off

% plot error type
utm_error_easting = data_sta(:, 1) - utm_easting_ave1;
utm_error_northing = data_sta(:, 2) - utm_northing_ave1;

fprintf('----Stationary data mean error:---- \n');
fprintf('utm_easting mean error =');
disp(mean(utm_error_easting));
fprintf('utm_northing mean error =');
disp(mean(utm_error_northing));

figure(2)
subplot(1,2,1);
histogram2(utm_error_easting, utm_error_northing, 80, 'FaceColor','flat');
title('Error Distribution of UTM Easting & Northing in 3D');
xlabel('utm easting'); 
ylabel('utm northing');
subplot(1,2,2);
histogram2(utm_error_easting, utm_error_northing, 80, 'DisplayStyle','tile', 'ShowEmptyBins','on');   
title('Error Distribution of UTM Easting & Northing');
xlabel('utm easting'); 
ylabel('utm northing');

figure(3)
subplot(1,2,1);
histogram(utm_error_easting, 100);
title('Error Distribution of UTM Easting');
xlabel('utm easting'); 
subplot(1,2,2);
histogram(utm_error_northing, 100);
title('Error Distribution of UTM Northing');
xlabel('utm northing'); 

figure(4);
histogram(fix_quality_sta);
title('Stationary data fix quality');
xlabel('fix quality'); 

%% Moving data
%---------------uncomment one to choose dataset--------------
data_walk = readtable('move_clear_data.csv', 'HeaderLines',1);
%data_walk = readtable('move_reflect_data.csv', 'HeaderLines',1);
%------------------------------------------------------------
fix_quality_mov = table2array(data_walk(:, 12));
data_walk = table2array(data_walk(:, 8:9));
data_walk(:, 1) = data_walk(:, 1) - min(data_walk(:, 1));
data_walk(:, 2) = data_walk(:, 2) - min(data_walk(:, 2));

figure(5);
% normal points plot
subplot(1,2,1);
scatter(data_walk(:, 1), data_walk(:, 2), 20, 'b');
grid on
xlabel('utm easting'); 
ylabel('utm northing');
title('UTM moving data');

% the most recent data will be plotted with larger circles
subplot(1,2,2);
sz = linspace(5, 100, size(data_walk, 1));
scatter(data_walk(:, 1), data_walk(:, 2), sz, 'b');
grid on
hold on
xlabel('utm easting'); 
ylabel('utm northing');
title('UTM moving data');

figure(6);
histogram(fix_quality_mov);
title('Moving data fix quality');
xlabel('fix quality'); 

%% fitting each segment of the moving points to a straight line 
% by manually choose the starting and ending points of each sides
% and calculating the error
% NOTICE: this part is only work for 'move_clear_data.csv' dataset!!!

side1 = data_walk(93:141, :); % split the data
side2 = data_walk(141:240, :);
side3 = data_walk(240:292, :);
side4 = [data_walk(292:end, :); data_walk(1:93, :)];

[yy1, total_error1] = FitAndError(side1); % fitting
[yy2, total_error2] = FitAndError(side2);
[yy3, total_error3] = FitAndError(side3);
[yy4, total_error4] = FitAndError(side4);

figure(7);
subplot(1,2,1);
scatter(data_walk(:, 1), data_walk(:, 2), 20, 'b');
grid on
xlabel('utm easting'); 
ylabel('utm northing');
title('UTM moving data');

subplot(1,2,2);
plot(side1(:, 1), yy1, 'linewidth', 2);
hold on
grid on
plot(side2(:, 1), yy2, 'linewidth', 2);
plot(side3(:, 1), yy3, 'linewidth', 2);
plot(side4(:, 1), yy4, 'linewidth', 2);
xlabel('utm easting'); 
ylabel('utm northing');
title('ROVER moving data w/ fitting straight line');

error = [total_error1, total_error2, total_error3, total_error4]; % error
figure(8);
plot(error);
xlabel('number of points'); 
ylabel('error');
title('UTM moving measurement points error');

fprintf('----ROVER moving data mean error:---- \n');
disp(mean(abs(error)));

figure(9)
hist(error, 100);
xlabel('error');
title('Error distribution');

%% fitting each segment of the moving points to a straight line 
% by manually choose the starting and ending points of each sides
% and calculating the error
% NOTICE: this part is only work for 'stationary_reflect_data.csv' dataset!!!

side1 = [data_walk(222:end, :); data_walk(1:49, :)]; % split the data
side2 = data_walk(172:222, :);
side3 = data_walk(172:195, :);
side4 = data_walk(49:195, :);

[yy1, total_error1] = FitAndError(side1); % fitting
[yy2, total_error2] = FitAndError(side2);
[yy3, total_error3] = FitAndError(side3);
[yy4, total_error4] = FitAndError(side4);

figure(7);
subplot(1,2,1);
scatter(data_walk(:, 1), data_walk(:, 2), 20, 'b');
grid on
xlabel('utm easting'); 
ylabel('utm northing');
title('UTM moving data');

subplot(1,2,2);
plot(side1(:, 1), yy1, 'linewidth', 2);
hold on
grid on
plot(side2(:, 1), yy2, 'linewidth', 2);
plot(side3(:, 1), yy3, 'linewidth', 2);
plot(side4(:, 1), yy4, 'linewidth', 2);
xlabel('utm easting'); 
ylabel('utm northing');
title('ROVER moving data w/ fitting straight line');

error = [total_error1, total_error2, total_error3, total_error4]; % error
figure(8);
plot(error);
xlabel('number of points'); 
ylabel('error');
title('UTM moving measurement points error');

fprintf('----ROVER moving data mean error:---- \n');
disp(mean(abs(error)));

figure(9)
hist(error, 100);
xlabel('error');
title('Error distribution');