close all
clear
clc

%% import data
addpath('quaternion_library');
addpath('data');
load static_2.mat
% load circle_11.mat
% load circle_12.mat
% load circle_13.mat

time = 0.032* [1:300];
deg2rad = pi/180;
rad2deg = 1/deg2rad;
% gyro.x = gyro.x*deg2rad;
% gyro.y = gyro.y*deg2rad;
% gyro.z = gyro.z*deg2rad;

figure('Name', 'sensor_data');
axis(1) = subplot(2,1,1);
hold on;
plot(time, gyro.x, 'r');
plot(time, gyro.y, 'g');
plot(time, gyro.z, 'b');
legend('x', 'y', 'z');
xlabel('time (s)');
ylabel('omega (deg/s)')
title('gyro');
hold off;
axis(2) = subplot(2,1,2);
hold on;
plot(time, acc.x, 'r.');
plot(time, acc.y, 'g.');
plot(time, acc.z, 'b.');
legend('x', 'y', 'z');
xlabel('time (s)');
ylabel('acc (g)')
title('Acc');
hold off;

%% process

% AHRS = MadgwickAHRS('SamplePeriod', 0.032, 'Beta', 0.05);
AHRS = MahonyAHRS('SamplePeriod', 0.032, 'Kp', 2.5, 'Ki', 0.05);

quat = zeros(length(time), 4);
gyro_bias = zeros(length(time), 3);
for t=1:length(time)
    %AHRS.UpdateIMU([gyro.x(t), gyro.y(t), gyro.z(t)+0.13034]*deg2rad, [acc.x(t), acc.y(t), acc.z(t)]);
    AHRS.Update( [gyro.x(t), gyro.y(t), gyro.z(t)+0.13034]*deg2rad ...
               , [acc.x(t), acc.y(t), acc.z(t)] ...
               , [1, 0, 0]);
    quat(t,:) = AHRS.Quaternion;
    gyro_bias(t,:) = AHRS.w_b;
end
euler = quatern2euler(quaternConj(quat)) * (180/pi);

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('roll', 'pitch', 'yaw');
hold off; grid on;

figure('Name', 'Gyro Bias');
hold on;
plot(time, gyro_bias(:,1)*rad2deg, 'r');
plot(time, gyro_bias(:,2)*rad2deg, 'g');
plot(time, gyro_bias(:,3)*rad2deg, 'b');
title('Gyro Bias');
xlabel('Time (s)');
ylabel('Gyro Bias (deg/s)');
legend('wb_x', 'wb_y', 'wb_z');
hold off; grid on;


