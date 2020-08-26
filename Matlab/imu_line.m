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
gravity = 9.80665;
s_acc = 1.077;
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
grid on;
hold off;
axis(2) = subplot(2,1,2);
hold on;
plot(time, acc.x * gravity * s_acc, 'r.');
plot(time, acc.y * gravity * s_acc, 'g.');
plot(time, acc.z * gravity * s_acc, 'b.');
legend('x', 'y', 'z');
xlabel('time (s)');
ylabel('acc (g)')
title('Acc');
grid on;
hold off;

%% process

% AHRS = MadgwickAHRS('SamplePeriod', 0.032, 'Beta', 0.05);
AHRS = MahonyAHRS('SamplePeriod', 0.032, 'Kp', 2.5, 'Ki', 0.05);

len = length(time);
quat = zeros(len, 4);
gyro_bias = zeros(len, 3);
for t = 1:len
    %AHRS.UpdateIMU([gyro.x(t), gyro.y(t), gyro.z(t)+0.13034]*deg2rad, [acc.x(t), acc.y(t), acc.z(t)]);
    AHRS.Update( [gyro.x(t), gyro.y(t), gyro.z(t)]*deg2rad ...
               , [acc.x(t), acc.y(t), acc.z(t)] ...
               , [1, 0, 0]);
    quat(t,:) = AHRS.Quaternion;
    gyro_bias(t,:) = AHRS.w_b;
    corrected_gyro(t,:) = [gyro.x(t), gyro.y(t), gyro.z(t)] - gyro_bias(t,:)*rad2deg;
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

figure('Name', 'corrected gyro');
hold on;
plot(time, corrected_gyro(:,1), 'r');
plot(time, corrected_gyro(:,2), 'g');
plot(time, corrected_gyro(:,3), 'b');
title('corrected gyro');
xlabel('Time (s)');
ylabel('Gyro Bias (deg/s)');
legend('wb_x', 'wb_y', 'wb_z');
hold off; grid on;

%% calculate position
% sequence of states is: quaternion, velocity, position, gyrobias, accbias


mean_acc = zeros(len, 1);   % norm of acc
mean_gyro= zeros(len, 1);
M2_acc = zeros(len, 1);
M2_gyro= zeros(len, 1);
vibe_acc = zeros(len, 1);
vibe_gyro= zeros(len, 1);

for index = 1:len
    
    % read IMU measurements
    current_time = time(index);
    current_accel = [acc.x(index), acc.y(index), acc.z(index)]' * gravity * s_acc;
%   (4x1 quaternion, 3x1 velocity, 3x1 position, 3x1 delAng bias, 3x1 delVel bias)
    states(1:4) = quat(index,:);
    states(11:13) = gyro_bias(index,:);
    states(14:16) = zeros(3,1);
    if index == 1        
        states(5:10) = zeros(6,1);
        mean_acc(index) = norm(current_accel);
        mean_gyro(index) = norm([gyro.x(index), gyro.y(index), gyro.z(index)]);
    else     
        dt_imu = current_time - last_time;
        delVel = (0.5 * (current_accel + last_accel)- states(14:16)) * dt_imu;
        % Remove sensor bias errors
        delVel = delVel; % - states(14:16);
        % Calculate the body to nav cosine matrix
        Tbn = Quat2Tbn(states(1:4));
        Tnb = transpose(Tbn);
        % transform body delta velocities to delta velocities in the nav frame
        delVelNav = Tbn * delVel - [0;0;gravity] * dt_imu;
        % take a copy of the previous velocity
        prevVel = states(5:7);
        % Sum delta velocities to get the velocity
        states(5:7) = states(5:7) + delVelNav(1:3);
        % integrate the velocity vector to get the position using trapezoidal integration
        pos_int = 0.5 * dt_imu * (prevVel + states(5:7));
        states(8:10) = states(8:10) + pos_int;
        pos_err = -transpose(pos_int);
        %% zupt from vibe
        zupt = false;
        delta_acc = norm(current_accel) - mean_acc(index-1);
        mean_acc(index) = mean_acc(index-1) + delta_acc/index;
        M2_acc(index) = M2_acc(index-1) + delta_acc*(norm(current_accel) - mean_acc(index));
        vibe_acc(index) = sqrt(M2_acc(index)/(index-1));
        
        current_gyro = [gyro.x(index), gyro.y(index), gyro.z(index)];
        delta_gyro = norm(current_gyro) - mean_gyro(index-1);
        mean_gyro(index) = mean_gyro(index-1) + delta_gyro/index;
        M2_gyro(index) = M2_gyro(index-1) + delta_gyro*(norm(current_gyro)-mean_gyro(index));
        vibe_gyro(index) = sqrt(M2_gyro(index)/(index-1));
        
        if (vibe_acc(index) < 0.005)
            zupt = true;
            w = 0.8;
            states(8:10) = states(8:10) + pos_err' * w;
            states(5:7)  = states(5:7) - delVelNav(1:3) * w;
            states(14:16) = states(14:16) + (Tnb * (pos_err))';
        end
    end
    last_time = current_time;
    last_accel = current_accel;
    
    position(index,:) = states(8:10);
    velocity(index,:) = states(5:7);
    bias_acc(index,:) = states(14:16);
end
figure('Name', 'position');
subplot(2,1,1);
hold on;
plot(time, position(:,1), 'r');
plot(time, position(:,2), 'g');
plot(time, position(:,3), 'b');
title('position');
xlabel('Time (s)');
ylabel('position (m)');
legend('p_x', 'p_y', 'p_z');
hold off; grid on;
subplot(2,1,2)
plot(time, velocity(:,1), 'r'); hold on;
plot(time, velocity(:,2), 'g');
plot(time, velocity(:,3), 'b');
title('velocity');
xlabel('Time (s)');
ylabel('velocity (m)');
legend('v_x', 'v_y', 'v_z');
hold off; grid on;

figure('Name', 'bias')
plot(time, bias_acc(:,1), 'r'); hold on;
plot(time, bias_acc(:,2), 'g');
plot(time, bias_acc(:,3), 'b');
legend('b_x', 'b_y', 'b_z');