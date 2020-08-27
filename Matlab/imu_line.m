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

%% calculate position
% sequence of states is: quaternion, velocity, position, gyrobias, accbias


mean_acc = zeros(len, 1);   % norm of acc
mean_gyro= zeros(len, 1);
M2_acc = zeros(len, 1);
M2_gyro= zeros(len, 1);
vibe_acc = zeros(len, 1);
vibe_gyro= zeros(len, 1);
correct_acc = zeros(len, 3);

for index = 1:len

    % read IMU measurements
    current_time = time(index);
    current_accel = [acc.x(index), acc.y(index), acc.z(index)]' * gravity * s_acc;
%   (4x1 quaternion, 3x1 velocity, 3x1 position, 3x1 delAng bias, 3x1 delVel bias)
    states(1:4) = quat(index,:);
    states(11:13) = gyro_bias(index,:);
    % states(14:16) = zeros(3,1);
    if index == 1
        states(5:10) = zeros(6,1);
        mean_acc(index) = norm(current_accel);
        mean_gyro(index) = norm([gyro.x(index), gyro.y(index), gyro.z(index)]);
        zupt_position = states(8:10);
        states(14) = mean(acc.x)* gravity * s_acc;
        states(15) = mean(acc.y)* gravity * s_acc;
        states(16) = mean(acc.z)* gravity * s_acc - gravity;
        correct_acc(index, :) = current_accel' - states(14:16);
        disp(['acc bias: ', num2str(mean(acc.x)* gravity * s_acc),', ', num2str(mean(acc.y)* gravity * s_acc),', ', num2str(mean(acc.z)* gravity * s_acc - gravity)]);

    else
        correct_acc(index, :) = current_accel' - states(14:16);

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
        pos_err = -(pos_int);
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
            w = 0.9;

            pos_err = zupt_position - states(8:10);
            states(8:10) = states(8:10) + pos_err * w;
            states(5:7)  = states(5:7)  + pos_err * w * w;

            vel_err = -states(5:7);
            states(5:7)  = states(5:7)  + vel_err * w;

            states(14:16) = states(14:16) + (Tnb * (pos_err'))' * 0.2;
        else
            disp('ZUPT: update zupt position');
            zupt_position = states(8:10);
        end
    end
    last_time = current_time;
    last_accel = current_accel;

    position(index,:) = states(8:10);
    velocity(index,:) = states(5:7);
    bias_acc(index,:) = states(14:16);
end

imu_line_plot;