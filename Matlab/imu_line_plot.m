close all;

set(0,'DefaultFigureWindowStyle','docked');

%% sensor
figure('Name', 'sensor_data');
axis(1) = subplot(2,1,1); hold on;
plot(time, gyro.x, 'r');
plot(time, gyro.y, 'g');
plot(time, gyro.z, 'b');
legend('x', 'y', 'z');
title('gyro'); xlabel('time (s)'); ylabel('omega (deg/s)')
grid on; hold off;

axis(2) = subplot(2,1,2); hold on;
plot(time, acc.x * gravity * s_acc, 'r.');
plot(time, acc.y * gravity * s_acc, 'g.');
plot(time, acc.z * gravity * s_acc, 'b.');
legend('x', 'y', 'z');
title('Acc'); xlabel('time (s)'); ylabel('acc (g)')
grid on;hold off;
linkaxes(axis, 'x');

figure('Name', 'Euler Angles'); hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('roll', 'pitch', 'yaw');
hold off; grid on;

figure('Name', 'Gyro');
axisg(1) = subplot(2,1,1); hold on;
plot(time, gyro_bias(:,1)*rad2deg, 'r');
plot(time, gyro_bias(:,2)*rad2deg, 'g');
plot(time, gyro_bias(:,3)*rad2deg, 'b');
title('Gyro Bias'); xlabel('Time (s)'); ylabel('Gyro Bias (deg/s)');
legend('wb_x', 'wb_y', 'wb_z');
hold off; grid on;

axisg(2) = subplot(2,1,2); hold on;
plot(time, corrected_gyro(:,1), 'r');
plot(time, corrected_gyro(:,2), 'g');
plot(time, corrected_gyro(:,3), 'b');
title('corrected gyro'); xlabel('Time (s)'); ylabel('Gyro (deg/s)');
legend('wb_x', 'wb_y', 'wb_z');
hold off; grid on;
linkaxes(axisg, 'x');

figure('Name', 'acc');
axisa(1) = subplot(2,1,1); hold on;
plot(time, bias_acc(:,1), 'r');
plot(time, bias_acc(:,2), 'g');
plot(time, bias_acc(:,3), 'b');
title('Acc Bias'); xlabel('Time (s)'); ylabel('Acc Bias (m/s^2)');
legend('b_x', 'b_y', 'b_z');
hold off; grid on;
axisa(2) = subplot(2,1,2); hold on;
plot(time, correct_acc(:,1), 'r');
plot(time, correct_acc(:,2), 'g');
plot(time, correct_acc(:,3), 'b');
title('correct acc'); xlabel('Time (s)'); ylabel('Acc (m/s^2)');
legend('a_x', 'a_y', 'a_z');
hold off; grid on;
linkaxes(axisa, 'x');

figure('Name', 'position');
subplot(2,1,1); hold on;
plot(time, position(:,1), 'r');
plot(time, position(:,2), 'g');
plot(time, position(:,3), 'b');
title('position'); xlabel('Time (s)'); ylabel('position (m)');
legend('p_x', 'p_y', 'p_z');
hold off; grid on;
subplot(2,1,2); hold on;
plot(time, velocity(:,1), 'r');
plot(time, velocity(:,2), 'g');
plot(time, velocity(:,3), 'b');
title('velocity'); xlabel('Time (s)'); ylabel('velocity (m)');
legend('v_x', 'v_y', 'v_z');
hold off; grid on;

