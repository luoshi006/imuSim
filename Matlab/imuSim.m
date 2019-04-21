close all
clear
clc

%% param
freq = 100;         % 100Hz ~ 0.01s
SampleTime = 60;    % 1 min
len = freq * SampleTime;
% Gyro setting
gbx = 0.;     gby = 0;        gbz = 0;  % const bias
gwx = 0.075;    gwy = 0.078;    gwz = 0.079;  % angle random walk, deg/sqrt(s)
gix = 0.10;     giy = 0.009;    giz = 0.012;  % bias instability, deg/s
gtau = 10;  % Correlation time
% Acc setting
abx = 0.05; aby = 0; abz = 0;               % const bias
awx = 0.0011; awy = 0.0011; awz = 0.0020;   % velocity random walk, m/s^2/sqrt(s)
aix = 3E-4; aiy = 2.8E-4; aiz = 2.6E-4;     % bias instability, m/s^2
atau = 10;  % Correlation time

r2d = 180/pi;   % rad -> deg
g = 9.80665;
%% Generate IMU Data
dt = 1/freq;
time = 0:dt:dt*(len-1);
gyrox = zeros(1,len);  
gyroy = zeros(1,len);  
gyroz = zeros(1,len);
accx = zeros(1,len) + 0.01;
accy = zeros(1,len);
accz = zeros(1,len) - g;

for i=1:len
    if (i == 1)
        Rgx(i) = normrnd(0,sqrt(dt/gtau)*gix);
        Rgy(i) = normrnd(0,sqrt(dt/gtau)*giy);
        Rgz(i) = normrnd(0,sqrt(dt/gtau)*giz);
        Rax(i) = normrnd(0,sqrt(dt/atau)*aix);
        Ray(i) = normrnd(0,sqrt(dt/atau)*aiy);
        Raz(i) = normrnd(0,sqrt(dt/atau)*aiz);
    else
        Rgx(i) = Rgx(i-1) + normrnd(0,sqrt(dt/gtau)*gix);
        Rgy(i) = Rgy(i-1) + normrnd(0,sqrt(dt/gtau)*giy);
        Rgz(i) = Rgz(i-1) + normrnd(0,sqrt(dt/gtau)*giz);
        Rax(i) = Rax(i-1) + normrnd(0,sqrt(dt/atau)*aix);
        Ray(i) = Ray(i-1) + normrnd(0,sqrt(dt/atau)*aiy);
        Raz(i) = Raz(i-1) + normrnd(0,sqrt(dt/atau)*aiz);
    end
    gyrox_n(i) = gyrox(i) + gbx + normrnd(0,gwx/sqrt(dt)) + Rgx(i);
    gyroy_n(i) = gyroy(i) + gby + normrnd(0,gwy/sqrt(dt)) + Rgy(i);
    gyroz_n(i) = gyroz(i) + gbz + normrnd(0,gwz/sqrt(dt)) + Rgz(i);
    
    accx_n(i) = accx(i) + abx + normrnd(0,awx/sqrt(dt)) + Rax(i);
    accy_n(i) = accy(i) + aby + normrnd(0,awy/sqrt(dt)) + Ray(i);
    accz_n(i) = accz(i) + abz + normrnd(0,awz/sqrt(dt)) + Raz(i);
end

%% Ins
% function 41
Cnb = zeros(3, 3, len); Cnb(:,:,1) = eye(3);
Ct = zeros(3,3,len);    Ct(:,:,1) = eye(3); % true
roll = zeros(1,len);
pitch = zeros(1,len);
yaw = zeros(1,len);
Vn = zeros(3,len); Pn = zeros(3,len);
Vt = zeros(3,len); Pt = zeros(3,len);

for i = 2:len
    % calc the DCM 
    sig = norm([gyrox_n(i), gyroy_n(i), gyroz_n(i)])*dt;
    if sig < 0.00000000001
        Ct(:,:,i) = Ct(:,:,i-1);
    else
        B = [0, -gyroz_n(i)*dt, gyroy_n(i)*dt;
             gyroz_n(i)*dt, 0, -gyrox_n(i)*dt;
             -gyroy_n(i)*dt, gyrox_n(i)*dt, 0;];
        Cnb(:,:,i) = Cnb(:,:,i-1) * (eye(3) + sin(sig)/sig*B + (1-cos(sig))/sig^2 * B^2);
        [yaw(i), pitch(i), roll(i)] = dcm2angle( Cnb(:,:,i) );
    end
    
    % calc true DCM 
    sig = norm([gyrox(i), gyroy(i), gyroz(i)])*dt;
    if sig < 0.00000000001
        Ct(:,:,i) = Ct(:,:,i-1);
    else
        B = [0, -gyroz(i)*dt, gyroy(i)*dt;
             gyroz(i)*dt, 0, -gyrox(i)*dt;
             -gyroy(i)*dt, gyrox(i)*dt, 0;];
        Ct(:,:,i) = Ct(:,:,i-1) * (eye(3) + sin(sig)/sig*B + (1-cos(sig))/sig^2 * B^2);
    end
    
    % calc position
    An = Cnb(:,:,i)*[accx_n(i); accy_n(i); accz_n(i) + g];
    Vn(:,i) = Vn(:,i-1)+ An*dt;
    Pn(:,i) = Pn(:,i-1) + Vn(:,i)*dt;    % why not 1/2 * acc * dt^2
    
    % calc true postion 
    At = Ct(:,:,i)*[accx(i); accy(i); accz(i) + g];
    Vt(:,i) = Vt(:,i-1)+ At*dt;
    Pt(:,i) = Pt(:,i-1) + Vt(:,i)*dt;
end


%% display
figure
subplot(3,1,1)
plot(time, gyrox_n);
ylabel('gyro x'); title('Gyro sim data')
subplot(3,1,2)
plot(time, gyroy_n);
ylabel('gyro y')
subplot(3,1,3)
plot(time, gyroz_n)
ylabel('gyro z'); xlabel('time (s)');

figure
subplot(3,1,1)
plot(time, accx_n);
ylabel('acc x'); title('Acc sim data')
subplot(3,1,2)
plot(time, accy_n);
ylabel('acc y')
subplot(3,1,3)
plot(time, accz_n);
ylabel('acc z'); xlabel('time (s)');

figure
subplot(3,1,1)
plot(time, roll*r2d);
ylabel('roll'); title('Euler data')
subplot(3,1,2)
plot(time, pitch*r2d);
ylabel('pitch')
subplot(3,1,3)
plot(time, yaw*r2d);
ylabel('yaw'); xlabel('time (s)');

figure
plot3(Pn(1,:),Pn(2,:),Pn(3,:),'.'); hold on;
plot3(Pt(1,:),Pt(2,:),Pt(3,:),'.','MarkerSize',12);
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; legend('Position with noise','Position true')

