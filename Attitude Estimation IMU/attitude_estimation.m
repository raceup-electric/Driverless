% Estimaiton of Roll and Pitch using IMU data 

% Avg. time step
dt = 0.0185;
table = readtable("imu_data.csv");
Gx = table.Gx;
Gy = table.Gy;
Gz = table.Gz;
Ax = table.Ax;
Ay = table.Ay;
Az = table.Az;
t = table.t;

% Convert gyroscope measurements to radians
Gx_rad = Gx * pi / 180.0;
Gy_rad = Gy * pi / 180.0;
Gz_rad = Gz * pi / 180.0;

% 1) Accelerometer only
phi_hat_acc   = atan2(Ay, sqrt(Ax .^ 2 + Az .^ 2)); 
theta_hat_acc = atan2(-Ax, sqrt(Ay .^ 2 + Az .^ 2));

% 2) Gyroscope only
phi_hat_gyr   = zeros(1, length(t));
theta_hat_gyr = zeros(1, length(t));

for i = 2:length(t)
   gx = Gx_rad(i);
   gy = Gy_rad(i);
   gz = Gz_rad(i);
   
   phi_hat   = phi_hat_gyr(i - 1);
   theta_hat = theta_hat_gyr(i - 1);
    
   phi_hat_gyr(i)   = phi_hat   + dt * (gx + sin(phi_hat) * tan(theta_hat) * gy + cos(phi_hat) * tan(theta_hat) * gz);
   theta_hat_gyr(i) = theta_hat + dt * (cos(phi_hat) * gy - sin(phi_hat) * gz);
end

% 3) Complimentary Filter
alpha = 0.1;

phi_hat_complimentary   = zeros(1, length(t));
theta_hat_complimentary = zeros(1, length(t));

for i=2:length(t)
    gx = Gx_rad(i);
    gy = Gy_rad(i);
    gz = Gz_rad(i);
   
    phi_hat   = phi_hat_complimentary(i - 1);
    theta_hat = theta_hat_complimentary(i - 1);
    
    phi_hat_gyr_comp   = phi_hat   + dt * (gx + sin(phi_hat) * tan(theta_hat) * gy + cos(phi_hat) * tan(theta_hat) * gz);
    theta_hat_gyr_comp = theta_hat + dt * (cos(phi_hat) * gy - sin(phi_hat) * gz);
       
    phi_hat_complimentary(i)   = (1 - alpha) * phi_hat_gyr_comp   + alpha * phi_hat_acc(i);
    theta_hat_complimentary(i) = (1 - alpha) * theta_hat_gyr_comp + alpha * theta_hat_acc(i);    
end

% 4) Kalman Filter
A = [1 -dt 0 0; 0 1 0 0; 0 0 1 -dt; 0 0 0 1];         % A = [1 -dt 0  0]    B = [dt 0 0 0       C = [1 0 0 0
B = [dt 0 0 0; 0 0 dt 0]';                            %      0  1  0  0]         0  0 dt 0           0 0 1 -1]
C = [1 0 0 0; 0 0 1 0];                               %      0  0  1 -dt
P = eye(4);                                           %      0  0  0  1]
Q = eye(4) * 0.01; 
R = eye(2) * 10; 
state_estimate = [0 0 0 0]'; 

phi_hat_kalman    = zeros(1, length(t));
bias_phi_kalman   = zeros(1, length(t));
theta_hat_kalman  = zeros(1, length(t));
bias_theta_kalman = zeros(1, length(t));

for i=2:length(t)
    
    gx = Gx_rad(i);
    gy = Gy_rad(i);
    gz = Gz_rad(i);
   
    phi_hat   = phi_hat_kalman(i - 1);
    theta_hat = theta_hat_kalman(i - 1);
    
    phi_dot   = gx + sin(phi_hat) * tan(theta_hat) * gy + cos(phi_hat) * tan(theta_hat) * gz;
    theta_dot = cos(phi_hat) * gy - sin(phi_hat) * gz;
          
    % Predict
    state_estimate = A * state_estimate + B * [phi_dot, theta_dot]';
    P = A * P * A' + Q;
    
    % Update
    measurement = [phi_hat_acc(i) theta_hat_acc(i)]';
    y_tilde = measurement - C * state_estimate; % e(k) 
    S = R + C * P * C'; % N(K)
    K = P * C' * (S^-1); % L(k) 
    state_estimate = state_estimate + K * y_tilde; % x_hat(k|k) stima filtrata dello stato
    P = (eye(4) - K * C) * P; % P(k|k) varianza dell'errore di stima filtrata,
    
    phi_hat_kalman(i)    = state_estimate(1);
    bias_phi_kalman(i)   = state_estimate(2);
    theta_hat_kalman(i)  = state_estimate(3);
    bias_theta_kalman(i) = state_estimate(4);
    
end

% Convert all estimates to degrees
phi_hat_acc = phi_hat_acc * 180.0 / pi;
theta_hat_acc = theta_hat_acc * 180.0 / pi;
phi_hat_gyr = phi_hat_gyr * 180.0 / pi;
theta_hat_gyr = theta_hat_gyr * 180.0 / pi;
phi_hat_complimentary = phi_hat_complimentary * 180.0 / pi;
theta_hat_complimentary = theta_hat_complimentary * 180.0 / pi;
phi_hat_kalman = phi_hat_kalman * 180.0 / pi;
theta_hat_kalman = theta_hat_kalman * 180.0 / pi;

% Plots
subplot(2, 1, 1);
plot(t, phi_hat_complimentary, t, phi_hat_acc, t, phi_hat_gyr, t, phi_hat_kalman)
legend('Complimentary', 'Accelerometer', 'Gyro', 'Kalman')
xlabel('Time (s)')
ylabel('Angle (Degrees)')
title('Roll')

subplot(2, 1, 2);
plot(t, theta_hat_complimentary, t, theta_hat_acc, t, theta_hat_gyr, t, theta_hat_kalman)
legend('Complementary', 'Accelerometer', 'Gyro', 'Kalman')
xlabel('Time (s)')
ylabel('Angle (Degrees)')
title('Pitch')
