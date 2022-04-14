% Example KF implementation
clear; close all; clc;
rng(1337)

%% Our Code: Read in camera/lidar/imu csv data
cameraData = readmatrix("Mar30Trial4/xpos_trial4_3-30.csv");
cameraData = cameraData(60:109,:);
cameraData(:, 1) = cameraData(:, 1) - cameraData(1, 1); %making time start at 0
cameraData(:, 2) = cameraData(:, 2) * -1; %invert camera measurements. since its all negative changes in position. should be positive.
% disp(cameraData)

lidarData = readmatrix("Mar30Trial4/lidar.txt");
lidarData = lidarData(60:end,:);
lidarData(:, 3) = lidarData(:, 3) - lidarData(1, 3);
% disp(lidarData);
lidar1 = lidarData(lidarData(:, 1) == 1, :);
lidar2 = lidarData(lidarData(:, 1) == 2, :);
% disp(lidar1);
% disp(lidar2);

imuData = readmatrix("Mar30Trial4/imu_normal.txt");
imuData = imuData(551:end, :);
imuData(:, 1) = imuData(:, 1) - imuData(1, 1);
%calculate trailing mean, clumping past 10 data points
% trail = 10;
% imuData(:, 2) = movmean(imuData(:, 2), [trail 0]);
% imuData(:, 3) = movmean(imuData(:, 3), [trail 0]);
% imuData(:, 4) = movmean(imuData(:, 4), [trail 0]);
% imuData(:, 5) = movmean(imuData(:, 5), [trail 0]);
% imuData(:, 6) = movmean(imuData(:, 6), [trail 0]);
% imuData(:, 7) = movmean(imuData(:, 7), [trail 0]);
% disp(imuData)


%% Define the initial states
% Here, we start at position zero with velocity +1m/s and absolute
% certainty about our initial state
x0 = [0 0.10824]';
P0 = diag([1e-4, 1e-6]);

% dt = 0.050;  % Update once per 0.050 second (camera freq)
dt = 0.10 % but its not actually always 0.10!!!! (camera timestamp hz)
t_end = 4.9;  % Simulate for ~4.9 seconds (duration of trial)

steps = t_end/dt;
% time = linspace(0,t_end,steps);
time = cameraData(:, 1);
% disp(time)

%% Define A, Q, and R matrices
sigma_a = 0.01;  % [m/s^2] Process noise standard deviation

A = [1, dt; 0, 1]; % state transition matrix

% Q quantifies the uncertainty added from process noise in the system
% Recall that our model assumes process noise in the state propagation:
%       x_k|k-1 = A_k * x_k-1 + w_k   (Equation 11 from the slides)
% Where w_k is a drawn from a zero-mean Guassian distribution with standard
% deviation sigma_a. This noise represents unmodeled accelerations. The
% intermediate matrix G models the effects that these accelerations have on
% our position and velocity states. In this way, sigma_a can be used as a
% tuning parameter to increase or decrease the amount of process noise
% modeled in our filter.
G = [0.5 * dt ^ 2 dt]';
Q = diag(G.^2) * sigma_a^2;

%% Define Measurement matrices (H, R)
H_camera = [0, dt]; % The camera is measuring CHANGE in position, which is essentially velocity.
H_lidar = [1, 0]; % The lidar measures position
H = [H_camera; H_lidar];

% ===================== Sigmas ====================
% R quantifies the uncertainty in our sensor measurements. Recall that
% measurement noise is modeled as:
%       z_k = H_k * x_k + r_k         (Equation 12 from the slides)
% Where r_k is drawn from a zero-mean Gaussian with standard deviation
% sigma_r. The R matrix captures this uncertainty for all sensor inputs.
% Since we have one sensor here, R is simply a scalar that equals sigma_r^2.
sigma_r_camera = 0.00692412970236608;  % [m] Measurement noise standard deviation <-- CHANGE THIS TO MATCH CAMERA STD

sigma_r_lidar = 0.05; %arbitrary value. change this !!!!

% The camera noise is sigma_r_camera, and the lidar noise is sigma_r_lidar. We have 2 measurements,
% so the measurement covariance is 2x2. In general, unless you have reason to know better, it is
% safe to assume the measurement noises are independent, hence the off-diagonal terms are 0.
R = diag([sigma_r_camera^2, sigma_r_lidar^2]); % 2x2 measurement covariance



%% Run the filter
%[x_hat, P_hat, time, x_true, z_k] = kalman(dt, t_end, x0, P0, A, H, Q, R);
% x_true = zeros([2,2]);
x_true = x0; % initialize truth to
x_hat = x0;
P_hat = zeros(2, 2, numel(time)); % pre-allocate
P_hat(:, :, 1) = P0; % initial covariance

LR = chol(R, 'lower'); % used for generating noise
LQ = chol(Q, 'lower');

%the x variables represent system state (like position, velocity, etc)
%the z variables represent sensor measurements
%z_k is the measurement from the sensor. We are faking it rn
%z_hat is predicted sensor measurement
%x_hat is predicted state based off previous state
%p_hat is covariance matrix which gets updated every iteration
z = zeros(2, numel(time));
z_hat = zeros(2, 1);
[~, last_imu_index] = min(abs(imuData(:, 1) - cameraData(2, 1)));
for k = 2:numel(time)
    %Propagate the truth state and add noise
    %disp(x_true(:, k-1))
    
    %finds the closest lidar timestamp to the camera timestamp
%     disp("Dealing with time")
%     disp(time(k))
    [~, closest_lidar1_index] = min(abs(lidar1(:, 3) - cameraData(k, 1)));
    [~, closest_lidar2_index] = min(abs(lidar2(:, 3) - cameraData(k, 1)));
    closest_lidar1_timestamp = lidar1(closest_lidar1_index, 3);
    closest_lidar2_timestamp = lidar2(closest_lidar2_index, 3);

    closest_lidar1_measurement = lidar1(closest_lidar1_index, 2);
    closest_lidar2_measurement = lidar2(closest_lidar2_index, 2);

%     disp("Closest lidar time stamp is")
%     disp(closest_lidar1_timestamp);
%     disp(closest_lidar2_timestamp);

%     disp("lidar data is ");
%     disp(closest_lidar1_measurement);
%     disp(closest_lidar2_measurement);


    %finding nearest imu timestamp
    [~, closest_imu_index] = min(abs(imuData(:, 1) - cameraData(k, 1)));
    closest_imu_timestamp = imuData(closest_imu_index, 1);
    closest_imu_measurement = imuData(closest_imu_index, :);
%     disp("closest imu timestamp is ");
%     disp(closest_imu_measurement);
%     disp(closest_imu_index);

%     disp("the second to recent one is ");
%     disp(imuData(last_imu_index, :));
%     disp(last_imu_index);

    rolling_imu_measurement = mean(imuData([last_imu_index:closest_imu_index], :));
%     disp(rolling_imu_measurement);
    
    x_true(:, k) = A * x_true(:, k-1) + LQ*randn(size(x0)); % should add process noise but this is OK

    % =========== Propagate the filter states ===========
    x_hat(:, k) = A * x_hat(:, k-1); % predicted state
    P_hat(:, :, k) = A * P_hat(:, :, k-1) * A' + Q; % estimated covariance of predicted state
    
    % =========== Generate a noisy measurement as a function of our true state ===========
    % The measurement vector is [camera; lidar]
    
    % Camera
%     z(1, k) = H_camera * x_true(:, k); % <-- CHANGE THIS TO WHATEVER THE CAMERA IS OUTPUTTING
    z(1, k) = cameraData(k, 2);
%     z(2, k) = H_lidar * x_true(:, k); % <-- CHANGE THIS TO WHATEVER THE LIDAR IS OUTPUTTING
    z(2, k) = 1.00 - closest_lidar2_measurement; %inverted because position = inverse of lidar distance away from front object

    % Usually we just do H * x_true, but I left this as two steps for clarity
    z(:, k) = z(:, k) + LR * randn(length(x0), 1); % measurement noise term, generated using cholesky decomp of measurement covariance
        
    % =================== Predicted measurements ===================
    z_hat(1) = H_camera * x_hat(:, k); %Predicted measurement is our DELTA position
    z_hat(2) = (H_lidar * x_hat(:, k)); %Predicted measurement is our position
    y = z(:, k) - z_hat; % we call this the "innovation" since it is essentially the new information
    
%     y(1) = 0;
%     y(2) = 0;

    % =================== Measurement update ===================
    S = H * P_hat(:, :, k) * H' + R;
    K = P_hat(:, :, k) * H' / S;
    
%     fprintf('Camera: %8.3f \t Lidar: %10.5f\n', z(1, k), z(2, k)) % print the actual measurements
    
    x_hat(:, k) = x_hat(:, k) + (K * y);
    P_hat(:, :, k) = (eye(size(P0)) - K * H) * P_hat(:, :, k);


    last_imu_index = closest_imu_index;
end

%% Plot results
set(groot, 'DefaultLineLinewidth', 2);
set(groot, 'DefaultLineMarkersize', 25);
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultAxesBox', 'on');
%set(0, 'DefaultFigureWindowStyle', 'normal')
set(0, 'DefaultFigureWindowStyle', 'docked')
set(groot, 'DefaultFigurePosition', [100 100 1200 500]);
set(0, 'DefaultAxesColor', 'white')


fig_prefix = 'baseline';  % Change this to prevent overwriting existing figures
fig_format = 'png';  % Format with which to save the figures
figdir = fullfile('..','imgs');
if (~exist(figdir, 'dir')), mkdir(figdir); end

fmt = @(x) strip(strip(sprintf('%f',x), '0'), 'right', '.');
sigstr = sprintf('$$\\sigma_a=%s$$, $$\\sigma_r=%s$$', fmt(sigma_a), fmt(sigma_r_camera));
sigstr_save = replace(sprintf('sigmaa=%s_sigmar=%s', fmt(sigma_a), fmt(sigma_r_camera)), '.','p');
mkdir(fullfile(figdir, sigstr_save))
get_figname = @(x) fullfile(sigstr_save, sprintf('%s.%s', x, fig_format));


% ============= Position =============
f = figure('name', 'Estimated Position'); hold on;
cmap = colormap('lines');
plot(time, x_true(1, :), 'k', 'linewidth', 3, 'DisplayName', 'Ideal'); %Changed "Truth" to "Ideal"
plot(time, x_hat(1, :), 'linewidth', 3, 'DisplayName', 'Kalman Filter');
plot(time, z(2, :), '.', 'linewidth', 1.5, 'DisplayName', 'Lidar Measurement');
legend('Location', 'northwest')
xlabel('Time (s)');
ylabel('Position (m)');
title(sprintf('Estimated Position (%s)', sigstr), 'interpreter', 'latex');
box on; grid on;
figname = get_figname('EstimatedPosition');
print(f, fullfile(figdir, figname), '-dpng', '-r300');

% ============= Velocity =============
f = figure('name', 'Estimated Velocity'); hold on;
plot(time, x_true(2, :), 'k', 'linewidth', 2, 'DisplayName', 'Truth');
plot(time, x_hat(2, :), 'linewidth', 2, 'DisplayName', 'Kalman Filter');
plot(time, z(1, :) / dt, '.', 'linewidth', 1.5, 'DisplayName', 'Camera Measurement / dt');
legend('Location', 'northwest')
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title(sprintf('Estimated Velocity (%s)', sigstr), 'interpreter', 'latex');
box on; grid on;

figname = get_figname('EstimatedVelocity');
saveas(f, fullfile(figdir, figname));

% Position Sigma Bounds / Error
sigma_position_bound = 3 * sqrt(squeeze(P_hat(1, 1, :)));
f = figure('name', 'Position Error'); hold on;
plot(time, x_hat(1, :) - x_true(1, :), 'linewidth', 2);
plot(time, sigma_position_bound, 'k', 'linewidth', 2);
plot(time, -sigma_position_bound, 'k', 'linewidth', 2);
xlabel('Time (s)');
ylabel('Position Error (m)');
title(sprintf('Position Error (%s)', sigstr), 'interpreter', 'latex');
legend('Position Error', 'Position 3 Sigma Bounds');
box on; grid on;
figname = get_figname('PositionError');
saveas(f, fullfile(figdir, figname));

% Velocity Sigma Bounds / Error
sigma_velocity_bound = 3 * sqrt(squeeze(P_hat(2, 2, :)));
f = figure('name', 'Velocity Error');
hold on;
plot(time, x_hat(2, :) - x_true(2, :), 'linewidth', 2);
plot(time, sigma_velocity_bound, 'k', 'linewidth', 2);
plot(time, -sigma_velocity_bound, 'k', 'linewidth', 2);
xlabel('Time (s)');
ylabel('Velocity Error (m/s)');
title('Velocity Error');
title(sprintf('Velocity Error (%s)', sigstr), 'interpreter', 'latex');
legend('Velocity Error', '3 Sigma Bound');
box on; grid on;
figname = get_figname('VelocityError');
saveas(f, fullfile(figdir, figname));

%% TESTING
%close all

diag_terms = [squeeze(P_hat(1, 1, :)), squeeze(P_hat(2, 2, :))];
%diag_terms = squeeze(P_hat(1,1, :));

RSS = vecnorm(x_hat - x_true, 2, 1);
RSS(RSS == 0) = 1e-10;
RSS_1sig = vecnorm(sqrt(diag_terms), 2, 2);

n = 1:numel(time);
err_norm = RSS(:) ./ RSS_1sig(:);
err_norm = sort(err_norm);

xvals = linspace(0, 1.1*err_norm(end), 1000);
ef = @(x)  normcdf(x) - normcdf(-1*x);
cdf_norm = ef(xvals);

f = figure('name', 'CDF');
hold on
plot(xvals, 100*cdf_norm, 'DisplayName', 'Normal CDF')
plot([1,2], 100 * ef([1,2]), '.r', 'HandleVisibility', 'off')
plot(unique(err_norm), linspace(0,100,numel(time)), 'DisplayName', 'Data')
hold off
xlabel('Normalized Error (RSS/1-sigma)')
ylabel('Percent Less than Error')
title('CDF of Data Error vs Normal')
legend('location', 'best')
hold off
grid on
figname = get_figname('CDF');
print(f, fullfile(figdir, figname), '-dpng', '-r300');

fprintf('Saved output to %s\n', figdir)