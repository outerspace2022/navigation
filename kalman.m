% Example KF implementation
clear; close all; clc;
rng(1337)

%% Define the initial states
% Here, we start at position zero with velocity +1m/s and absolute
% certainty about our initial state
x0 = [0 0.31]';
P0 = diag([1e-4, 1e-6]);

dt = 0.050;  % Update once per 0.050 second (camera freq)
t_end = 41 * 0.05;  % Simulate for 100 seconds

steps = t_end/dt;
time = linspace(0,t_end,steps);

%% Define A, H, Q, and R matrices
A = [1 dt; 
     0 1];
H = [1 0];

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
sigma_a = 1;  % [m/s^2] Process noise standard deviation
Q = diag(G.^2) * sigma_a^2;

% R quantifies the uncertainty in our sensor measurements. Recall that
% measurement noise is modeled as:
%       z_k = H_k * x_k + r_k         (Equation 12 from the slides)
% Where r_k is drawn from a zero-mean Gaussian with standard deviation
% sigma_r. The R matrix captures this uncertainty for all sensor inputs.
% Since we have one sensor here, R is simply a scalar that equals sigma_r^2.
sigma_r = 0.00692412970236608;  % [m] Measurement noise standard deviation <-- CHANGE THIS TO MATCH CAMERA STD
R = sigma_r^2;

%% Our Code: Read in camera csv data
ourData = readmatrix("x_pos_data.csv");
disp(ourData)

%% Run the filter
%[x_hat, P_hat, time, x_true, z_k] = kalman(dt, t_end, x0, P0, A, H, Q, R);
% x_true = zeros([2,2]);
x_true = [ 0 0
           0.31 0.31];
x_hat = [ 0 0
           0.31 0.31];
P_hat = zeros([2,2]);
x_hat_k = zeros([2,2]);
P_hat_k = zeros([2,2]);

LR = chol(R, 'lower');
LQ = chol(Q, 'lower');

%the x variables represent system state (like position, velocity, etc)
%the z variables represent sensor measurements
%z_k is the measurement from the sensor. We are faking it rn
%z_hat is predicted sensor measurement
%x_hat is predicted state based off previous state
%p_hat is covariance matrix which gets updated every iteration
for k = 2:numel(time)
    %Propagate the truth state and add noise
     disp(x_true(:, k-1))

    x_true(:,k) = A*x_true(:,k-1); % + LQ*randn(size(x0));
    

    %Propagate the filter states
    x_hat_k(:,k) = A*x_hat(:,k-1);
    P_hat_k(:,:,k) = A*P_hat(:,:,k-1)*A' + Q;
    
    %Generate a noisy measurement as a function of our true state
%     z_k(k) = (H*x_true(:,k) - H*x_true(:, k-1)) + LR*randn(size(H,1),1); % <-- CHANGE THIS TO WHATEVER THE CAMERA IS OUTPUTTING
    z_k(k) = ourData(k-1); % <-- CHANGE THIS TO WHATEVER THE CAMERA IS OUTPUTTING

    z_hat_k = H*x_hat_k(:,k) - H*x_hat_k(:, k-1); %Predicted measurement is our expected position
    y = z_k(k) - z_hat_k;
    
    %Measurement update
    S = H*P_hat_k(:,:,k)*H' + R;
    K = P_hat_k(:,:,k)*H'/S;
    x_hat(:,k) = x_hat_k(:,k) + K*y;
    P_hat(:,:,k) = (eye(size(P0)) - K*H) * P_hat_k(:,:,k);
end
    
%% Plot results
set(groot, 'DefaultLineLinewidth', 2);
set(groot, 'DefaultLineMarkersize', 25);
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultAxesBox', 'on');
set(0, 'DefaultFigureWindowStyle', 'normal')
%set(0, 'DefaultFigureWindowStyle', 'docked')
set(groot, 'DefaultFigurePosition', [100 100 1200 500]);
set(0, 'DefaultAxesColor', 'white')


fig_prefix = 'baseline';  % Change this to prevent overwriting existing figures
fig_format = 'png';  % Format with which to save the figures
figdir = fullfile('..','imgs');
if (~exist(figdir, 'dir')), mkdir(figdir); end

fmt = @(x) strip(strip(sprintf('%f',x), '0'), 'right', '.');
sigstr = sprintf('$$\\sigma_a=%s$$, $$\\sigma_r=%s$$', fmt(sigma_a), fmt(sigma_r));
sigstr_save = replace(sprintf('sigmaa=%s_sigmar=%s', fmt(sigma_a), fmt(sigma_r)), '.','p');
mkdir(fullfile(figdir, sigstr_save))
get_figname = @(x) fullfile(sigstr_save, sprintf('%s.%s', x, fig_format));


% Position
f = figure('name', 'Estimated Position'); hold on;
cmap = colormap('lines');
plot(time, x_true(1, :), 'k', 'linewidth', 3);
plot(time, z_k, '--', 'color', cmap(2, :), 'linewidth', 1.5);
plot(time, x_hat(1, :), 'color', cmap(1, :), 'linewidth', 3);
legend('Truth', 'Measurement', 'Kalman')
xlabel('Time (s)');
ylabel('Position (m)');
title(sprintf('Estimated Position (%s)', sigstr), 'interpreter', 'latex');
box on; grid on;
figname = get_figname('EstimatedPosition');
print(f, fullfile(figdir, figname), '-dpng', '-r300');

% Velocity
f = figure('name', 'Estimated Velocity'); hold on;
plot(time, x_true(2, :), 'k', 'linewidth', 2);
plot(time, x_hat(2, :), 'color', cmap(1, :), 'linewidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title(sprintf('Estimated Velocity (%s)', sigstr), 'interpreter', 'latex');
legend({'Ground Truth', 'Estimated Velocity'}, 'Location', 'northwest');
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

diag_terms = [squeeze(P_hat(1,1, :)), squeeze(P_hat(2,2, :))];
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