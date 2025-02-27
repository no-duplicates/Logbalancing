time_interval=1:1000;
size_results=size(results);
size_results=size_results(1);
t = results{size_results}.t(time_interval);
betas = results{size_results}.betas(time_interval);
COMs = results{size_results}.COMs(time_interval);
thetas = results{size_results}.thetas(time_interval);

n = length(betas);  % Assuming COMs, thetas, and betas are of the same length
alphas = zeros(1, n);

% Loop through each COM, theta, and beta triplet
for i = 1:n
    % Extract the current COM, theta, and beta
    COM_x = COMs(i);
    theta = thetas(i);
    beta = betas(i);
    
    % Call the compute_alpha function to calculate alpha for current values
    alphas(i) = compute_alpha(Model_consts, COM_x, theta, beta);
end

% Set figure size for flatter plots (8x6 inches)
figure('Units', 'inches', 'Position', [0, 0, 8, 6], 'PaperPositionMode', 'auto');

% Plot 1: alphas-thetas+pi/2 (ankle)
subplot(3, 1, 1);  % First plot (1st row in 3x1 grid)
ankle_angles = alphas - thetas + pi/2;  % Calculate the ankle angles
plot(t, ankle_angles, 'b-', 'LineWidth', 5);  % Blue line with LineWidth 5
title('Ankle Angle over Time', 'FontSize', 16, 'FontWeight', 'bold');  % Bold title with FontSize 32
ylabel('Ankle Angle (rad)', 'FontSize', 14);  % Label with FontSize 16
grid on;

% Plot 2: betas-alphas+pi (hip)
subplot(3, 1, 2);  % Second plot (2nd row in 3x1 grid)
hip_angles = betas - alphas + pi;  % Calculate the hip angles
plot(t, hip_angles, 'b-', 'LineWidth', 5);  % Blue line with LineWidth 5
title('Hip Angle over Time', 'FontSize', 16, 'FontWeight', 'bold');  % Bold title with FontSize 32
ylabel('Hip Angle (rad)', 'FontSize', 14);  % Label with FontSize 16
grid on;

% Plot 3: COMs
subplot(3, 1, 3);  % Third plot (3rd row in 3x1 grid)
plot(t, COMs, 'b-', 'LineWidth', 5);  % Blue line with LineWidth 5
title('Center of Mass over Time', 'FontSize', 16, 'FontWeight', 'bold');  % Bold title with FontSize 32
xlabel('Time (s)', 'FontSize', 14);  % Label with FontSize 16
ylabel('COM X (m)', 'FontSize', 14);  % Label with FontSize 16
grid on;

% Adjust the layout
tight_layout();  % Ensures proper layout without overlaps
