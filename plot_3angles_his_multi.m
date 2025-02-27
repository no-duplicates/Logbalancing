% Set up color order for consistency across all subplots
colors = lines(7);  % Use MATLAB's "lines" colormap to generate 7 distinct colors

% Set figure size for flatter plots (8x6 inches)
figure('Units', 'inches', 'Position', [0, 0, 8, 6], 'PaperPositionMode', 'auto');
results=results_noised_3;
% Loop for i = 1:7
for i = 1:7
    % Extract data from results{i}
    t = results{i}.t;
    betas = results{i}.betas;
    COMs = results{i}.COMs;
    thetas = results{i}.thetas;

    n = length(betas);  % Assuming COMs, thetas, and betas are of the same length
    alphas = zeros(1, n);

    % Loop through each COM, theta, and beta triplet to compute alphas
    for j = 1:n
        % Extract the current COM, theta, and beta
        COM_x = COMs(j);
        theta = thetas(j);
        beta = betas(j);

        % Call the compute_alpha function to calculate alpha for current values
        alphas(j) = compute_alpha(Model_consts, COM_x, theta, beta);
    end
    
    % Plot 1: alphas-thetas+pi/2 (ankle)
    subplot(3, 1, 1);  % First plot (1st row in 3x1 grid)
    ankle_angles = alphas - thetas + pi/2;  % Calculate the ankle angles
    plot(t, ankle_angles, 'Color', colors(i, :), 'LineWidth', 2);  % Use consistent color
    hold on;  % Allow multiple lines to be plotted
    title('Ankle Angle over Time', 'FontSize', 32, 'FontWeight', 'bold');  % Bold title with FontSize 32
    ylabel('Ankle Angle (rad)', 'FontSize', 16);  % Label with FontSize 16
    grid on;
    
    % Plot 2: betas-alphas+pi (hip)
    subplot(3, 1, 2);  % Second plot (2nd row in 3x1 grid)
    hip_angles = betas - alphas + pi;  % Calculate the hip angles
    plot(t, hip_angles, 'Color', colors(i, :), 'LineWidth', 2);  % Use consistent color
    hold on;
    title('Hip Angle over Time', 'FontSize', 32, 'FontWeight', 'bold');  % Bold title with FontSize 32
    ylabel('Hip Angle (rad)', 'FontSize', 16);  % Label with FontSize 16
    grid on;
    
    % Plot 3: COMs
    subplot(3, 1, 3);  % Third plot (3rd row in 3x1 grid)
    plot(t, COMs, 'Color', colors(i, :), 'LineWidth', 2);  % Use consistent color
    hold on;
    title('Center of Mass over Time', 'FontSize', 32, 'FontWeight', 'bold');  % Bold title with FontSize 32
    xlabel('Time (s)', 'FontSize', 16);  % Label with FontSize 16
    ylabel('COM X (m)', 'FontSize', 16);  % Label with FontSize 16
    grid on;
end

% Finalize figure after all lines are plotted
hold off;  % Stop holding plots
tight_layout();  % Ensures proper layout without overlaps
