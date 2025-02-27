% Set figure size for better visibility in publications
figure('Units', 'inches', 'Position', [0, 0, 6, 4], 'PaperPositionMode', 'auto');

% Define colors for the different COM plots
colors = lines(7);  % Using MATLAB's 'lines' colormap for distinct colors

% Hold on to add multiple COM trajectories to the same plot
hold on;
%t=results_noised{i}.t;
%results=results;

% Loop through the results to plot each COM trajectory with solid lines
for i = 1:size(results)
    % Plot the COM data for the i-th result with a solid line
    plot(results{i}.t, results{i}.COMs, 'Color', colors(i, :), 'LineStyle', '-', 'LineWidth', 2);  
end

% Highlight the upper bound of the controllable region for state 3
%yline(0.04, '--r', 'LineWidth', 3);  % Red dashed line to indicate the upper bound

% Shade the controllable region for state 3
fill([min(t) max(t) max(t) min(t)], [-inf -inf 0.04 0.04], 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');

% Tighten the axis limits
xlim([min(t), max(t)]);
ylim([min(cellfun(@(r) min(r.COMs), results)) - 0.01, max(cellfun(@(r) max(r.COMs), results)) + 0.01]);  % Tighten y-axis based on data range

% Add title and axis labels with LaTeX interpreter and increased font size
title('Center of Mass over Time', 'FontSize', 32, 'Interpreter', 'latex');
xlabel('Time (s)', 'FontSize', 16, 'Interpreter', 'latex');
ylabel('Center of Mass (m)', 'FontSize', 16, 'Interpreter', 'latex');

% Add grid for better readability
grid on;

% Set font size and style for axis ticks
set(gca, 'FontSize', 12, 'FontName', 'Times New Roman');

% Save the figure in a publication-quality format
print(gcf, 'COM_Upper_Bound_State_3_Multi_COM_Tight_Axis_Solid_Lines', '-dpdf', '-r300'); % Save as PDF with 300 dpi resolution
