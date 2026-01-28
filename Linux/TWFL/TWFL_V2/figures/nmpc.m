% ==========================================
% 1. PARAMETERS & DATA GENERATION
% ==========================================
t_current = 5;          % Current time step (k)
N = 10;                 % Prediction Horizon (N)
dt = 1.0;

% Time vectors
time_past = 0:t_current;
time_pred = t_current:(t_current + N);

% Generate "Past" Data (History)
% Simulating a system settling down (damped oscillation)
x_past = zeros(1, length(time_past));
for i = 1:length(time_past)
    k = time_past(i);
    x_past(i) = 4.0 * (0.85^k) + 0.2 * sin(k);
end
current_state = x_past(end);

% Generate "Predicted" Data (The Optimization Window)
% Simulating the model's prediction of convergence over N steps
x_pred = zeros(1, length(time_pred));
for i = 1:length(time_pred)
    k = time_pred(i);
    x_pred(i) = current_state * (0.75^(k - t_current));
end

% Generate "Optimal Control Sequence"
% Simple proportional logic to simulate the computed inputs
% Note: Control is typically defined for N steps (k to k+N-1)
u_pred = -0.5 * x_pred(1:end-1);
time_u = time_pred(1:end-1);

% ==========================================
% 2. VISUALIZATION
% ==========================================
figure('Position', [100, 100, 1000, 600]);

% --- Plotting States (Left Y-Axis) ---
yyaxis left
color_state = [0, 0.4470, 0.7410]; % 'tab:blue' equivalent

% 1. Past State (Solid Line)
h1 = plot(time_past, x_past, 'Color', color_state, 'LineWidth', 2);
hold on;

% 2. Predicted State (Dashed Line)
h2 = plot(time_pred, x_pred, '--', 'Color', color_state, 'LineWidth', 2);

ylabel('System State x(k)', 'Color', color_state, 'FontWeight', 'bold');
ax = gca;
ax.YAxis(1).Color = color_state;

% --- Plotting Controls (Right Y-Axis) ---
yyaxis right
color_control = [0.5, 0.5, 0.5]; % 'tab:gray' equivalent

% 3. Optimal Control Sequence (Gray Dots)
h3 = scatter(time_u, u_pred, 50, color_control, 'filled', ...
        'MarkerEdgeColor', color_control, 'MarkerFaceAlpha', 0.6);
hold on;

% 4. Applied Control (Red Highlight)
% This represents u(k|k) - the only value actually used
h4 = scatter(t_current, u_pred(1), 150, 'r', 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

ylabel('Control Input u(k)', 'Color', color_control, 'FontWeight', 'bold');
ax.YAxis(2).Color = color_control;

% --- Formatting & Annotations ---
% Vertical Line for "Current Time"
xline(t_current, ':', 'LineWidth', 1.5, 'Color', 'k');
text(t_current + 0.2, max(u_pred)*0.9, 'Current Time (k)', ...
     'VerticalAlignment', 'middle');

% Title and Grid
title(sprintf('NMPC Receding Horizon Strategy (N=%d)', N), 'FontSize', 14);
xlabel('Time Step (k)');
grid on;
ax.GridAlpha = 0.3;

% Legend
legend([h1, h2, h3, h4], {'Past State (Measured)', ...
                           'Predicted State (Horizon)', ...
                           'Optimal Control Sequence', ...
                           'Applied Control (u_0)'}, ...
       'Location', 'northeast');

hold off;