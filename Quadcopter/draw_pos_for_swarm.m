n_drones = 6;
sim_time = 12;
drone1 = [];
drone2 = [];
drone3 = [];
drone4 = [];
drone5 = [];

x = 1;
for i = 1:n_drones:size(simout_p,1)
    drone1(x,:) = simout_p(i,:);
    drone2(x,:) = simout_p(i+1,:);
    drone3(x,:) = simout_p(i+2,:);
    drone4(x,:) = simout_p(i+3,:);
    drone5(x,:) = simout_p(i+4,:);
    x = x+1;
end

% Normalize the time values to range from 0 to 2 to repeat colors twice
num_points = size(drone1, 1);
time_normalized = linspace(0, 1, num_points);
time_values = linspace(0, sim_time, num_points); % Adjusted to sim_time seconds

% Create a figure
figure;
hold on;
grid on; % Turn on gridlines

% Plot each drone's trajectory with varying hues over time
for i = 1:num_points-1
    color = hsv2rgb([mod(time_normalized(i), 1), 1, 1]); % Generate color based on hue, repeating twice
    plot(drone1(i:i+1,1), drone1(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
    plot(drone2(i:i+1,1), drone2(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
    plot(drone3(i:i+1,1), drone3(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
    plot(drone4(i:i+1,1), drone4(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
    plot(drone5(i:i+1,1), drone5(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
end

% Add labels for each drone at their first point
text(drone1(1,1), drone1(1,2), 'Drone 1', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');
text(drone2(1,1), drone2(1,2), 'Drone 2', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');
text(drone3(1,1), drone3(1,2), 'Drone 3', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');
text(drone4(1,1), drone4(1,2), 'Drone 4', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');
text(drone5(1,1), drone5(1,2), 'Drone 5', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');

% Plot an "X" marker at the last position of each drone
plot(drone1(end,1), drone1(end,2), 'rx', 'MarkerSize', 10);
plot(drone2(end,1), drone2(end,2), 'rx', 'MarkerSize', 10);
plot(drone3(end,1), drone3(end,2), 'rx', 'MarkerSize', 10);
plot(drone4(end,1), drone4(end,2), 'rx', 'MarkerSize', 10);
plot(drone5(end,1), drone5(end,2), 'rx', 'MarkerSize', 10);

% Add black dots at sim_time/5-second intervals along each drone's trajectory
step_size = sim_time/5; % Time step size in seconds
for i = 1:num_points
    if mod(time_values(i), step_size) == 0
        plot(drone1(i,1), drone1(i,2), 'k.', 'MarkerSize', 5);
        plot(drone2(i,1), drone2(i,2), 'k.', 'MarkerSize', 5);
        plot(drone3(i,1), drone3(i,2), 'k.', 'MarkerSize', 5);
        plot(drone4(i,1), drone4(i,2), 'k.', 'MarkerSize', 5);
        plot(drone5(i,1), drone5(i,2), 'k.', 'MarkerSize', 5);
    end
end


% Set the background color of the graph to white
ax = gca;
ax.Color = [0.89, 0.89, 0.89];

% Create a custom colormap by concatenating two copies of the HSV colormap
num_hues = 512; % Number of hues in each copy of the HSV colormap
custom_colormap = [hsv(num_hues)];

% Set the custom colormap
colormap(custom_colormap);

% Add a color bar
c = colorbar;

% Set the ticks to 20 evenly spaced points between 0 and sim_time seconds
num_ticks = 20;
tick_values = linspace(0, sim_time, num_ticks);
tick_positions = tick_values / sim_time; % Normalize tick values to [0, 1]
tick_labels = arrayfun(@(x) sprintf('%.1f', x), tick_values, 'UniformOutput', false);

c.Ticks = tick_positions; % Set the ticks to match the normalized positions
c.TickLabels = tick_labels; % Label the ticks with the corresponding time values
c.FontSize = 12;

% Label the color bar
ylabel(c, 'Time (s)', 'FontSize', 14);

hold off;
xlabel('X Position', 'FontSize', 14);
ylabel('Y Position', 'FontSize', 14);
