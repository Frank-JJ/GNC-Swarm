%load("rotation_test_pointtox_POS.mat")
filename = "fix_1ntx_dot.mat";
arrows = false;
flip_y = false;
NED_xyflip = true;
save_ws = false;

n_drones = 6;
sim_time = 15;

drone1 = [];
drone2 = [];
drone3 = [];
drone4 = [];
drone5 = []; 

x = 1;
for i = 7:n_drones:size(simout_p,1)
    drone1(x,:) = simout_p(i,:);
    drone2(x,:) = simout_p(i+1,:);
    drone3(x,:) = simout_p(i+2,:);
    drone4(x,:) = simout_p(i+3,:);
    drone5(x,:) = simout_p(i+4,:);
    x = x+1;
end

if flip_y
    %Make the NED frame to Earth-fixed frame by inverting the Y-axis:
    drones = {drone1, drone2, drone3, drone4, drone5};
    
    for k = 1:length(drones)
        drones{k}(:, 2) = -drones{k}(:, 2);
    end
    
    drone1 = drones{1};
    drone2 = drones{2};
    drone3 = drones{3};
    drone4 = drones{4};
    drone5 = drones{5};
end

if NED_xyflip
    drone1 = drone1(:, [2, 1]);
    drone2 = drone2(:, [2, 1]);
    drone3 = drone3(:, [2, 1]);
    drone4 = drone4(:, [2, 1]);
    drone5 = drone5(:, [2, 1]);
end

% Normalize the time values to range from 0 to 1
num_points = size(drone1, 1);
time_normalized = linspace(0, 1, num_points);
time_values = linspace(0, sim_time, num_points);

% Create a figure
figure;
hold on;
grid on;

% Plot each drone's trajectory with varying hues over time
for i = 1:num_points-1
    color = hsv2rgb([mod(time_normalized(i), 1), 1, 1]);
    plot(drone1(i:i+1,1), drone1(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
    plot(drone2(i:i+1,1), drone2(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
    plot(drone3(i:i+1,1), drone3(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
    plot(drone4(i:i+1,1), drone4(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
    plot(drone5(i:i+1,1), drone5(i:i+1,2), 'Color', color, 'LineWidth', 1.5);
end

% Add labels for each drone at their first point
text(drone1(1,1), drone1(1,2), 'Drone 1', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');
text(drone2(1,1), drone2(1,2), 'Drone 2', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');
text(drone3(1,1)+4, drone3(1,2)+1, 'Drone 3 (-TX)', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');
text(drone4(1,1), drone4(1,2), 'Drone 4', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');
text(drone5(1,1), drone5(1,2), 'Drone 5', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'k');

% Plot markers at the first and last positions of each drone
start_marker = 'bo';
end_marker = 'rx';

plot(drone1(1, 1), drone1(1, 2), start_marker, 'MarkerSize', 10);
plot(drone2(1, 1), drone2(1, 2), start_marker, 'MarkerSize', 10);
plot(drone3(1, 1), drone3(1, 2), start_marker, 'MarkerSize', 10);
plot(drone4(1, 1), drone4(1, 2), start_marker, 'MarkerSize', 10);
plot(drone5(1, 1), drone5(1, 2), start_marker, 'MarkerSize', 10);

plot(drone1(end, 1), drone1(end, 2), end_marker, 'MarkerSize', 20);
plot(drone2(end, 1), drone2(end, 2), end_marker, 'MarkerSize', 20);
plot(drone3(end, 1), drone3(end, 2), end_marker, 'MarkerSize', 20);
plot(drone4(end, 1), drone4(end, 2), end_marker, 'MarkerSize', 20);
plot(drone5(end, 1), drone5(end, 2), end_marker, 'MarkerSize', 20);

% Add black dots at sim_time/3-second intervals along each drone's trajectory
%step_size = sim_time/3;
%for i = 1:num_points
%    if mod(time_values(i), step_size) < 0.05
%        plot(drone1(i,1), drone1(i,2), 'k.', 'MarkerSize', 10);
%        plot(drone2(i,1), drone2(i,2), 'k.', 'MarkerSize', 10);
%        plot(drone3(i,1), drone3(i,2), 'k.', 'MarkerSize', 10);
%        plot(drone4(i,1), drone4(i,2), 'k.', 'MarkerSize', 10);
%        plot(drone5(i,1), drone5(i,2), 'k.', 'MarkerSize', 10);
%    else
%        mod(time_values(i), step_size)
%    end
%end

if arrows
    % Plot the orientation arrows at the first and last positions of each drone
    arrow_length = 1.25; % Length of the arrow
    
    % Plot arrows for the first position
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_1(1)));
    quiver(drone1(1,1), drone1(1,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_2(1)));
    quiver(drone2(1,1), drone2(1,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_3(1)));
    quiver(drone3(1,1), drone3(1,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_4(1)));
    quiver(drone4(1,1), drone4(1,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_5(1)));
    quiver(drone5(1,1), drone5(1,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
    
    % Plot arrows for the last position
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_1(end)));
    quiver(drone1(end,1), drone1(end,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_2(end)));
    quiver(drone2(end,1), drone2(end,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_3(end)));
    quiver(drone3(end,1), drone3(end,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_4(end)));
    quiver(drone4(end,1), drone4(end,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
    [dx, dy] = calculate_arrow(arrow_length,deg2rad(90-1*simout_psi_5(end)));
    quiver(drone5(end,1), drone5(end,2), dx, dy, 0, 'k', 'MaxHeadSize', 3);
end

% Set the background color of the graph to light gray
ax = gca;
ax.Color = [0.89, 0.89, 0.89];

% Create a custom colormap by concatenating two copies of the HSV colormap
num_hues = 512;
custom_colormap = [hsv(num_hues)];

% Set the custom colormap
colormap(custom_colormap);

% Add a color bar
c = colorbar;

% Set the ticks to 20 evenly spaced points between 0 and sim_time seconds
num_ticks = 20;
tick_values = linspace(0, sim_time, num_ticks);
tick_positions = tick_values / sim_time;
tick_labels = arrayfun(@(x) sprintf('%.1f', x), tick_values, 'UniformOutput', false);

c.Ticks = tick_positions;
c.TickLabels = tick_labels;
c.FontSize = 12;

% Label the color bar
ylabel(c, 'Time (s)', 'FontSize', 14);

% Create dummy plot objects for the legend
h_start = plot(NaN, NaN, start_marker, 'MarkerSize', 10);
h_end = plot(NaN, NaN, end_marker, 'MarkerSize', 20);

if arrows
    % Create a dummy quiver for the legend
    dummy_x = 0; % x position for the dummy quiver
    dummy_y = 0; % y position for the dummy quiver
    dummy_u = 0.0001; % x component of the arrow
    dummy_v = 0; % y component of the arrow
    h_arrow = quiver(dummy_x, dummy_y, dummy_u, dummy_v, 0, 'k', 'MaxHeadSize', 2);
    legend([h_start, h_end, h_arrow], {'Starting Position', 'Ending Position', 'Drone yaw (psi)'}, 'Location', 'Best', 'FontSize', 12);
else
% Add the legend
    legend([h_start, h_end], {'Starting Position', 'Ending Position'}, 'Location', 'Best', 'FontSize', 12);
end

hold off;

if NED_xyflip
    xlabel('East', 'FontSize', 14);
    ylabel('North', 'FontSize', 14);
else
    xlabel('X Position', 'FontSize', 14);
    ylabel('Y Position', 'FontSize', 14);
end

axis([-10 10 -10 10]);
xticks(-10:2:10); 
yticks(-10:2:10); 
axis equal

if save_ws
    save(filename)
end

% Local function to calculate arrow components
function [dx, dy] = calculate_arrow(arrow_length, angle)
    dx = arrow_length * cos(angle);
    dy = arrow_length * sin(angle);
end