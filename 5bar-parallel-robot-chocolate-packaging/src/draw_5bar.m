function draw_5bar(A, B, P, O1, O2, gx, gy, placed_points)
% Visualise the 5-bar robot configuration.

    cla; hold on; grid on;

    % Background regions
    rectangle('Position', [-10 0 10 20], ...
              'FaceColor', [1 0.6 0.6], 'EdgeColor', 'none'); % robot mount
    fill([0 8 8 0], [0 0 20 20], [0.85 0.92 1], 'EdgeColor', 'none'); % conveyor
    rectangle('Position', [8 0 20 20], ...
              'EdgeColor', 'k', 'LineWidth', 2);              % tray boundary

    % Tray dimple grid
    plot(gx, gy, 'k.', 'MarkerSize', 10);

    % Placed chocolates
    if ~isempty(placed_points)
        plot(placed_points(:,1), placed_points(:,2), 's', ...
             'MarkerSize', 6, 'MarkerFaceColor', [0.6 0.3 0.1], ...
             'MarkerEdgeColor', 'k');
    end

    % Pick-up point marker
    plot(4, 10, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    % Base joints
    plot(O1(1), O1(2), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot(O2(1), O2(2), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

    % Left chain: O1 → A → P  (blue)
    plot([O1(1) A(1) P(1)], [O1(2) A(2) P(2)], ...
         '-o', 'Color', 'b', 'LineWidth', 2);

    % Right chain: O2 → B → P  (red)
    plot([O2(1) B(1) P(1)], [O2(2) B(2) P(2)], ...
         '-o', 'Color', 'r', 'LineWidth', 2);

    % End-effector highlight
    plot(P(1), P(2), 'o', 'MarkerSize', 12, ...
         'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y', 'LineWidth', 2);

    axis equal;
    xlim([-20 35]);  ylim([-15 35]);
    xlabel('X (cm)');  ylabel('Y (cm)');
    title('5-Bar Robot  Chocolate Pick and Place');

end
