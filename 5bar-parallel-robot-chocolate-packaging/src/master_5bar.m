clear; clc; close all;

% ── Robot Parameters ──────────────────────────────
L1 = 20; L2 = 20;   % Left chain link lengths (cm)
L3 = 20; L4 = 20;   % Right chain link lengths (cm)
O1 = [-8  10];       % Left  base joint position (cm)
O2 = [-2  10];       % Right base joint position (cm)

% ── Pick-up point (conveyor centre) ──────────────
pick = [4 10];

% ── Tray grid: 10×10 dimples, 20 cm × 20 cm ─────
spacing = 20/9;   % ≈ 2.22 cm between dimple centres
[gx, gy] = meshgrid(8 : spacing : 28, ...
                    0 : spacing : 20);
grid_pos = [gx(:), gy(:)];  % 100×2 matrix

% ── Storage matrices ─────────────────────────────
placed_points = [];
q_traj        = [];   % joint angle history
ee_traj       = [];   % end-effector trajectory

figure('Position', [100 50 1200 700]);

for i = 1 : length(grid_pos)
    target = grid_pos(i, :);
    steps  = 8;

    % ── Step 1: pick → place ───────────────────
    for s = 0 : steps
        a  = s / steps;
        Px = (1-a)*pick(1) + a*target(1);
        Py = (1-a)*pick(2) + a*target(2);
        q  = InvKin_5bar(L1,L2,L3,L4, O1,O2, [Px Py]);
        [P, A, B] = ForwKin_5bar(L1,L2,L3,L4, O1,O2, q);
        q_traj  = [q_traj;  q];
        ee_traj = [ee_traj; P];
        draw_5bar(A,B,P, O1,O2, gx,gy, placed_points);
        drawnow limitrate;
        %pause(0.02);
    end

    placed_points = [placed_points; target];

    % ── Step 2: place → pick (return) ──────────
    for s = 0 : steps
        a  = s / steps;
        Px = (1-a)*target(1) + a*pick(1);
        Py = (1-a)*target(2) + a*pick(2);
        q  = InvKin_5bar(L1,L2,L3,L4, O1,O2, [Px Py]);
        [P, A, B] = ForwKin_5bar(L1,L2,L3,L4, O1,O2, q);
        q_traj  = [q_traj;  q];
        ee_traj = [ee_traj; P];
        draw_5bar(A,B,P, O1,O2, gx,gy, placed_points);
        drawnow limitrate;
       % pause(0.02);
    end
end

% ── Summary plots ──────────────────────────────
figure;
subplot(2,1,1);
plot(q_traj(:,1)*180/pi, 'b', 'LineWidth', 1.5); hold on;
plot(q_traj(:,2)*180/pi, 'r', 'LineWidth', 1.5);
legend('q1 (left chain)', 'q2 (right chain)');
xlabel('Step'); ylabel('Angle (degrees)');
title('Active Joint Angles over Pick-and-Place Cycle');

subplot(2,1,2);
plot(ee_traj(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(ee_traj(:,2), 'r', 'LineWidth', 1.5);
legend('X', 'Y');
xlabel('Step'); ylabel('Position (cm)');
title('End-effector Trajectory');
