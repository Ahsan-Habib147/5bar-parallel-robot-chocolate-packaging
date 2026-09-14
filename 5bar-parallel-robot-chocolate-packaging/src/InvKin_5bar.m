function q = InvKin_5bar(L1, L2, L3, L4, O1, O2, P)
% Inverse kinematics of a 5-bar planar parallel robot.
% Given desired end-effector position P=[Px,Py], returns:
%   q = [q1, q2]   active joint angles (radians)

    Px = P(1);  Py = P(2);

    % ── Left chain: O1 → A → P  (links L1, L2) ──
    x1 = Px - O1(1);
    y1 = Py - O1(2);
    R1 = sqrt(x1^2 + y1^2);
    c1 = (R1^2 + L1^2 - L2^2) / (2 * L1 * R1);
    c1 = max(min(c1, 1), -1);
    q1 = atan2(y1, x1) - acos(c1);   % elbow-up

    % ── Right chain: O2 → B → P  (links L3, L4) ──
    x2 = Px - O2(1);
    y2 = Py - O2(2);
    R2 = sqrt(x2^2 + y2^2);
    c2 = (R2^2 + L3^2 - L4^2) / (2 * L3 * R2);
    c2 = max(min(c2, 1), -1);
    q2 = atan2(y2, x2) + acos(c2);   % elbow-down

    q = [q1, q2];
end
