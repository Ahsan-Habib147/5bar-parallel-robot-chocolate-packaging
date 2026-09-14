function [P, A, B] = ForwKin_5bar(L1, L2, L3, L4, O1, O2, q)
% Forward kinematics of a 5-bar planar parallel robot.
% Given active joint angles q=[q1,q2], returns:
%   A   passive joint of left  chain [x,y]
%   B   passive joint of right chain [x,y]
%   P   end-effector position [Px, Py]

    q1 = q(1);  q2 = q(2);

    % Passive joint A (end of L1 from O1)
    A = [O1(1) + L1*cos(q1),  O1(2) + L1*sin(q1)];

    % Passive joint B (end of L3 from O2)
    B = [O2(1) + L3*cos(q2),  O2(2) + L3*sin(q2)];

    % Vector AB
    dx = B(1) - A(1);
    dy = B(2) - A(2);
    r  = sqrt(dx^2 + dy^2);

    % Cosine rule: angle at A in triangle A-P-B
    c     = (r^2 + L2^2 - L4^2) / (2 * r * L2);
    c     = max(min(c, 1), -1);      % clamp for numerical safety
    alpha = acos(c);

    % Angle of AB from x-axis
    psi = atan2(dy, dx);

    % End-effector (elbow-up: subtract alpha)
    Px = A(1) + L2 * cos(psi - alpha);
    Py = A(2) + L2 * sin(psi - alpha);
    P  = [Px, Py];
end
