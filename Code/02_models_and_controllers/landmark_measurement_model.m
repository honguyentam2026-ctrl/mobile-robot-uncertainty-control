function [zHat, Hx] = landmark_measurement_model(pose, landmarkXY)
%LANDMARK_MEASUREMENT_MODEL Range-bearing observation and 2x3 Jacobian.
%   [ZHAT, HX] = LANDMARK_MEASUREMENT_MODEL(POSE, LANDMARKXY) computes the
%   expected measurement [range; bearing] and the Jacobian Hx w.r.t.
%   pose = [x; y; theta] per Slide Chapter 4_part 2 (p.27-30).

    dx = landmarkXY(1) - pose(1);
    dy = landmarkXY(2) - pose(2);
    q = dx^2 + dy^2;
    r = sqrt(max(q, 1e-12));

    bearing = wrap_to_pi(atan2(dy, dx) - pose(3));
    zHat = [r; bearing];

    if nargout > 1
        qSafe = max(q, 1e-12);
        Hx = [-dx / r,     -dy / r,      0.0;
               dy / qSafe, -dx / qSafe, -1.0];
    end
end
