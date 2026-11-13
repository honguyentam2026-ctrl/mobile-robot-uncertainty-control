function metrics = compute_path_metrics(t, poses, path, vCmd, wCmd)
%COMPUTE_PATH_METRICS Compute quantitative path-following performance metrics.
%   METRICS = COMPUTE_PATH_METRICS(T, POSES, PATH, VCMD, WCMD) computes
%   cross-track error statistics, final goal error, travel time, traveled
%   distance, and control effort from a simulated trajectory.
%
%   Inputs:
%     t     - N-by-1 time vector [s]
%     poses - N-by-3 matrix of [x, y, theta] poses [m, m, rad]
%     path  - M-by-2 matrix of reference waypoints [x, y] [m]
%     vCmd  - (optional) N-by-1 linear velocity commands [m/s]
%     wCmd  - (optional) N-by-1 angular velocity commands [rad/s]
%
%   Output:
%     metrics - struct with fields:
%       crossTrackError   - N-by-1 minimum distance from each pose to path segments [m]
%       rmseCrossTrack    - Root-mean-square cross-track error [m]
%       maxCrossTrack     - Maximum cross-track error [m]
%       finalGoalError    - Euclidean distance from final pose to last waypoint [m]
%       completionTime    - Total simulation duration t(end) - t(1) [s]
%       pathLengthActual  - Total traveled distance [m]
%       pathLengthRef     - Total reference polyline length [m]
%       vRms              - RMS linear velocity command [m/s] (NaN if omitted)
%       wRms              - RMS angular velocity command [rad/s] (NaN if omitted)
%       wTotalVariation   - Sum of |diff(wCmd)| measuring steering smoothness (NaN if omitted)

    n = size(poses, 1);
    cte = zeros(n, 1);

    for k = 1:n
        cte(k) = point_to_polyline_distance(poses(k, 1:2), path);
    end

    diffsActual = diff(poses(:, 1:2), 1, 1);
    diffsRef = diff(path, 1, 1);

    metrics.crossTrackError = cte;
    metrics.rmseCrossTrack = sqrt(mean(cte.^2));
    metrics.maxCrossTrack = max(cte);
    metrics.finalGoalError = norm(poses(end, 1:2) - path(end, :));
    metrics.completionTime = t(end) - t(1);
    metrics.pathLengthActual = sum(sqrt(sum(diffsActual.^2, 2)));
    metrics.pathLengthRef = sum(sqrt(sum(diffsRef.^2, 2)));

    if nargin >= 5 && ~isempty(vCmd) && ~isempty(wCmd)
        metrics.vRms = sqrt(mean(vCmd(:).^2));
        metrics.wRms = sqrt(mean(wCmd(:).^2));
        metrics.wTotalVariation = sum(abs(diff(wCmd(:))));
    else
        metrics.vRms = NaN;
        metrics.wRms = NaN;
        metrics.wTotalVariation = NaN;
    end
end

function dMin = point_to_polyline_distance(p, poly)
%POINT_TO_POLYLINE_DISTANCE Shortest Euclidean distance from point p to polyline.
    numSegs = size(poly, 1) - 1;
    dMin = inf;
    for s = 1:numSegs
        a = poly(s, :);
        b = poly(s + 1, :);
        ab = b - a;
        ab2 = dot(ab, ab);
        if ab2 < eps
            d = norm(p - a);
        else
            proj = dot(p - a, ab) / ab2;
            proj = min(max(proj, 0.0), 1.0);
            closest = a + proj * ab;
            d = norm(p - closest);
        end
        if d < dMin
            dMin = d;
        end
    end
end
