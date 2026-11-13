function [x, P, stepNis, numAccepted, numRejected] = ekf_joseph_update( ...
    x, P, obsList, landmarks, What, useGating, gateChi2)
%EKF_JOSEPH_UPDATE Sequential range-bearing update with Joseph-form covariance.
%   [X, P, STEPNIS, NUMACCEPTED, NUMREJECTED] = EKF_JOSEPH_UPDATE(...)
%   updates an n-state EKF (n >= 3, first 3 states = [x; y; theta]) across
%   all landmark observations in obsList = [id, range, bearing], using the
%   Joseph-form covariance update (Chapter 4_part 1) and optional chi2
%   Mahalanobis gating (Chapter 4_part 2, p.35).

    n = length(x);
    I_n = eye(n);
    nisList = [];
    numAccepted = 0;
    numRejected = 0;

    for k = 1:size(obsList, 1)
        lmId = obsList(k, 1);
        zMeas = obsList(k, 2:3)';
        lmXY = landmarks(lmId, :)';

        [zHat, Hx3] = landmark_measurement_model(x(1:3), lmXY);
        H = [Hx3, zeros(2, n - 3)];

        nu = [zMeas(1) - zHat(1);
              wrap_to_pi(zMeas(2) - zHat(2))];

        S = H * P * H' + What;
        nis = nu' * (S \ nu);
        nisList(end+1, 1) = nis; %#ok<AGROW>

        if useGating && (nis > gateChi2)
            numRejected = numRejected + 1;
            continue;
        end

        K = (P * H') / S;
        x = x + K * nu;
        x(3) = wrap_to_pi(x(3));

        I_KH = I_n - K * H;
        P = I_KH * P * I_KH' + K * What * K';
        P = 0.5 * (P + P');
        numAccepted = numAccepted + 1;
    end

    if isempty(nisList)
        stepNis = NaN;
    else
        stepNis = mean(nisList);
    end
end
