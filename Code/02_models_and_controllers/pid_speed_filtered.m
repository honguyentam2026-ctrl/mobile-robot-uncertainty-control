function [pidState, uOut] = pid_speed_filtered(pidState, refVel, measVel, cfg, gainScale)
%PID_SPEED_FILTERED Discrete 2-channel PIDF (PID with filtered derivative).
%   [PIDSTATE, UOUT] = PID_SPEED_FILTERED(PIDSTATE, REFVEL, MEASVEL, CFG)
%   computes inner-loop velocity commands [uV; uW] using fixed PID gains
%   with conditional-integration anti-windup and a first-order low-pass
%   filter on derivative-on-measurement to avoid amplifying encoder noise.
%
%   Optional GAINSCALE = [sKp, sKi, sKd] scales (Kp, Ki, Kd) online (used
%   by Fuzzy-PID in C4 and C5; defaults to [1, 1, 1] for fixed PIDF in C3).

    if nargin < 5 || isempty(gainScale)
        gainScale = [1.0, 1.0, 1.0];
    end

    dt = cfg.dt;
    if isempty(pidState)
        pidState.Kp0 = cfg.pidf.Kp;
        pidState.Ti0 = cfg.pidf.Ti;
        pidState.Td0 = cfg.pidf.Td;
        pidState.Tf  = cfg.pidf.Tf;
        pidState.intState  = [0.0; 0.0];
        pidState.dFilt     = [0.0; 0.0];
        pidState.prevMeas  = measVel(:);
    end

    Kp = pidState.Kp0 * gainScale(1);
    Ki = (pidState.Kp0 / pidState.Ti0) * gainScale(2);
    Kd = (pidState.Kp0 * pidState.Td0) * gainScale(3);
    Tf = pidState.Tf;

    limits = [cfg.actuator.vLim; cfg.actuator.wLim];
    uOut = zeros(2, 1);

    alphaD = Tf / (Tf + dt);
    betaD  = Kd / (Tf + dt);

    for ch = 1:2
        err = refVel(ch) - measVel(ch);

        % Filtered derivative on measurement (no setpoint kick, attenuates noise)
        dMeas = measVel(ch) - pidState.prevMeas(ch);
        pidState.dFilt(ch) = alphaD * pidState.dFilt(ch) - betaD * dMeas;
        pidState.prevMeas(ch) = measVel(ch);

        uUnsat = Kp * err + pidState.intState(ch) + pidState.dFilt(ch);
        uSat = min(max(uUnsat, limits(ch, 1)), limits(ch, 2));

        % Conditional-integration anti-windup
        isNotSat = abs(uUnsat - uSat) < 1e-12;
        drivesInside = (uUnsat > limits(ch, 2) && err < 0) || ...
                       (uUnsat < limits(ch, 1) && err > 0);
        if isNotSat || drivesInside
            pidState.intState(ch) = pidState.intState(ch) + Ki * err * dt;
        end

        uOut(ch) = uSat;
    end
end
