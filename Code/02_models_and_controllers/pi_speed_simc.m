function [piState, uOut] = pi_speed_simc(piState, refVel, measVel, cfg)
%PI_SPEED_SIMC Discrete PI speed controller tuned via Skogestad S-IMC rules.
%   [PISTATE, UOUT] = PI_SPEED_SIMC(PISTATE, REFVEL, MEASVEL, CFG) updates a
%   two-channel (v, omega) PI velocity loop with conditional-integration
%   anti-windup and actuator saturation.
%
%   Skogestad S-IMC formula (Slide Chapter 3_part 2, p.73):
%     Kp = (1/K0) * T0 / (Tc + tau0)
%     Ti = min(T0, 4*(Tc + tau0))

    dt = cfg.dt;
    if isempty(piState)
        K0 = cfg.actuator.K0;
        T0 = cfg.actuator.T0;
        tau0 = cfg.actuator.tau0;
        Tc = cfg.actuator.Tc;

        piState.Kp = (1.0 / K0) * (T0 / (Tc + tau0));
        piState.Ti = min(T0, 4.0 * (Tc + tau0));
        piState.intState = [0.0; 0.0];
    end

    limits = [cfg.actuator.vLim; cfg.actuator.wLim];
    uOut = zeros(2, 1);

    for ch = 1:2
        err = refVel(ch) - measVel(ch);
        uUnsat = piState.Kp * err + piState.intState(ch);
        uSat = min(max(uUnsat, limits(ch, 1)), limits(ch, 2));

        % Conditional integration anti-windup
        isNotSat = abs(uUnsat - uSat) < 1e-12;
        drivesInside = (uUnsat > limits(ch, 2) && err < 0) || ...
                       (uUnsat < limits(ch, 1) && err > 0);
        if isNotSat || drivesInside
            piState.intState(ch) = piState.intState(ch) + ...
                (piState.Kp / piState.Ti) * err * dt;
        end

        uOut(ch) = uSat;
    end
end
