function [plantState, poseTrue, encoderDelta, etaTrue] = robot_plant_step( ...
    plantState, uCmd, tCurr, cfg)
%ROBOT_PLANT_STEP Advance true robot plant with actuator lag, load change,
%geometric mismatch, and floor slip.
%   [PLANTSTATE, POSETRUE, ENCODERDELTA, ETATRUE] = ROBOT_PLANT_STEP(
%   PLANTSTATE, UCMD, TCURR, CFG) simulates one discrete time step dt of
%   the physical differential-drive robot.
%
%   Physical separation modeled:
%     1) Inner velocity actuator (FOPDT: gain K, time constant T, delay tau)
%        drives wheel shaft speeds [vShaft; wShaft], which encoders measure.
%     2) Tire-floor contact scales actual body motion by true parameters
%        eta_v = etaVBase*(1 - slipV) and eta_w = etaWBase*(1 - slipW), plus
%        optional yaw disturbance biasW. Encoders cannot observe floor slip
%        or wheel-radius/track-width mismatch.

    dt = cfg.dt;
    scen = cfg.scenario;

    %% 1. Select active FOPDT actuator parameters (nominal vs payload step)
    if ~cfg.actuator.useNominalDynamics
        K = 1.0; T = 0.0; tau = 0.0;
    elseif scen.loadStep.enabled && (tCurr >= scen.loadStep.time)
        K = scen.loadStep.K;
        T = scen.loadStep.T;
        tau = scen.loadStep.tau;
    else
        K = cfg.actuator.K0;
        T = cfg.actuator.T0;
        tau = cfg.actuator.tau0;
    end

    % Saturate actuator inputs within physical limits
    uV = min(max(uCmd(1), cfg.actuator.vLim(1)), cfg.actuator.vLim(2));
    uW = min(max(uCmd(2), cfg.actuator.wLim(1)), cfg.actuator.wLim(2));

    %% 2. Delay buffer and discrete ZOH first-order lag for shaft velocities
    delaySteps = max(0, round(tau / dt));
    plantState.uHistV = [plantState.uHistV, uV];
    plantState.uHistW = [plantState.uHistW, uW];

    idxDelayed = length(plantState.uHistV) - delaySteps;
    if idxDelayed < 1
        uVDel = 0.0;
        uWDel = 0.0;
    else
        uVDel = plantState.uHistV(idxDelayed);
        uWDel = plantState.uHistW(idxDelayed);
    end

    if T <= 1e-6
        vShaft = K * uVDel;
        wShaft = K * uWDel;
    else
        a = exp(-dt / T);
        vShaft = a * plantState.vShaft + (1 - a) * K * uVDel;
        wShaft = a * plantState.wShaft + (1 - a) * K * uWDel;
    end

    plantState.vShaft = vShaft;
    plantState.wShaft = wShaft;

    % Encoder reports shaft increments without seeing slip or calibration error
    encoderDelta = [vShaft * dt; wShaft * dt];

    %% 3. Compute true floor-contact scale factors (mismatch + slip patch)
    x = plantState.poseTrue(1);
    y = plantState.poseTrue(2);
    theta = plantState.poseTrue(3);

    slipV = 0.0;
    slipW = 0.0;
    if scen.slipZone.enabled
        inX = (x >= scen.slipZone.xRange(1)) && (x <= scen.slipZone.xRange(2));
        inY = (y >= scen.slipZone.yRange(1)) && (y <= scen.slipZone.yRange(2));
        if inX && inY
            slipV = scen.slipZone.slipV;
            slipW = scen.slipZone.slipW;
        end
    end

    etaV = scen.etaVBase * (1.0 - slipV);
    etaW = scen.etaWBase * (1.0 - slipW);
    etaTrue = [etaV; etaW];

    vTrue = etaV * vShaft;
    wTrue = etaW * wShaft + scen.biasW;

    %% 4. Integrate true 2D pose (midpoint rule for clean accuracy at dt=0.1)
    thetaMid = theta + 0.5 * wTrue * dt;
    xNext = x + vTrue * dt * cos(thetaMid);
    yNext = y + vTrue * dt * sin(thetaMid);
    thetaNext = wrap_to_pi(theta + wTrue * dt);

    poseTrue = [xNext; yNext; thetaNext];
    plantState.poseTrue = poseTrue;
end
