function [gainScale, slipBoost, fisState] = fuzzy_pid_scheduler( ...
    fisState, speedErrNorm, slipOrRateNorm, cfg)
%FUZZY_PID_SCHEDULER Sugeno Fuzzy Logic gain scheduler for Fuzzy-PID (C4).
%   [GAINSCALE, SLIPBOOST, FISSTATE] = FUZZY_PID_SCHEDULER(FISSTATE,
%   SPEEDERRNORM, SLIPORRATENORM, CFG) uses MATLAB Fuzzy Logic Toolbox
%   (sugfis / evalfis) to adapt PIDF gain multipliers [sKp, sKi, sKd] and
%   an outer slip-compensation boost SLIPBOOST online from normalized
%   tracking error and apparent slip/disturbance level.
%
%   Rule intuition:
%     - Small error & low disturbance  -> lower Kp/Kd to stay smooth under noise
%     - Medium error or moderate slip  -> nominal-to-firm PIDF gains
%     - Large error or high slip/load  -> boost Kp, Ki, Kd & outer slip boost

    if isempty(fisState) || ~isfield(fisState, 'fis')
        fis = sugfis('Name', 'FuzzyPID_MobileRobot');

        % Input 1: Normalized error magnitude [0, 1]
        fis = addInput(fis, [0 1], 'Name', 'err');
        fis = addMF(fis, 'err', 'trimf', [0.0 0.0 0.45], 'Name', 'Small');
        fis = addMF(fis, 'err', 'trimf', [0.15 0.50 0.85], 'Name', 'Medium');
        fis = addMF(fis, 'err', 'trimf', [0.55 1.0 1.0], 'Name', 'Large');

        % Input 2: Normalized slip / disturbance intensity [0, 1]
        fis = addInput(fis, [0 1], 'Name', 'slip');
        fis = addMF(fis, 'slip', 'trimf', [0.0 0.0 0.45], 'Name', 'Low');
        fis = addMF(fis, 'slip', 'trimf', [0.15 0.50 0.85], 'Name', 'Moderate');
        fis = addMF(fis, 'slip', 'trimf', [0.55 1.0 1.0], 'Name', 'High');

        % Output 1: Kp multiplier
        fis = addOutput(fis, [0.7 2.2], 'Name', 'sKp');
        fis = addMF(fis, 'sKp', 'constant', cfg.fuzzyPid.kpLevels(1), 'Name', 'Low');
        fis = addMF(fis, 'sKp', 'constant', cfg.fuzzyPid.kpLevels(2), 'Name', 'Med');
        fis = addMF(fis, 'sKp', 'constant', cfg.fuzzyPid.kpLevels(3), 'Name', 'High');

        % Output 2: Ki multiplier
        fis = addOutput(fis, [0.8 2.4], 'Name', 'sKi');
        fis = addMF(fis, 'sKi', 'constant', cfg.fuzzyPid.kiLevels(1), 'Name', 'Low');
        fis = addMF(fis, 'sKi', 'constant', cfg.fuzzyPid.kiLevels(2), 'Name', 'Med');
        fis = addMF(fis, 'sKi', 'constant', cfg.fuzzyPid.kiLevels(3), 'Name', 'High');

        % Output 3: Kd multiplier
        fis = addOutput(fis, [0.6 1.8], 'Name', 'sKd');
        fis = addMF(fis, 'sKd', 'constant', cfg.fuzzyPid.kdLevels(1), 'Name', 'Low');
        fis = addMF(fis, 'sKd', 'constant', cfg.fuzzyPid.kdLevels(2), 'Name', 'Med');
        fis = addMF(fis, 'sKd', 'constant', cfg.fuzzyPid.kdLevels(3), 'Name', 'High');

        % Output 4: Outer slip feedforward boost factor
        fis = addOutput(fis, [1.0 1.4], 'Name', 'boost');
        fis = addMF(fis, 'boost', 'constant', cfg.fuzzyPid.boostLevels(1), 'Name', 'None');
        fis = addMF(fis, 'boost', 'constant', cfg.fuzzyPid.boostLevels(2), 'Name', 'Mild');
        fis = addMF(fis, 'boost', 'constant', cfg.fuzzyPid.boostLevels(3), 'Name', 'Strong');

        % 3x3 Rule Base: [in1 in2 out1 out2 out3 out4 weight op(1=AND)]
        rules = [
            1 1  1 1 1 1  1 1;  % Small err, Low slip      -> soft & smooth
            1 2  2 2 1 2  1 1;  % Small err, Mod slip      -> mild integral & boost
            1 3  2 3 2 3  1 1;  % Small err, High slip     -> strong boost & Ki
            2 1  2 2 2 1  1 1;  % Med err, Low slip        -> nominal PIDF
            2 2  2 2 2 2  1 1;  % Med err, Mod slip        -> firm PIDF + mild boost
            2 3  3 3 3 3  1 1;  % Med err, High slip       -> aggressive + strong boost
            3 1  3 2 3 1  1 1;  % Large err, Low slip      -> fast transient recovery
            3 2  3 3 3 2  1 1;  % Large err, Mod slip      -> high Kp/Ki/Kd + boost
            3 3  3 3 3 3  1 1;  % Large err, High slip     -> maximum recovery
        ];
        fis = addRule(fis, rules);
        fisState.fis = fis;
    end

    in1 = min(max(speedErrNorm, 0.0), 1.0);
    in2 = min(max(slipOrRateNorm, 0.0), 1.0);

    out = evalfis(fisState.fis, [in1, in2]);
    gainScale = out(1:3);
    slipBoost = out(4);
end
