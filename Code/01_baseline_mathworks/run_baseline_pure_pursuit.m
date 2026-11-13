%% Baseline Path Following for a Differential Drive Robot (Ideal S0)
% Reference reproduction based on official MathWorks Robotics System Toolbox /
% Navigation Toolbox workflow: "Path Following for a Differential Drive Robot"
% using differentialDriveKinematics + controllerPurePursuit.
%
% Purpose:
%   1) Verify MATLAB R2024b toolboxes and reproduce the unmodified ideal
%      kinematic baseline (no sensor noise, no slip, no model mismatch).
%   2) Save reference trajectory data and plots to 01_baseline_mathworks/results/
%      to serve as the ideal upper-bound benchmark (S0) before introducing
%      uncertainty, EKF, and PI (SIMC) extensions in subsequent modules.

clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
codeRoot = fileparts(scriptDir);
addpath(fullfile(codeRoot, 'utils'));

resultsDir = fullfile(scriptDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% 1. Define Reference Waypoints and Initial Pose
path = [2.00  1.00;
        1.25  1.75;
        5.25  8.25;
        7.25  8.75;
        11.75 10.75;
        12.00 10.00];

robotInitialLocation = path(1, :);
robotGoal = path(end, :);
initialOrientation = 0; % [rad]

robotCurrentPose = [robotInitialLocation, initialOrientation]';

%% 2. Initialize Kinematic Model and Pure Pursuit Controller (MathWorks API)
robot = differentialDriveKinematics( ...
    'TrackWidth', 1.0, ...
    'VehicleInputs', 'VehicleSpeedHeadingRate');

controller = controllerPurePursuit( ...
    'Waypoints', path, ...
    'DesiredLinearVelocity', 0.3, ...
    'MaxAngularVelocity', 2.0, ...
    'LookaheadDistance', 0.3);

%% 3. Run Simulation Loop
sampleTime = 0.1;       % [s]
goalRadius = 0.1;       % [m]
maxSimTime = 120.0;     % [s] safety cap

maxSteps = ceil(maxSimTime / sampleTime) + 1;
tHist = zeros(maxSteps, 1);
poseHist = zeros(maxSteps, 3);
vHist = zeros(maxSteps, 1);
wHist = zeros(maxSteps, 1);
lookaheadHist = zeros(maxSteps, 2);

stepIdx = 1;
poseHist(stepIdx, :) = robotCurrentPose';
distanceToGoal = norm(robotInitialLocation - robotGoal);

while (distanceToGoal > goalRadius) && (stepIdx < maxSteps)
    % Compute control commands from Pure Pursuit using true pose (ideal baseline)
    [v, omega, lookaheadPt] = controller(robotCurrentPose);

    % Log command at current step
    vHist(stepIdx) = v;
    wHist(stepIdx) = omega;
    lookaheadHist(stepIdx, :) = lookaheadPt;

    % Advance robot kinematics using Euler integration
    vel = derivative(robot, robotCurrentPose, [v, omega]);
    robotCurrentPose = robotCurrentPose + vel * sampleTime;
    robotCurrentPose(3) = wrap_to_pi(robotCurrentPose(3));

    stepIdx = stepIdx + 1;
    tHist(stepIdx) = (stepIdx - 1) * sampleTime;
    poseHist(stepIdx, :) = robotCurrentPose';

    distanceToGoal = norm(robotCurrentPose(1:2)' - robotGoal);
end

% Trim unused preallocated rows
tHist = tHist(1:stepIdx);
poseHist = poseHist(1:stepIdx, :);
vHist = vHist(1:stepIdx);
wHist = wHist(1:stepIdx);
lookaheadHist = lookaheadHist(1:stepIdx, :);

%% 4. Compute Baseline Quantitative Metrics
metrics = compute_path_metrics(tHist, poseHist, path, vHist(1:end-1), wHist(1:end-1));

fprintf('=== Baseline S0 (MathWorks Pure Pursuit - Ideal) ===\n');
fprintf('Reached goal        : %d (final error = %.4f m)\n', ...
    metrics.finalGoalError <= goalRadius, metrics.finalGoalError);
fprintf('Completion time     : %.2f s (%d steps)\n', metrics.completionTime, stepIdx - 1);
fprintf('Cross-track RMSE    : %.4f m\n', metrics.rmseCrossTrack);
fprintf('Max cross-track err : %.4f m\n', metrics.maxCrossTrack);
fprintf('Path length (act/ref): %.3f m / %.3f m\n', ...
    metrics.pathLengthActual, metrics.pathLengthRef);
fprintf('Control RMS (v, w)  : %.4f m/s, %.4f rad/s\n', metrics.vRms, metrics.wRms);

%% 5. Plot & Save Baseline Results
figVisible = 'on';
if batchStartupOptionUsed
    figVisible = 'off';
end

fig = figure('Visible', figVisible, 'Position', [100 100 1000 420], 'Color', 'w');

subplot(1, 2, 1);
plot(path(:, 1), path(:, 2), 'k--o', 'LineWidth', 1.4, 'MarkerSize', 5, ...
    'DisplayName', 'Reference Waypoints');
hold on; grid on; axis equal;
plot(poseHist(:, 1), poseHist(:, 2), 'b-', 'LineWidth', 1.8, ...
    'DisplayName', 'Robot Trajectory (Ideal)');
plot(path(1, 1), path(1, 2), 'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g', ...
    'DisplayName', 'Start');
plot(path(end, 1), path(end, 2), 'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
    'DisplayName', 'Goal');
xlim([0 13]); ylim([0 13]);
xlabel('x [m]'); ylabel('y [m]');
title('Baseline S0: Pure Pursuit Trajectory');
legend('Location', 'southeast');

subplot(1, 2, 2);
yyaxis left;
plot(tHist, metrics.crossTrackError, 'b-', 'LineWidth', 1.5);
ylabel('Cross-Track Error [m]');
yyaxis right;
plot(tHist, wHist, 'r--', 'LineWidth', 1.2);
ylabel('Angular Velocity \omega [rad/s]');
grid on;
xlabel('Time [s]');
title(sprintf('Cross-Track Error (RMSE = %.4f m)', metrics.rmseCrossTrack));

exportgraphics(fig, fullfile(resultsDir, 'baseline_s0_trajectory.png'), 'Resolution', 200);
save(fullfile(resultsDir, 'baseline_s0_results.mat'), ...
    'path', 'tHist', 'poseHist', 'vHist', 'wHist', 'lookaheadHist', 'metrics', 'sampleTime');

fprintf('Saved baseline outputs to: %s\n', resultsDir);
