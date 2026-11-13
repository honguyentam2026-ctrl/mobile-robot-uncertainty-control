function angle = wrap_to_pi(angle)
%WRAP_TO_PI Wrap angle(s) in radians to the interval [-pi, pi].
%   ANGLE = WRAP_TO_PI(ANGLE) normalizes scalar or array angles in radians
%   into [-pi, pi] without requiring external toolbox dependencies.

    angle = mod(angle + pi, 2*pi) - pi;
    angle(angle == -pi & sin(angle) == 0) = pi; % preserve +pi if exact
    % Cleaner canonical wrapping using atan2(sin, cos):
    angle = atan2(sin(angle), cos(angle));
end
