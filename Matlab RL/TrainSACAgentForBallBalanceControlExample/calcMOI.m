function ball_moi = calcMOI(ball_radius,ball_shell,ball_mass)
% Moment of inertia of a hollow sphere with inner and outer radii r1 and r2:
%    I = 2/5 * m * (r2^5 - r1^5) / (r2^3 - r1^3)

% Copyright 2021, The MathWorks, Inc.

r1       = ball_radius - ball_shell;
r2       = ball_radius;
I        = 2/5 * ball_mass * (r2^5 - r1^5) / (r2^3 - r1^3);
ball_moi = I * ones(1,3);   % kg*m^2