function [k,c,w] = cor2SpringDamperParams(e,ball_mass)
% Contact force parameters (these will work with ode23t RelTol 1e-4 and 
% AbsTol 1e-5 with Auto scale disabled). Stiffness and damping coefficients 
% are approximate values, converted from coefficient of restitution.
%
% Ref. paper: Nagurka, Huang, A Mass-Spring-Damper Model of a Bouncing
% Ball, pg.3-4

% Copyright 2021, The MathWorks, Inc.

dT = 3.4e-3;    % contact time for 70th bounce in the paper pg.4

k  = ball_mass / dT^2 * ( pi^2 + log(e)^2 );     % N/m
c  = -2 * ball_mass / dT * log(e);               % N/(m/s)
w  = 1e-3;      % m

c = 5;  % 5 N/(m/s) reduces oscillations in the simulation