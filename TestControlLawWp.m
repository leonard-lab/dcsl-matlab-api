function [ commands ] = TestControlLawWp(t, states )
%TestControlLaw Summary of this function goes here
%   Detailed explanation goes here

n_robots = size(states, 1);

commands  = zeros(n_robots, 4);
commands(1,1) = .5;
commands(1,2) = .2;
commands(1,4) = -pi/2;

commands(2,1) = -.2;
commands(2,2) = -.3;
commands(2,4) = pi/2;
end

