function [ commands ] = TestControlLawWp(t, states )
%TestControlLaw Summary of this function goes here
%   Detailed explanation goes here

n_robots = size(states, 1);

commands  = zeros(n_robots, 4);
commands(1,1) = 0;
commands(1,2) = 0;
commands(1,3) = 1;
commands(1,4) = pi/2;

if n_robots > 1
    commands(2,1) = -.2;
    commands(2,2) = -.3;
    commands(2,4) = pi/2;
end
end

