function [ commands ] = TestControlLaw(~, states )
%TestControlLaw Summary of this function goes here
%   Detailed explanation goes here

n_robots = size(states, 1);

commands  = zeros(n_robots, 3);
commands(:,1) = ones(n_robots, 1);
commands(:,2) = ones(n_robots, 1)*0.5;


end

