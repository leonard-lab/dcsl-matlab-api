function [ commands ] = WpControlLaw(t, states)
%TestControlLaw Summary of this function goes here
%   Detailed explanation goes here

global r_h p_h d_h k_h

k1 = 1;
k2 = 4;
k3 = 0.5;

v_max = 0.5;
beta = 0.2;
lambda = 2;
omega_max = 1;

r_park = 0.5;

kz = 5;

waypoint = [1 1 1.2 pi/2];
x = states(1,:);

r = sqrt((x(1)-waypoint(1))^2 + (x(2)-waypoint(2))^2);

% Drive to waypoint
alpha = atan2(waypoint(2)-x(2), waypoint(1)-x(1));
psi = waypoint(4) - alpha;
delta = x(6) - alpha;

if (r < r_park && (delta > pi/2 || delta < -pi/2)) % Robot is past waypoint but still close
    delta = -x(6) - alpha;
    psi = -waypoint(4) - alpha;
    reverse = true;
else
    reverse = false;
end

r_h(end+1,:) = [t r];
p_h(end+1,:) = [t psi];
d_h(end+1,:) = [t delta];

kappa = -1/r * (k2*(delta-atan(-k1*psi))+(1+k1/(1+(k1*psi)^2))*sin(delta));

k_h(end+1,:) = [t kappa];

v = v_max/(1+beta*abs(kappa)^lambda);

if r < r_park
    v = k3*r; % Slow to park
end

omega = kappa*v;

if abs(v) - v_max > 0
    v = v_max;
end

if abs(omega) > omega_max
    omega = sign(omega) * omega_max;
end

if reverse == true
    v = -v;
    omega = -omega;
end

vz = -kz*(x(3) - waypoint(3));

u = [v omega vz];

commands(1, :) = u;

end

