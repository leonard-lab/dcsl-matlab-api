function [ commands ] = WpControlLaw(t, states, waypoint)
%TestControlLaw Summary of this function goes here
%   Detailed explanation goes here

global a_h r_h p_h d_h k_h

k1 = 1;
k2 = 2;
k3 = 2;
k4 = 1;

v_max = 0.5;
beta = 0.2;
lambda = 2;
omega_max = 1;

r_park = 0.25;
r_ok = 0.10;

kz = 5;

x = states(1,:);

r = sqrt((x(1)-waypoint(1))^2 + (x(2)-waypoint(2))^2);

% Drive to waypoint
alpha = atan2(waypoint(2)-x(2), waypoint(1)-x(1));
psi = waypoint(4) - alpha;
delta = x(6) - alpha;
% 
% if (r < r_park && (delta > pi/2 || delta < -pi/2)) % Robot is past waypoint but still close
%     delta = -x(6) - alpha;
%     psi = -waypoint(4) - alpha;
%     reverse = true;
% else
%     reverse = false;
% end

delta = wrapToPi(delta);
psi = wrapToPi(psi);

r_h(end+1,:) = [t r];
p_h(end+1,:) = [t psi];
d_h(end+1,:) = [t delta];
a_h(end+1,:) = [t alpha];

kappa = -1/r * (k2*(delta-atan(-k1*psi))+(1+k1/(1+(k1*psi)^2))*sin(delta));

k_h(end+1,:) = [t kappa];

v = v_max/(1+beta*abs(kappa)^lambda);

% if r < r_park
%     v = k3*r; % Slow to park
%     disp('Parking')
% end

omega = kappa*v;

if r < r_park
    v = k3*r*cos(psi);
    disp('parking')
    omega = k4*(waypoint(4) - x(6));
%     if psi > -pi/2 || psi < pi/2
%         v = -v;
%         disp('and reversing')
%     end
end


% if reverse == true
%     v = -v;
%     omega = -omega;
%     disp('Reversing')
% end

vz = -kz*(x(3) - waypoint(3));

% if r < r_ok
%     v = 0;
%     omega = k4*(waypoint(4) - x(6));
%     disp('r ok!')
%     if abs(psi) < -0.2
%         omega = 0;
%         disp('theta ok!');
%     end
% end

if abs(v) - v_max > 0
    v = sign(v)*v_max;
end

if abs(omega) > omega_max
    omega = sign(omega) * omega_max;
end

u = [v omega vz];

commands(1, :) = u;

end

