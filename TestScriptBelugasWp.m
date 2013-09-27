clear all
close all

global r_h p_h d_h k_h a_h

initial_poses = [0 0 1.2 0];
runtime = 30;
noise = [0.01, 0.01, 0, 0.01]*0;

a_h = [0 0];
r_h = [0 0];
p_h = [0 0];
d_h = [0 0];
k_h = [0 0];

waypoint = [1 1 1.2 pi/2];

cl = @(t,x) WpControlLaw(t,x, waypoint);

m = Belugas(initial_poses, cl, 'velocity', runtime, 'sim', false, 'sim_noise', noise);
m.start('use_initial_poses', false)

%%
m.shutdown();
plot(m.get_history(1,'x'), m.get_history(1,'y'))%,m.get_history(2,'x'),m.get_history(2,'y'));
title('x-y')
axis('equal');
m.plot_history(1);


figure
subplot(5,1,1); plot(r_h(:,1), r_h(:,2)); ylabel('r');
subplot(5,1,2); plot(p_h(:,1), p_h(:,2)); ylabel('psi');
subplot(5,1,3); plot(d_h(:,1), d_h(:,2)); ylabel('delta');
subplot(5,1,4); plot(k_h(:,1), k_h(:,2)); ylabel('kappa');
subplot(5,1,5); plot(a_h(:,1), a_h(:,2)); ylabel('alpha');
xlabel('Time (s)');

%gen_velocity_plot