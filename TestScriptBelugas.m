clear m
close all
initial_poses = [0 0 0 0];
runtime = 10;
noise = [0.01, 0.01, 0, 0.01];
m = Belugas(initial_poses, @TestControlLaw, 'velocity', runtime, 'sim', false);%, 'sim_noise', noise);
m.start('use_initial_poses', false)

%%
m.shutdown();
plot(m.get_history(1,'x'), m.get_history(1,'y'))%,m.get_history(2,'x'),m.get_history(2,'y'));
title('x-y')
axis('equal');
m.plot_history(1);
