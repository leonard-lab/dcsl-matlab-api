clear m
close all
initial_poses = [0 0 0 0; 0 0 0 0];
runtime = 5;
noise = [0.01, 0.01, 0, 0.01];
m = Belugas(initial_poses, @TestControlLaw, 'direct', runtime, 'sim', false);%, 'sim_noise', noise);
m.start

%%
plot(m.get_history(1,'x'), m.get_history(1,'y'))%,m.get_history(2,'x'),m.get_history(2,'y'));
title('x-y')
axis('equal');
figure;
plot(m.get_history(1,'state_times'), m.get_history(1,'z')); title('z');
figure;
plot(m.get_history(1,'state_times'), m.get_history(1,'x')); title('x');
