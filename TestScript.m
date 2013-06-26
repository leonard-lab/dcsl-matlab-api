clear m
initial_poses = [0 0 0 0];
runtime = 2;
noise = [0.01, 0.01, 0, 0.01];
m = Miabots(initial_poses, @TestControlLaw, 'velocity', runtime, 'sim', true);%, 'sim_noise', noise);
m.start
plot(m.get_history(1,'x'), m.get_history(1,'y'))%,m.get_history(2,'x'),m.get_history(2,'y'));
axis('equal');