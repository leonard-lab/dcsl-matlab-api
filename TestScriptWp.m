clear m
initial_poses = [0 0 0 0; -0.2 0 0 0];
runtime = 20;
noise = [0.01, 0.01, 0, 0.01];
m = Miabots(initial_poses, @TestControlLawWp, 'waypoint', runtime, 'sim', false);%, 'sim_noise', noise);
m.start
%%
plot(m.get_history(1,'x'), m.get_history(1,'y'), 'b', m.get_history(1, 'u1'), m.get_history(1, 'u2'), 'b--', ...
    m.get_history(2,'x'),m.get_history(2,'y'), 'r', m.get_history(2, 'u1'), m.get_history(2, 'u2'), 'r--');
axis('equal');