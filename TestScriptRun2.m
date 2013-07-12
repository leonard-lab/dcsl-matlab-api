clear m
initial_poses = [0.5 0 0 0; -0.2 0.3 0 pi/2];
runtime = 10;
noise = [0.01, 0.01, 0, 0.01];
m = Miabots(initial_poses, @TestControlLaw, 'velocity', runtime, 'sim', false);

%%
m.start
%%
m.shutdown
%%
plot(m.get_history(1,'x'), m.get_history(1,'y'), m.get_history(2,'x'), m.get_history(2,'y'));
axis('equal');