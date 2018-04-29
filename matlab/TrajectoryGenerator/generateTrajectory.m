start = [0, 0, 0, 0, 0;0, 0, 0, 0, 0;2, 0, 0, 0, 0];
mid = [-5; -3;2.5;0; -5; 2.5; 10; 0; 2.5; 5; 5; 2.5];
final = [10, 0, 0, 0, 0;10, 0, 0, 0, 0; 2, 0, 0, 0, 0]


[traj, flightTime] = minimumTimeTrajectoryGenerator(start, mid, final, 'VEL', 5, [1.5, 1.5, 1], 10, -45, 100, pi/6, 1)

%plot the trajectory
clf;
[p1, p2] = trajectoryPlotter(traj);
daspect([5 5 5])
xlim([-10, 15])
ylim([-10, 15])
zlim([-10, 10])
hold on
arrow3(p1, p2, 'b', 0.4)
title('Initial Trajectory')
xlabel('x (m)')
ylabel('y (m)')
grid on;
savefig('initial_trajectory');
hold off

[traj, flightTime] = minimumTimeTrajectoryGenerator(start, mid, final, 'VEL', 5, [1.5, 1.5, 1], 10, -45, 100, pi/6, 10)

%plot the trajectory
clf;
[p1, p2] = trajectoryPlotter(traj);
daspect([5 5 5])
xlim([-10, 15])
ylim([-10, 15])
zlim([-10, 10])
hold on
arrow3(p1, p2, 'b', 0.4)
title('Final Trajectory')
xlabel('x (m)')
ylabel('y (m)')
grid on;
savefig('final_trajectory');
hold off