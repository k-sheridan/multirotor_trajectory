start = [0, 0, 0, 0, 0;0, 0, 0, 0, 0;0, 0, 0, 0, 0];
mid = [5; 2;2.5;1; -5; -3]
final = [10, 0, 0, 0, 0;5, 0, 0, 0, 0; 0.3, 0, 0, 0, 0]

[traj, flightTime] = minimumTimeTrajectoryGenerator(start, mid, final, 'VEL', 5, [1.5, 1.5, 1], 10, -45, 100, pi/6)