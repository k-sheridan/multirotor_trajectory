function [ p1, p2 ] = trajectoryPlotter( Traj )
%UNTITLED Summary of this function goes here
%   plots the trajectory

[trajRow, trajCol, trajZ] = size(Traj);

time = 0;

p1 = [polyval(Traj(1, 1:10, 1), (time:0.1:time + Traj(1, 11, 1)))', polyval(Traj(2, 1:10, 1), (time:0.1:time + Traj(1, 11, 1)))', polyval(Traj(3, 1:10, 1), (time:0.1:time + Traj(1, 11, 1)))'];
pAccel = [polyval(polyder(Traj(1, 1:10, 1)), (time:0.1:time + Traj(1, 11, 1)))', polyval(polyder(Traj(2, 1:10, 1)), (time:0.1:time + Traj(1, 11, 1)))', polyval(polyder(Traj(3, 1:10, 1)), (time:0.1:time + Traj(1, 11, 1)))'];
time = time + Traj(1, 11, 1);

dt_vec = [];

for z_index = (2:1:trajZ)
    p1 = [p1; [polyval(Traj(1, 1:10, z_index), (0:0.1:Traj(1, 11, z_index)))', polyval(Traj(2, 1:10, z_index), (0:0.1:Traj(1, 11, z_index)))', polyval(Traj(3, 1:10, z_index), (0:0.1:Traj(1, 11, z_index)))']];
    pAccel = [pAccel; [polyval((polyder(Traj(1, 1:10, z_index))), (0:0.1:Traj(1, 11, z_index)))', polyval((polyder(Traj(2, 1:10, z_index))), (0:0.1:Traj(1, 11, z_index)))', polyval((polyder(Traj(3, 1:10, z_index))), (0:0.1:Traj(1, 11, z_index)))']];
    time = time + Traj(1, 11, z_index);
    dt_vec = [dt_vec, Traj(1, 11, z_index)];
end

dt_vec

p2 = p1 + (pAccel * 0.4);

end

