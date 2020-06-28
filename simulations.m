% Coverage Control
clear; clc; close all

rng(1)
% Robot Positions
num_robots = 3;
num_debris = 1;
a = 2; b = 3;
p = a + (b-a).*rand(num_robots,2);
px = p(:,1);
py = p(:,2);

% Convex Environment
% bnd = [min(px) max(px) min(py) max(py)];
bnd = [0 5 0 5];
Q = [bnd(1) bnd(4);bnd(2) bnd(4);bnd(2) bnd(3);bnd(1) bnd(3);bnd(1) bnd(4)];

% Simulate
simTime = 0:1:100;
robot_start = [px, py];
debris_start = [0.5 0.5];
gain = 5;
sensor_range = 1.5;

[R,D,Mask] = mainSim(simTime, robot_start, debris_start, gain, sensor_range, Q);

p = reshape(R, [2*num_robots,length(simTime), 1])';

plotVoronoiDiagram(p, simTime, bnd, Mask)


