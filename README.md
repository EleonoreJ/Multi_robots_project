# Multi_robots_project

## Pusuers-Evador Problem

The pursuit-evasion problem is approached based on the Voronoi coverage which decomposes the environment into Voronoi cells. 
The Voronoi cells are convex sets which gather the nearest points. The robots are controlled to reach the centroid of their cell.

## Lloyds Algorithms: Main File

The Lloyd_algorithms is modified to define the pursuers and evador strategies as follow:
  - Evadors: controlled to reach the furthest vertice
  - Pursuers: controlled to reach the evador position to catch it. 

Command to exexcut the simulation:
lloydsAlgorithm([2;10*rand(7,1)],[4;5*rand(7,1)], [0,0;0,1;1,1;1,0], 100, true)
