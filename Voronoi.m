%% Voronoi Coverage Controller
% Stanford University 2019
% AA 277 - Multi Robot Control Project
% Stephen Palmieri - 06275330

x= 1:10;
y = 1:10;
%pairwise potential function 
phiq = 1;

%grid of all points
[x,y] = meshgrid(x,y);
%initial positions of robots
n = 2; %number of robots
xpos = [1,2,5,4,3]';
ypos = [1,2,6,0,2]';
numIterations = 10;
showPlot = true;
crs = [0,0;0,10;10,10;10,0]; %this is a square environment
crs = [0,0; 
       0,20;
       10,20;
       10,10;
       20,20;
       20,0;
       15,10;
       15,20]; 

%testing coverage controller
[Px,Py] = lloydsAlgorithm(xpos,ypos,crs,numIterations, showPlot);