clear all
close all

%% Gobal Variables
% Dobot base transforms
dobotBase = transl(0,0,0);
paperBase = transl(0.2875,0,0);
pencilBase = transl(0,-0.3,0.055);

% determines if the perspex is plotted or not. set from GUI
perspexOn = false;

%% Main
% Set up equipment
dobot = Dobot(dobotBase);
environment = Environment(paperBase, perspexOn);
pencil = Pencil(pencilBase);
axis equal

%% Max's To Do:
% I will be responsible for:
% - Finalising the environment layout
%   -Use toggle to inlcude or not include perspex in drawing
%   - Add pen holder into environment, and new pen class
% - Perform RMRC to draw shapes, including scaling the shapes.
% - Sensing the paper paper location using IBVS
%% Yuxiang To Do:
% Yuxiang will be responsible for:
% - Moving the robot to pick up pencil. this includes transforming the end effector to the pencil tip.
% - Obstacle avoidance, by not colliding with itself or the table.
% - completing the GUI and writing the teach function for the x,y,z of the end effector and joint angle control.
