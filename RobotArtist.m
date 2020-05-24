clear all
close all

%% Gobal Variables
% Dobot base transforms
dobotBase = transl(0,0,0.1);

%% Main
% Set up equipment
dobot = Dobot(dobotBase);
environment = Environment();
axis equal