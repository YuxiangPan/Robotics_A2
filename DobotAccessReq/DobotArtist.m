clf;
close all;
clc;
%% Dobot 

L1 = Link('d',0.135,'a',0,'alpha',-pi/2,'offset',deg2rad(0), 'qlim',deg2rad([-135 135]));
L2 = Link('d',0,'a',0.135,'alpha',0,deg2rad(-90), 'qlim',deg2rad([5 80]));
L3 = Link('d',0,'a',0.147,'alpha',0,'offset',deg2rad(0),'qlim', deg2rad([15 170]));
L4 = Link('d',0,'a',0.05,'alpha',pi/2,'offset',deg2rad(-90),'qlim',deg2rad([-90 90]));
L5 = Link('d',-0.05,'a',0,'alpha',0,'qlim',deg2rad([-85 85]));
robot1 = SerialLink([L1 L2 L3 L4 L5], 'name', 'DobotArtist');

workspace  = [-0.6 0.6 -0.6 0.6 -0.2 0.6];
scale = 0.5;
robot1.plot('workspace',workspace,'scale', scale);