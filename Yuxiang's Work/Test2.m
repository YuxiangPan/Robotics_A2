close all;
clear;
clc;

%% Set DobotModel
dobot = Dobot(transl(0,0,0));
hold on;
Bh = transl(0.3,0,0);
Bh = Move.Load3D('blackhexagon.ply',Bh);
%% 
% blackhexagon = plyread('blackhexagon.ply','tri');
q = [0, pi/4, pi/4, 3*pi/2, 0];
Move.SetJoint(dobot,q);
T = transl(0.3,0,0.1);
Move.DobotMove(Dobot,T);
T =  transl(0.3,0,0.03);
Move.DobotMove(Dobot,T);
T =  transl(0.3,0,0.08);
Move.PickPen(dobot,Bh,T,1);
q = [0, pi/4, pi/4, 3*pi/2, 0];
% T = transl(0.3,0,0.14)
Move.PickPen(dobot,Bh,T,0);
% Move.MovePen(dobot,blackhexagon,q);
