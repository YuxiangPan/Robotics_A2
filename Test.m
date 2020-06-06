%% RMRC

clear all
close all

dobot = Dobot(transl(0,0,0))
q = [-pi/4, pi/4, pi/4, 3*pi/2, 0];
dobot.model.animate(q)
qGoal = [pi/4, pi/4, pi/4, 3*pi/2, 0];
goalTr = dobot.model.fkine(qGoal);

% qMatrix = dobot.PathToDesiredTransform(goalTr,dobot.qn);
% points = [];
% for i = 1:size(qMatrix,1)
%     dobot.model.animate(qMatrix(i,:));
%     endTr = dobot.model.fkine(qMatrix(i,:));
%     point = endTr(1:3,4);
%     points(:,i) = point;
%     %plot3(point');
% end
% plot_sphere(points, 0.01, 'r');
pencil = Pencil(transl(0.5,0,0));
pause(1);
dobot.model.animate(q);
qMatrix = dobot.StraightMovementToNewTransform(goalTr);
% dobot.model.tool = transl(0,0,0.055);
dobot.Draw(qMatrix, pencil);
%% Collision
clear all
close all
clc

dobot = Dobot(transl(0,0,0));
dobot.CollisionMode();
dobot.model.teach;
%%
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])        
robot = SerialLink([L1 L2 L3],'name','myRobot');  

% New values for the ellipsoid (guessed these, need proper model to work out correctly)
centerPoint = [0,0,0];
radii = [1,0.5,0.5];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:4
    robot.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.faces{i} = delaunay(robot.points{i});    
    warning on;
end

robot.plot3d([0,0,0]);
axis equal
camlight
% robot.teach
% keyboard

%% adf
a = true;
while a == true;
    disp('before change')
    a = false;
    pause(1);
    disp('after change')
end