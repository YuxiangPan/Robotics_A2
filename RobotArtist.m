clear all
close all

%% Gobal Variables
% Dobot base transforms
dobotBase = transl(0,0,0);
paperBase = transl(0.2875,0,0);
pencilBase = transl(0,-0.25,0.06);

% Determines if the perspex is plotted or not. set from GUI
perspexOn = false;

% Determines if the normal process is run or if dobot_q is demonstrated
showDobotQ = false;

%% Test dobot_q
if showDobotQ == true
    dobot = Dobot(transl(0,0,0));
    load('dobot_q.mat');
    dobot.model.animate(dobot.CalcJointAngles(dobot_q));
    
%% Main
else
    dobot = Dobot(dobotBase);
    environment = Environment(paperBase, pencilBase, perspexOn);
    pencil = Pencil(pencilBase);
    axis equal;
    
%     T = transl(0,-0.3,0.06)*rpy2tr(pi,0,0);
%     M = [1,1,1,0,0,0];
%     q0 = [0 0.7854 0.7854 4.7124 0];
%     q = dobot.model.ikine(T,q0,M)
%     dobot.model.animate(q);
    
%     T = transl(0,-0.3,0.06)*rpy2tr(pi,0,0);
%     M = [1,1,1,1,1,0];
%     q0 = [0 0.7854 0.7854 4.7124 0];
%     q = dobot.model.ikine(T,q0,M)
%     dobot.model.animate(q);

    T = transl(0.3,0,0.2)*rpy2tr(pi,0,0);
    q0 = [0 0.7854 0.7854 4.7124 0];
    q = dobot.model.ikcon(T,q0)
    dobot.model.animate(q);
%     
%     dobot.PickPencil(pencil);
end
%% Max's To Do:
% I will be responsible for:
% - Finalising the environment layout: Done
%   - Use toggle to inlcude or not include perspex in drawing: Done
%   - Add pen holder into environment, and new pen class: Done
% - Perform RMRC to draw shapes, including scaling the shapes
% - Sensing the paper paper location using IBVS
%% Yuxiang To Do:
% Yuxiang will be responsible for:
% - Moving the robot to pick up pencil. this includes transforming the end effector to the pencil tip.
% - Obstacle avoidance, by not colliding with itself or the table.
% - completing the GUI and writing the teach function for the x,y,z of the end effector and joint angle control.
