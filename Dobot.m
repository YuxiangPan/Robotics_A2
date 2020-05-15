%% Notes
% gripper may need to be added into the actual end piece.
% joint 4 is always vertical ie. theta4 = 360 - theta3 -theta2 (for inverse kinematics roll and pitch must be masked out)

%%
classdef Dobot < handle
    %Dobot: Creates object for Dobot robot
    properties
        % Dobot model
        model;
        
        % Initial workspace size
        workspace = [-2 2 -2 2 -0.3 2];   
        
        % Common Joint angles
        %   define the workspace vectors:
        %       qz         zero joint angle configuration
        %       qr         vertical 'READY' configuration, arm up
        %       qstretch   arm is stretched out in the X direction
        %       qn         arm is at a nominal non-singular configuration
        qz = [0, 0, 0, 0, 0, 0];
        qr = [0, -pi/2, pi/2, 0, 0, 0];
        qs = [pi/2, 0, 0, -pi/2, 0, 0];
        qn = [0, -pi/2, 0, -pi/2, 0, 0];
    end
    
    methods
        %% Constructor
        function self = Dobot()
            self.GetDobotRobot();
            self.PlotAndColourRobot();
        end
        
         %% Set up Dobot Properties
        function GetDobotRobot(self, baseTr)
            % Give robot a unique name
            pause(0.001);
            name = ['Dobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            % DH parameters of UR3 Robot
            L1 = Link('d', 0.08, 'a', 0, 'alpha', deg2rad(90), 'qlim', [deg2rad(-90) deg2rad(90)]);
            L2 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(0) deg2rad(85)]);
            L3 = Link('d', 0, 'a', 0.160, 'alpha', 0, 'qlim', [deg2rad(-10) deg2rad(95)]);
            L4 = Link('d', 0.11235, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-90) deg2rad(90)]);

            
            % Create UR3 robot
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
            self.model.base = baseTr;
        end
        
    end
end

