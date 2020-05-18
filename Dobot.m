classdef Dobot < handle
    %Dobot: Creates object for Dobot robot
    properties
        % Dobot model
        model;
        
        % Initial workspace size
        workspace = [-0.6 0.6 -0.6 0.6 -0.3 0.6];   
        
        % Common Joint angles
        %   define the workspace vectors:
        %       qz         zero joint angle configuration
        %       qr         vertical 'READY' configuration, arm up
        %       qstretch   arm is stretched out in the X direction
        %       qn         arm is at a nominal non-singular configuration
        qz = [0, 0, 0, 0, 0];
        qr = [0, 17*pi/36, pi/18, -19*pi/36, 0];
        qs = [0, 0, 0, 0, 0];
        qn = [0, pi/4, pi/4, 3*pi/2, 0];
    end
    
    methods
        %% Constructor
        function self = Dobot()
            self.GetDobotRobot();
            self.PlotAndColourRobot();
        end
        
         %% Set up Dobot Properties
        function GetDobotRobot(self)
            % Give robot a unique name
            pause(0.001);
            name = ['Dobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            % DH parameters of Dobot Robot
            L1 = Link('d', 0.139, 'a', 0, 'alpha', deg2rad(-90),'offset', deg2rad(0), 'qlim', [deg2rad(-90) deg2rad(90)]);
            L2 = Link('d', 0, 'a', 0.135, 'alpha', deg2rad(0),'offset', deg2rad(-90), 'qlim', [deg2rad(0) deg2rad(85)]);
            L3 = Link('d', 0, 'a', 0.147, 'alpha', deg2rad(0),'offset', deg2rad(0), 'qlim', [deg2rad(-10) deg2rad(95)]);
            L4 = Link('d', 0, 'a', -0.061, 'alpha', deg2rad(90),'offset', deg2rad(-90), 'qlim', [deg2rad(180) deg2rad(370)]);
            L5 = Link('d', 0.09191, 'a', 0, 'alpha', deg2rad(0),'offset', deg2rad(180), 'qlim', [deg2rad(-90) deg2rad(90)]); % Include the gripper in the last joint
            
            % Create Dobot robot
            self.model = SerialLink([L1 L2 L3 L4 L5],'name',name);
        end
        
        %% Plot and try to colour the Robot
        function PlotAndColourRobot(self)
            reloadData = 0; % 1 = reload data, 0 = use preloaded data
            switch reloadData
                case 0
                    load('dobotLinks/DobotLinksDataPreloaded.mat');
                case 1
                    for linkIndex = 0:self.model.n
                        [faceData{linkIndex+1}, vertexData{linkIndex+1}, plyData{linkIndex+1}]  = plyread(['dobotLinks/Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                    end
                otherwise
                    error('reloadData = 0 to use preloaded 3D data, or 1 reload 3D data')
            end
            for linkIndex = 0:self.model.n
                self.model.faces{linkIndex+1} = faceData{linkIndex+1};
                self.model.points{linkIndex+1} = vertexData{linkIndex+1};
            end
            
            self.model.plot3d(self.qn,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0.001;
            
            % Try to colour the arm
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        %% Calculate joint 4 angle (the uncontrollable Joint)
        function q = CalcJointAngles(self, joints)
            % Calculates joint angles particularly the uncontrollable
            % joint (joint 4). pass vector with 4 controllable joints 
            % it will return a joint angle with the joints with joint 4 at the
            % correct angle
            q = [joints, zeros(size(joints,1),1)];
            if size(joints,2) ~= 4
                error('Pass in all 4 controllable joint angles');
            else
                % Move the last controllable joint to the 5th column
                q(:,5) = q(:,4);
                % Calculate the angles of the uncontrollable joint
                q(:,4) = 2*pi-joints(2)-joints(3);
            end
        end
    end
end