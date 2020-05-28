classdef Dobot < handle
    %Dobot: Creates object for Dobot robot
    properties
        % Dobot model
        model;
        
        % Camera
        cam
        
        % Initial workspace size
        workspace = [-0.6 0.6 -1 1 -0.65 0.7];   
        
        % Common Joint angles
        %   define the workspace vectors:
        %       qz         zero joint angle configuration
        %       qr         vertical 'READY' configuration, arm up
        %       qstretch   arm is stretched out in the X direction
        %       qn         arm is at a nominal non-singular configuration
        qz = [0, 0, 0, 2*pi, 0];
        qr = [0, 0, 0, 2*pi, 0];
        qs = [0, 17*pi/36, pi/36, 3*pi/2, 0];
        qn = [0, pi/4, pi/4, 3*pi/2, 0];
    end
    
    methods
        %% Constructor
        function self = Dobot(baseTr)
            self.GetDobotRobot(baseTr);
            self.PlotAndColourRobot();
        end
        
         %% Set up Dobot Properties
        function GetDobotRobot(self, baseTr)
            % Give robot a unique name
            pause(0.001);
            name = ['Dobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            % DH parameters of Dobot Robot
            L1 = Link('d', 0.139, 'a', 0, 'alpha', deg2rad(-90),'offset', deg2rad(0), 'qlim', [deg2rad(-90) deg2rad(90)]);
            L2 = Link('d', 0, 'a', 0.135, 'alpha', deg2rad(0),'offset', deg2rad(-90), 'qlim', [deg2rad(0) deg2rad(85)]);
            L3 = Link('d', 0, 'a', 0.147, 'alpha', deg2rad(0),'offset', deg2rad(0), 'qlim', [deg2rad(-10) deg2rad(95)]);
            L4 = Link('d', 0, 'a', -0.061, 'alpha', deg2rad(90),'offset', deg2rad(-90), 'qlim', [deg2rad(180) deg2rad(370)]);
            L5 = Link('d', 0.09191, 'a', 0, 'alpha', deg2rad(0),'offset', deg2rad(180), 'qlim', [deg2rad(-90) deg2rad(90)]); % Include the gripper in the last joint
            
            % Create Dobot robotd
            self.model = SerialLink([L1 L2 L3 L4 L5],'name',name);
            self.model.base = baseTr;
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
            axis equal
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
                q(:,4) = 2*pi-joints(:,2)-joints(:,3);
            end
        end
        
        %% Pick up pencil function
        function PickPencil(self, pencil)
            % Move to pick up 
            goal = pencil.pencil.base
            qMatrix = self.PathToDesiredTransform(goal, self.qn);
            self.model.animate(qMatrix);
            
            % Lift pencil up out of holder
            goal = pencil.pencil.base*transl(0,0,0.07);
            qMatrix = self.PathToDesiredTransform(goal, self.qn);
            for i=1:size(qMatrix,1)
                self.model.animate(qMatrix(i,:));
                pencil.pencil.base = self.PencilPointingDown(self.model.fkine(qMatrix(i,:)));
                pencil.pencil.animate(0);
                drawnow;
            end
            
            % Move dobot to qn with pencil
            goal = self.model.fkine(self.qn);
            qMatrix = self.PathToDesiredTransform(goal, self.qn);
            for i=1:size(qMatrix,1)
                self.model.animate(qMatrix(i,:));
                pencil.pencil.base = self.PencilPointingDown(self.model.fkine(qMatrix(i,:)));
                pencil.pencil.animate(0);
                drawnow;
            end
        end
        
        %% Matrix of points from current position to goal position
        function qMatrixCurrentToGoal = PathToDesiredTransform(self,goalTransform, q0)
            % Number of steps between each transform
            steps = 100;
            qCurrent = self.model.getpos;
            
            goalTransform = self.MakeTransformVertical(goalTransform);
            [qGoal, err, exitflag] = self.model.ikcon(goalTransform, q0);

            % Using Trapezoidal Velocity Method
            % s is a value between 0,1 that provides the displacement at velcoities
            % & accelerations that makes full use of motor speeds, and doesn't waste
            % power accelerating and decelerating.
            s = lspb(0,1,steps);
            qMatrix5Joints = nan(steps,5);
            for i = 1:steps
                qMatrix5Joints(i,:) = (1-s(i))*qCurrent + s(i)*qGoal;
            end
            qMatrix5Joints(:,5) = 0;
            qMatrix5Joints(:,4) = [];
            qMatrixCurrentToGoal = self.CalcJointAngles(qMatrix5Joints);
        end
        
        %% Ensure the transform's orientation is correct for uncontrollable joint
        function tr = MakeTransformVertical(self,tr)
            tr(1,1) = 1;
            tr(1,2) = 0;
            tr(1,3) = 0;
            tr(2,1) = 0;
            tr(2,2) = -1;
            tr(2,3) = 0;
            tr(3,1) = 0;
            tr(3,2) = 0;
            tr(3,3) = -1;
        end
        
        %% Ensure Pencil is pointing down
        function tr = PencilPointingDown(self,tr)
            tr(1,1) = 1;
            tr(1,2) = 0;
            tr(1,3) = 0;
            tr(2,1) = 0;
            tr(2,2) = 1;
            tr(2,3) = 0;
            tr(3,1) = 0;
            tr(3,2) = 0;
            tr(3,3) = 1;
        end
        
        %% Visual Servoing over paper
        function CentrePencilOverPaper(environment)
            pTarget = [662, 362, 362, 662;
                       362, 362, 662, 662];
                   
            P = environment.paperCorners;
            
            q0 = self.model.getpos();
            
            % Add the camera
            self.cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');
            
            % frame rate
            fps = 25;
            
            %Define values
            %gain of the controler
            lambda = 0.6;
            %depth of the IBVS
            depth = mean (P(1,:));
            
            Tc0 = self.model.fkine(self.model.getpos());
            
            % plot camera and points
            self.cam.T = Tc0;
            
            % Display points in 3D and the camera
            cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);
            plot_sphere(P, 0.005, 'b')
            
            %Project points to the image
            p = cam.plot(P, 'Tcam', Tc0);
            
            %camera view and plotting
            cam.clf()
            cam.plot(pTarget, '*'); % create the camera view
            cam.hold(true);
            cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
            pause(2)
            cam.hold(true);
            cam.plot(P);    % show initial view
            
            %Initialise display arrays
            vel_p = [];
            uv_p = [];
            history = [];
            
            for ksteps=1:100
                % compute the view of the camera
                uv = cam.plot(P);
                
                % compute image plane error as a column
                e = pStar-uv;   % feature error
                e = e(:);
                Zest = [];
                
                % compute the Jacobian
                if isempty(depth)
                    % exact depth from simulation (not possible in practice)
                    pt = homtrans(inv(Tcam), P);
                    J = cam.visjac_p(uv, pt(3,:) );
                elseif ~isempty(Zest)
                    J = cam.visjac_p(uv, Zest);
                else
                    J = cam.visjac_p(uv, depth );
                end
                
                % compute the velocity of camera in camera frame
                try
                    v = lambda * pinv(J) * e;
                catch
                    status = -1;
                    return
                end
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
                
                %compute robot's Jacobian and inverse
                J2 = r.model.jacobn(q0);
                Jinv = pinv(J2);
                % get joint velocities
                qp = Jinv*v;
                
                %Maximum angular velocity cannot exceed 180 degrees/s
                ind=find(qp>pi);
                if ~isempty(ind)
                    qp(ind)=pi;
                end
                ind=find(qp<-pi);
                if ~isempty(ind)
                    qp(ind)=-pi;
                end
                
                %Update joints
                q = q0 + (1/fps)*qp;
                r.model.animate(q');
                
                %Get camera location
                Tc = r.model.fkine(q);
                cam.T = Tc;
                
                drawnow
                
                % update the history variables
                hist.uv = uv(:);
                vel = v;
                hist.vel = vel;
                hist.e = e;
                hist.en = norm(e);
                hist.jcond = cond(J);
                hist.Tcam = Tc;
                hist.vel_p = vel;
                hist.uv_p = uv;
                hist.qp = qp;
                hist.q = q;
                
                history = [history hist];
                
                pause(1/fps)
                %update current joint position
                q0 = q;
            end
        end
    end
end