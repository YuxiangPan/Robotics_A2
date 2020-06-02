classdef Dobot < handle
    %Dobot: Creates object for Dobot robot
    properties
        % Dobot model
        model;
        
        % Camera
        cam
        
        % Steps between each movement
        steps = 30;
        
        % Shape corners
        shapeCorners = [];
        
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
            reloadData = 1; % 1 = reload data, 0 = use preloaded data
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
            goal = pencil.pencil.base;
            qMatrix = self.PathToDesiredTransform(goal, self.qn);
            self.model.animate(qMatrix);
            
            % Lift pencil up out of holder
            goal = pencil.pencil.base*transl(0,0,0.07);
            qMatrix = self.PathToDesiredTransform(goal, self.qn);
            self.AnimateDobotAndPencil(qMatrix, pencil)
            
            % Move dobot to qn with pencil
            goal = self.model.fkine(self.qn);
            qMatrix = self.PathToDesiredTransform(goal, self.qn);
            self.AnimateDobotAndPencil(qMatrix, pencil);

        end
        
        %% Matrix of points from current position to goal position
        function qMatrixCurrentToGoal = PathToDesiredTransform(self,goalTransform, q0)
            qCurrent = self.model.getpos;
            
            goalTransform = self.MakeTransformVertical(goalTransform);
            [qGoal, err, exitflag] = self.model.ikcon(goalTransform, q0);

            % Using Trapezoidal Velocity Method
            % s is a value between 0,1 that provides the displacement at velcoities
            % & accelerations that makes full use of motor speeds, and doesn't waste
            % power accelerating and decelerating.
            s = lspb(0,1,self.steps);
            qMatrix5Joints = nan(self.steps,5);
            for i = 1:self.steps
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
        
        %% Visual Servoing over paper, based off tutorial 8
        function CentrePencilOverPaper(self, environment, pencil)
            % Do you want to plot the cameras view?
            showCamView = true; % true = plot, false = don't plot
            
            % Set the target pose of the paper on the camera
            pTarget = [910, 410, 910, 410
                       262, 262, 762, 762];

            % retreive the corner points of the peice of paper
            P = environment.paperCorners;
            %plot_sphere(P, 0.01, 'b');
            
            % Det the joint angles of the current position of Dobo
            q0 = self.model.getpos()';
            % Delete the uncontrollable joint angle
            q0(4) = [];
            % Add the camera
            self.cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                'resolution', [1024 1024], 'centre', [512 512],'name', 'RunCam Eagle');
            
            % frame rate
            fps = 60;
            
            % Define values
            %   gain of the controler
            lambda = 1.5;
            %   depth of the IBVS (height of end effector off table (z=0))
            Tc0 = self.model.fkine(self.model.getpos());
            depth = Tc0(3,4);
            
            Tc0 = self.CameraLocationTr(self.model.fkine(self.model.getpos()));
            % Plot camera and points
            self.cam.T = Tc0;
            % Display points in 3D and the camera
            %self.cam.plot_camera('Tcam',Tc0, 'label','scale',0.01);
            % Project points to the image
            %p = self.cam.plot(P, 'Tcam', Tc0);
                       
            % Camera view and plotting
            if showCamView == true
                self.cam.clf()
                self.cam.plot(pTarget, '*'); % create the camera view
                self.cam.hold(true);
                self.cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
                pause(2)
                self.cam.hold(true);
                self.cam.plot(P);    % show initial view
            end
            % Move robot until error is small
            %   Set a high error initially
            e =[-100, -100, -100, -100, -100, -100, -100, -100]';
            while mean(abs(e)) > 5
                % Compute the view of the camera
                uv = self.cam.project(P);
                if showCamView == true
                    self.cam.plot(P);
                end
                
                % Compute image plane error as a column
                e = pTarget-uv;   % feature error
                e = e(:);
                
                % compute the Jacobian
                J = self.cam.visjac_p(uv, depth);
                
                % Compute the velocity of camera in camera frame
                try
                    v = lambda * pinv(J) * e;
                catch
                    status = -1;
                    return
                end
                
                % Compute robot's Jacobian and inverse
                %   Calculate uncontrollable joint
                q0 = self.CalcJointAngles(q0');
                q0 = q0';
                J2 = self.model.jacobn(q0);
                Jinv = pinv(J2);
                
                % Get joint velocities
                qp = Jinv*v;
                %   Ensure the correct joint velocity of uncontrollable joint
                qp = self.CalcJointVelocity(qp);
                
                %   Ensure the maximum angular velocity does not exceed 320 degrees/s
                ind=find(qp>pi*16/9);
                if ~isempty(ind)
                    qp(ind)=pi*16/9;
                end
                ind=find(qp<-pi*16/9);
                if ~isempty(ind)
                    qp(ind)=-pi*16/9;
                end
                
                % Update joints
                q = q0 + (1/fps)*qp;
                self.AnimateDobotAndPencil(q',pencil);
                
                % Get camera location
                Tc = self.model.fkine(q);
                self.cam.T = self.CameraLocationTr(Tc);
                
                drawnow;
                
                pause(1/fps);
                %update current joint position
                q0 = q;
                q0(4) = []; 
            end
        end
        
        %% Animate moving the dobot and the pencil at the same time. 
        function AnimateDobotAndPencil(self, qMatrix, pencil)
            for i=1:size(qMatrix,1)
                self.model.animate(qMatrix(i,:));
                pencil.pencil.base = self.PencilPointingDown(self.model.fkine(qMatrix(i,:)));
                pencil.pencil.animate(0);
                drawnow;
            end
        end
        
        %% Calculate joint 4 velocity
        function v = CalcJointVelocity(self, v)
            % Pass in joint velocity to this function to ensure the
            % uncontrollable joint has the correct velocity.
            if size(v,1) ~= 5
                error('Pass in all 5 joint velocities');
            else
                % Calculate the angles of the uncontrollable joint
                v(4,:) = -v(2,:)-v(3,:);
            end
        end
        
        %% Translate Camera
        function cameraTr = CameraLocationTr(self, endEffector)
            cameraTr = endEffector*transl(-0.028,0,-0.045);%*rpy2tr(0,0,-pi/2);
        end
        
        %% Plan points to draw shape
        function points = planCornersOfShape(self, shape, scale)
            centre = self.model.fkine(self.model.getpos());
            centre(3,4) = 0.005;
            distanceFromCentre = scale*0.02;
            switch shape
                case 'Square'
                    p1 = centre*transl(distanceFromCentre, distanceFromCentre, 0);
                    p2 = centre*transl(-distanceFromCentre, distanceFromCentre, 0);
                    p3 = centre*transl(distanceFromCentre, -distanceFromCentre, 0);
                    p4 = centre*transl(-distanceFromCentre, -distanceFromCentre, 0);
                    
                    % Extract data from transforms to get X,Y,Z coordinates of corners
                    self.shapeCorners = [p1(1,4), p2(1,4), p3(1,4), p4(1,4);
                                        p1(2,4), p2(2,4), p3(2,4), p4(2,4);
                                        p1(3,4), p2(3,4), p3(3,4), p4(3,4)];
                                    plot_sphere(self.shapeCorners, 0.01, 'b');
                    
                case 'Triangle'
                    p1 = centre*transl(distanceFromCentre, 0, 0);
                    p2 = centre*transl(-distanceFromCentre*sin(pi/6), distanceFromCentre*cos(pi/6), 0);
                    p3 = centre*transl(-distanceFromCentre*sin(pi/6), -distanceFromCentre*cos(pi/6), 0);
                    
                    % Extract data from transforms to get X,Y,Z coordinates of corners
                    self.shapeCorners = [p1(1,4), p2(1,4), p3(1,4);
                                        p1(2,4), p2(2,4), p3(2,4);
                                        p1(3,4), p2(3,4), p3(3,4)];
                                    plot_sphere(self.shapeCorners, 0.01, 'b');
            end 
        end
        
        %% Resolved Motion Rate Control
        % For this project the yaw of the end effector doesn't matter for
        % RMRC        
        function StraightMovementToNewTransform(self, goalTr)
            % get current joint angles
            q0 = self.model.getpos();
            % get current end effector transform
            startTr = self.model.fkine(q0);
            % get coordinates of start and goal poitns
            xStart = startTr(1:3,4)';
            %xStart(4) = 0;
            xGoal = goalTr(1:3,4)';
            deltaT = 0.05; %descrete timestep
            
            % Matrix of way points
            x = zeros(3,self.steps);
            s = lspb(0,1,self.steps);
            for i = 1:self.steps
                x(:,i) = xStart*(1-s(i)) + s(i)*xGoal;
            end
            
            % create matrix of joint angles
            qMatrix = nan(self.steps,3);
            qMatrix(1,:) = q0(1:3);
            
            for i = 1:self.steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
                q = [qMatrix(i,:), 0]
                q = self.CalcJointAngles(q)
                J = self.model.jacob0(q);            % Get the Jacobian at the current state
                J = J(1:3,1:3);                           % Take only first 3 rows for 3 joints
                m(:,i)= sqrt(det(J*J'));                                                % Measure of Manipulability
                if m(:,i) < 0.1
                    qdot = inv(J'*J + 0.01*eye(3))*J'*xdot;
                else
                    qdot = inv(J) * xdot;
                end
                %qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
                
                qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
            end
            qMatrix(:,4) = 0;
            qMatrix = self.CalcJointAngles(qMatrix);
            points = [];
            for i = 1:size(qMatrix,1)
                self.model.animate(qMatrix(i,:));
                endTr = self.model.fkine(qMatrix(i,:))
                point = endTr(1:3,4);
                points(:,i) = point;
                %plot3(point');
            end
            plot_sphere(points, 0.01, 'b');
        end
        
        %% Plot the Ellipsoids around each joint
        function CollisionMode(self)
            % Beginning of sweep
            q0 = deg2rad([-90,85,5,270,0]);
            self.model.animate(q0);
            warning off
            
            % Link 0 ellipsoid
            centerPoint = [0,0,0.027];
            radii = [0.110,0.110,0.040];
            [X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            self.model.points{1} = [X(:),Y(:),Z(:)];
            self.model.faces{1} = delaunay(self.model.points{1});
            centerPoints{1} = centerPoint;
            radiiValues{1} = radii;
            
            % Link 1 ellipsoid
            centerPoint = [0,0.04,0];
            radii = [0.070,0.090,0.070];
            [X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            self.model.points{2} = [X(:),Y(:),Z(:)];
            self.model.faces{2} = delaunay(self.model.points{2});
            centerPoints{2} = centerPoint;
            radiiValues{2} = radii;
            
            % Link 2 ellipsoid
            centerPoint = [-0.06,-0.03,0];
            radii = [0.090,0.050,0.040];
            [X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            self.model.points{3} = [X(:),Y(:),Z(:)];
            self.model.faces{3} = delaunay(self.model.points{3});
            centerPoints{3} = centerPoint;
            radiiValues{3} = radii;
            
            % Link 3 ellipsoid
            centerPoint = [-0.08,-0.03,0];
            radii = [0.095,0.045,0.035];
            [X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            self.model.points{4} = [X(:),Y(:),Z(:)];
            self.model.faces{4} = delaunay(self.model.points{4});
            centerPoints{4} = centerPoint;
            radiiValues{4} = radii;
            
            % Link 4 ellipsoid
            centerPoint = [0.04,0,-0.015];
            radii = [0.04,0.02,0.035];
            [X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            self.model.points{5} = [X(:),Y(:),Z(:)];
            self.model.faces{5} = delaunay(self.model.points{5});
            centerPoints{5} = centerPoint;
            radiiValues{5} = radii;
            
            % Link 5 ellipsoid
            centerPoint = [0,0,-0.090];
            radii = [0.040,0.030,0.095];
            [X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            self.model.points{6} = [X(:),Y(:),Z(:)];
            self.model.faces{6} = delaunay(self.model.points{6});
            centerPoints{6} = centerPoint;
            radiiValues{6} = radii;
            
            warning on
            self.model.plot3d(q0);
            axis equal
            camlight;
            
            % Define Cube Points
            cubeSides = 0.1;
            [X,Y] = meshgrid(-cubeSides:0.01:cubeSides,-cubeSides:0.01:cubeSides);
            Z = repmat(cubeSides,size(X,1),size(X,2));
            cubePoints = [X(:),Y(:),Z(:)];
            cubePoints = [ cubePoints; ...
                           cubePoints * rotx(pi/2); ...
                           cubePoints * rotx(pi); ...
                           cubePoints * rotx(3*pi/2); ...
                           cubePoints * roty(pi/2); ...
                           cubePoints * roty(-pi/2)];
            centreOfCube = [0.25,0,0.1];
            cubePoints = cubePoints + repmat(centreOfCube,size(cubePoints,1),1);
            hold on
%             cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
            hold off
            axis equal
            % Plot cube body (note how plotting the points is commented out above)
            plotOptions.plotFaces = true;
            rectangularPrismCorners1 = [centreOfCube(1)-cubeSides,centreOfCube(2)-cubeSides,centreOfCube(3)-cubeSides];
            rectangularPrismCorners2 = [centreOfCube(1)+cubeSides,centreOfCube(2)+cubeSides,centreOfCube(3)+cubeSides];
            % Use provided Rectangular prism function from tutorials
            RectangularPrism(rectangularPrismCorners1,rectangularPrismCorners2,plotOptions);
            
            % Define end of sweep and calculate qMatrix to get there
            qEnd = deg2rad([90,85,5,270,0]);
            endOfSweepTr = self.model.fkine(qEnd);
            qMatrix = self.PathToDesiredTransform(endOfSweepTr,q0);
            
            % Transform for each link
            tr = zeros(4,4,self.model.n+1);
            tr(:,:,1) = self.model.base;
            % store DH parameters to calculate the transform at each link
            L = self.model.links;

            % This collision checking technique is similar to that provided
            % in tutorial 6
            collisionDetected = false;
            % Loop through each joint on the qMatrix
            for i = 1:size(qMatrix,1)
                % Find transform at each link
                for j = 1:self.model.n
                    tr(:,:,j+1) = tr(:,:,j) * trotz(qMatrix(i,j)+L(j).offset) * transl(0,0,L(j).d) * transl(L(j).a,0,0) * trotx(L(j).alpha);
                end
                % Determine if there are any intersections at each link
                for j = 1:size(tr,3)
                    cubePointsAndOnes = [inv(tr(:,:,j)) * [cubePoints,ones(size(cubePoints,1),1)]']';
                    updatedCubePoints = cubePointsAndOnes(:,1:3);
                    algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoints{j}, radiiValues{j});
                    pointsInside = find(algebraicDist < 1);
                    if pointsInside >= 1
                        collisionDetected = true;
                    end
                end
                if collisionDetected == true;
                    % store the joint angles before the collision
                    qBeforeCollision = qMatrix(i-1,:);
                    break;
                end
            end
            % recalculate a qMatrix that doesn't collide with block
            trBeforeCollision = self.model.fkine(qBeforeCollision);
            qMatrix = self.PathToDesiredTransform(trBeforeCollision, q0);
            self.model.animate(qMatrix);
        end
 
    end
end