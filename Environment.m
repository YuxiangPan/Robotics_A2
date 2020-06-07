classdef Environment < handle
    properties
        table;
        frame;
        pencilHolder;
        paper;
        eStop;
        
        workspace = [-0.6 0.6 -1 1 -0.65 0.7];  
    end
    
    methods
        %% Constructor
        function self = Environment(pencilBase, perspexOn)
            self.GetEnvironment(pencilBase);
            self.PlotAndColourEnvironment(perspexOn);
        end
        
        %% Setup Environment Properties
        function GetEnvironment(self, pencilBase)
            pause(0.001);
            nameTable = ['Table', datestr(now,'yyyymmddTHHMMSSFFF')];
            nameFrame = ['Frame', datestr(now,'yyyymmddTHHMMSSFFF')];
            namePencilHolder = ['pencilHolder', datestr(now,'yyyymmddTHHMMSSFFF')];
            %namePaper = ['Paper', datestr(now,'yyyymmddTHHMMSSFFF')];
            nameEStop = ['E-Stop', datestr(now,'yyyymmddTHHMMSSFFF')];
            
            % DH parameters of Environment
            L = Link('alpha',0,'a',0.1,'d',0,'offset',0);
            self.table = SerialLink(L, 'name', nameTable);
            self.frame = SerialLink(L, 'name', nameFrame);
            self.pencilHolder = SerialLink(L, 'name', namePencilHolder);
            %self.paper = SerialLink(L, 'name', namePaper);
            self.eStop = SerialLink(L, 'name', nameEStop);
            
            % Set base location of Environment components
            %self.paper.base = paperBase;%transl(0.2875,0,0);
            self.eStop.base = transl(0.3,0,0.65);
            self.table.base = rpy2tr(0,0,pi/2);
            pencilHolderBase = pencilBase;
            pencilHolderBase(3,4) = 0;
            self.pencilHolder.base = pencilHolderBase;
        end
        
        %% Plot and try to colour the Environment
        function PlotAndColourEnvironment(self, perspexOn)
            reloadData = 0; % 1 = reload data, 0 = use preloaded data
            switch reloadData
                case 0
                    if perspexOn == true
                        load('environment/environmentWithPerspexDataPreloaded.mat');
                    else
                        load('environment/environmentWithoutPerspexDataPreloaded.mat');
                    end
                    
                case 1
                    [tableFaceData, tableVertexData, tablePlyData] = plyread('environment/table.ply','tri');
                    [pencilHolderFaceData, pencilHolderVertexData, pencilHolderPlyData] = plyread('environment/pencilHolder.ply','tri');
                    [eStopFaceData, eStopVertexData, eStopPlyData] = plyread('environment/eStop.ply','tri');
                    if perspexOn == true
                        [frameFaceData, frameVertexData, framePlyData] = plyread('environment/frameWithPerspex.ply','tri');
                    else
                        [frameFaceData, frameVertexData, framePlyData] = plyread('environment/frameWithoutPerspex.ply','tri');
                    end
                    
                otherwise
                    error('reloadData = 0 to use preloaded 3D data, or 1 reload 3D data')
            end
            
            % set up 3d properties
            self.table.faces = {tableFaceData, []};
            self.table.points = {tableVertexData, []};
            
            self.frame.faces = {frameFaceData, []};
            self.frame.points = {frameVertexData, []};
            
            self.pencilHolder.faces = {pencilHolderFaceData, []};
            self.pencilHolder.points = {pencilHolderVertexData, []};
            
            self.eStop.faces = {eStopFaceData, []};
            self.eStop.points = {eStopVertexData, []};
            
            % Plot Environment
            plot3d(self.table, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            plot3d(self.frame, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            plot3d(self.pencilHolder, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            plot3d(self.eStop, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            axis equal;
            
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight;
            end
            
            % colour environment
            handles = findobj('Tag', self.table.name);
            h_table = get(handles, 'UserData');
            
            handles = findobj('Tag', self.frame.name);
            h_frame = get(handles, 'UserData');
            
            handles = findobj('Tag', self.pencilHolder.name);
            h_pencilHolder = get(handles, 'UserData');
            
            handles = findobj('Tag', self.eStop.name);
            h_eStop = get(handles, 'UserData');
            try
                h_table.link(1).Children.FaceVertexCData = [tablePlyData.vertex.red ...
                    ,tablePlyData.vertex.green ...
                    ,tablePlyData.vertex.blue]/255;
                h_table.link(1).Children.FaceColor = 'interp';
                
                h_frame.link(1).Children.FaceVertexCData = [framePlyData.vertex.red ...
                    ,framePlyData.vertex.green ...
                    ,framePlyData.vertex.blue]/255;
                h_frame.link(1).Children.FaceColor = 'interp';
                
                h_pencilHolder.link(1).Children.FaceVertexCData = [pencilHolderPlyData.vertex.red ...
                    ,pencilHolderPlyData.vertex.green ...
                    ,pencilHolderPlyData.vertex.blue]/255;
                h_pencilHolder.link(1).Children.FaceColor = 'interp';
                
                h_eStop.link(1).Children.FaceVertexCData = [eStopPlyData.vertex.red ...
                    ,eStopPlyData.vertex.green ...
                    ,eStopPlyData.vertex.blue]/255;
                h_eStop.link(1).Children.FaceColor = 'interp';
            end
        end
    end
end

