classdef Environment < handle
    properties
        table;
        frame;
        penHolder;
        paper;
        eStop;
        
        workspace = [-0.6 0.6 -0.6 0.6 -0.3 0.6];  
    end
    
    methods
        %% Constructor
        function self = Environment(paperBase, perspexOn)
            self.GetEnvironment(paperBase);
            self.PlotAndColourEnvironment(perspexOn);
        end
        
        %% Setup Environment Properties
        function GetEnvironment(self, paperBase)
            pause(0.001);
            nameTable = ['Table', datestr(now,'yyyymmddTHHMMSSFFF')];
            nameFrame = ['Frame', datestr(now,'yyyymmddTHHMMSSFFF')];
%             namePen = ['Pen', datestr(now,'yyyymmddTHHMMSSFFF')];
            namePaper = ['Paper', datestr(now,'yyyymmddTHHMMSSFFF')];
            nameEStop = ['E-Stop', datestr(now,'yyyymmddTHHMMSSFFF')];
            
            % DH parameters of Environment
            L = Link('alpha',0,'a',0.1,'d',0,'offset',0);
            self.table = SerialLink(L, 'name', nameTable);
            self.frame = SerialLink(L, 'name', nameFrame);
%             self.pen = SerialLink(L, 'name', namePen);
            self.paper = SerialLink(L, 'name', namePaper);
            self.eStop = SerialLink(L, 'name', nameEStop);
            
            % Set base location of Environment components
            self.paper.base = paperBase%transl(0.2875,0,0);
            self.eStop.base = transl(0.3,0,0.65);
            self.table.base = rpy2tr(0,0,pi/2);
        end
        
        %% Plot and try to colour the Environment
        function PlotAndColourEnvironment(self, perspexOn)
            reloadData = 1; % 1 = reload data, 0 = use preloaded data
            switch reloadData
                case 0
                    if perspexOn == true
                    else
                    end
                    %load('dobotLinks/EnvironmentDataPreloaded.mat');
                case 1
                    [tableFaceData, tableVertexData, tablePlyData] = plyread('environment/table.ply','tri');

%                     [penFaceData, penVertexData, penPlyData] = plyread('environment/pen.ply','tri');
                    [paperFaceData, paperVertexData, paperPlyData] = plyread('environment/paper.ply','tri');
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
%             
%             self.pen.faces = {penFaceData, []};
%             self.pen.points = {penVertexData, []};
%             
            self.paper.faces = {paperFaceData, []};
            self.paper.points = {paperVertexData, []};
            
            self.eStop.faces = {eStopFaceData, []};
            self.eStop.points = {eStopVertexData, []};
            
            % Plot Environment
            plot3d(self.table, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            plot3d(self.frame, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);
%             plot3d(self.pen, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            plot3d(self.paper, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            plot3d(self.eStop, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            axis equal;
            
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            
            % colour environment
            handles = findobj('Tag', self.table.name);
            h_table = get(handles, 'UserData');
            
            handles = findobj('Tag', self.frame.name);
            h_frame = get(handles, 'UserData');
            
%             handles = findobj('Tag', self.pen.name);
%             h_pen = get(handles, 'UserData');
%             
            handles = findobj('Tag', self.paper.name);
            h_paper = get(handles, 'UserData');
            
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
%                 
%                 h_pen.link(1).Children.FaceVertexCData = [penPlyData.vertex.red ...
%                     ,penPlyData.vertex.green ...
%                     ,penPlyData.vertex.blue]/255;
%                 h_pen.link(1).Children.FaceColor = 'interp';
%                 
                h_paper.link(1).Children.FaceVertexCData = [paperPlyData.vertex.red ...
                    ,paperPlyData.vertex.green ...
                    ,paperPlyData.vertex.blue]/255;
                h_paper.link(1).Children.FaceColor = 'interp';
                
                h_eStop.link(1).Children.FaceVertexCData = [eStopPlyData.vertex.red ...
                    ,eStopPlyData.vertex.green ...
                    ,eStopPlyData.vertex.blue]/255;
                h_eStop.link(1).Children.FaceColor = 'interp';
            end
        end
    end
end

