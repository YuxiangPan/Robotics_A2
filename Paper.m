classdef Paper < handle
    properties
        paper;
        paperCorners;
        workspace = [-0.6 0.6 -1 1 -0.65 0.7];
    end
    
    methods
        %% Constructor
        function self = Paper(paperBaseTr)
            self.GetPaper(paperBaseTr);
            self.PlotAndColourPaper();
            self.CornersOfPaper(paperBaseTr);
        end
        
       %% Setup Part Properties
        function GetPaper(self, paperBaseTr)
            % Give part a unique name
            namePaper = ['Pencil_',datestr(now,'yyyymmddTHHMMSSFFF')];
           
            % DH parameters of part
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            
            
            self.paper = SerialLink(L1,'name',namePaper);
            self.paper.base = paperBaseTr;
        end
        
        %% Plot and try to colour parts
        function PlotAndColourPaper(self)
            reloadData = 0; % 1 = reload data, 0 = use preloaded data
            switch reloadData
                case 0
                    load('environment/paperDataPreloaded.mat');
                case 1
                    [PaperFaceData, PaperVertexData, PaperPlyData] = plyread('environment/paper.ply','tri');
                otherwise
                    error('reloadData = 0 to use preloaded 3D data, or 1 reload 3D data')
            end

            % Setup part
            self.paper.faces = {PaperFaceData, []};
            self.paper.points = {PaperVertexData, []};
            
            plot3d(self.paper, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);

            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            
            % colour part
            handles = findobj('Tag', self.paper.name);
            h = get(handles,'UserData');
            try
                h.link(1).Children.FaceVertexCData = [PaperPlyData.vertex.red ...
                                                         ,PaperPlyData.vertex.green ...
                                                         ,PaperPlyData.vertex.blue]/255;
                h.link(1).Children.FaceColor = 'interp';
            catch ME_1
                disp(ME_1);
            end
        end
        
        %% Remove paper
        function RemovePaper(self)
            handles = findobj('Tag', self.paper.name);
            h_paper = get(handles, 'UserData');
            
            h_paper.link(1).Children.FaceColor = 'none';
            h_paper.link(1).Children.FaceVertexCData = [];
            h_paper.link(1).Children.CData = [];
            h_paper.link(1).Children.XData = h_paper.link(1).Children.XData * eps;
            h_paper.link(1).Children.YData = h_paper.link(1).Children.YData * eps;
            h_paper.link(1).Children.ZData = h_paper.link(1).Children.ZData * eps;
            
            animate(self.paper,0);           
        end
        
        %% Store Corners of Paper
        function CornersOfPaper(self, paperBaseTr)
            % Translate to find the corners from the centre of the paper
            % (this is to simulate a sensor reading the values)
            p1 = paperBaseTr*transl(0.05, 0.05, 0);
            p2 = paperBaseTr*transl(-0.05, 0.05, 0);
            p3 = paperBaseTr*transl(0.05, -0.05, 0);
            p4 = paperBaseTr*transl(-0.05, -0.05, 0);
            
            % Extract data from transforms to get X,Y,Z coordinates of corners
            self.paperCorners = [p1(1,4), p2(1,4), p3(1,4), p4(1,4);
                                  p1(2,4), p2(2,4), p3(2,4), p4(2,4);
                                  p1(3,4), p2(3,4), p3(3,4), p4(3,4)];
        end
    end
end

