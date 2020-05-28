classdef Pencil < handle
    properties
        pencil;
        pencilCorners;
        
        workspace = [-0.6 0.6 -1 1 -0.65 0.7];
    end
    
    methods
        %% Constructor
        function self = Pencil(pencilBaseTr)
            self.GetPencil(pencilBaseTr);
            self.PlotAndColourPencil();
            self.CornersOfPencil(pencilBaseTr);
        end
        
        %% Setup Part Properties
        function GetPencil(self, pencilBaseTr)
            % Give part a unique name
            namePencil = ['Pencil_',datestr(now,'yyyymmddTHHMMSSFFF')];
           
            % DH parameters of part
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            
            
            self.pencil = SerialLink(L1,'name',namePencil);
            self.pencil.base = pencilBaseTr;
        end
        
        %% Plot and try to colour parts
        function PlotAndColourPencil(self)
            reloadData = 1;
            switch reloadData
                case 0
                    load('environment/PencilDataPreloaded.mat');
                case 1
                    [PencilFaceData, PencilVertexData, PencilPlyData] = plyread('environment/pencil.ply','tri');
                otherwise
                    error('reloadData = 0 to use preloaded 3D data, or 1 reload 3D data')
            end

            % setup part
            self.pencil.faces = {PencilFaceData, []};
            self.pencil.points = {PencilVertexData, []};
            
            plot3d(self.pencil, 0,'workspace',self.workspace,'view',[-30,30],'delay',0);

            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            
            % colour part
            handles = findobj('Tag', self.pencil.name);
            h = get(handles,'UserData');
            try
                h.link(1).Children.FaceVertexCData = [PencilPlyData.vertex.red ...
                                                         ,PencilPlyData.vertex.green ...
                                                         ,PencilPlyData.vertex.blue]/255;
                h.link(1).Children.FaceColor = 'interp';
            catch ME_1
                disp(ME_1);
            end
        end
        
        %% Store Corners of Pencil
        function CornersOfPencil(self, pencilBaseTr)
            xBase = pencilBaseTr(1,4);
            yBase = pencilBaseTr(2,4);
            zBase = pencilBaseTr(3,4);
            
            angle = acos(pencilBaseTr(1,1));
            x1Base = pencilBaseTr()*tranls(0,0,
            pencilCorners = [X+0.045, X-0.045, X-0.045 X-0.045;
                ]
            
            P=[1.8,1.8,1.8,1.8;
                -0.25,0.25,0.25,-0.25;
                1.25,1.25,0.75,0.75];
        end
    end
end

