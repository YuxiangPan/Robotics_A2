classdef Pencil < handle
    properties
        pencil;
        
        workspace = [-0.6 0.6 -1 1 -0.65 0.7];
    end
    
    methods
        %% Constructor
        function self = Pencil(pencilBaseTr)
            self.GetPencil(pencilBaseTr);
            self.PlotAndColourPencil();
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
            reloadData = 0; % 1 = reload data, 0 = use preloaded data
            switch reloadData
                case 0
                    load('environment/pencilDataPreloaded.mat');
                case 1
                    [PencilFaceData, PencilVertexData, PencilPlyData] = plyread('environment/pencil.ply','tri');
                otherwise
                    error('reloadData = 0 to use preloaded 3D data, or 1 reload 3D data')
            end

            % Setup part
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

    end
end

