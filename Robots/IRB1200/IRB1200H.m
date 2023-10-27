classdef IRB1200H < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'IRB1200H';
    end

    methods
%% Constructor
        function self = IRB1200H(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0.2);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('alpha',pi/2,'a',0, 'd',0.178, 'offset',pi/2,'qlim',[deg2rad(-170), deg2rad(170)] );
            link(2) = Link('alpha',0,'a',0.45, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]); %a o.448
            link(3) = Link('alpha',-pi/2,'a',0.042, 'd',0, 'offset',-pi/2,'qlim',[deg2rad(-125), deg2rad(155)]);
            link(4) = Link('alpha',pi/2,'a',0, 'd',0.451, 'offset',0,'qlim',[deg2rad(-270), deg2rad(270)]);
            link(5) = Link('alpha',-pi/2,'a',0, 'd',0, 'offset',0,'qlim',[deg2rad(-120), deg2rad(120)]);
            link(6) = Link('alpha',0,'a',0, 'd',0.08, 'offset',0,'qlim',[deg2rad(-360), deg2rad(360)]);
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
