classdef leftgripper < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'leftgripper';
    end
    
    properties(Constant)
        openL = deg2rad([0 -40]); %range of motion for the claw (through visual testing)
        closedL = deg2rad([0 -20]);
    end

    methods
%% Constructor
        function self = leftgripper(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
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
            link(1) = Link('d',0.01,'a',0,'alpha',pi/2,'qlim',deg2rad([0 360]), 'offset',0.1);
            link(2) = Link('d',0.01,'a',0,'alpha',-pi/2,'qlim',deg2rad([-36 0]), 'offset',0);
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
