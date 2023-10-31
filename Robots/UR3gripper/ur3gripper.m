classdef ur3gripper < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        gripperL;
        gripperR;
    end

    methods
        function self = ur3gripper()
            self.gripperL = leftgripper;
            self.gripperR = rightgripper;
            self.open();
        end

        % open gripper
        function open(self)
            openL = deg2rad([0 -40]); 
            openR = deg2rad([0 40]); 
            self.gripperL.model.animate(openL);
            self.gripperR.model.animate(openR);
        end
        
        % close cripper
        function closed(self)
            closedL = deg2rad([0 -20]);
            closedR = deg2rad([0 20]);
            self.gripperL.model.animate(closedL);
            self.gripperR.model.animate(closedR);
        end

        % update gripper base
        function base(self,ee,gripperstate)
            self.gripperL.model.base = ee;
            self.gripperR.model.base = ee;
            
            switch gripperstate
                case 'closed'
                    self.closed()
                case 'open'
                    self.open()
                otherwise
                    return
            end
        end
    end
end