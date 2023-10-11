classdef SlicerBot < handle
    %% Class for pizza cutting robot
    % uses visual servoing to detect when a pizza is in front of it and
    % cut it into 8 equal slices

    properties
    end

    methods
        % constructor
        function self = SlicerBot()
            %create a UR3 with a pizza slicer attachment
        end
        
        % use RGB-D camera to detect the pizza [BONUS]
        function DetectPizza(self)
        end

        % calculate cut based on pizza position
        function CalculateCut(self)
        end

        % step through slicing procedure
        function stepSlicing(self)
        end
    end
end