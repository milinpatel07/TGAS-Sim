classdef HelperEstimateCurrentEgoState < matlab.System & matlab.system.mixin.Propagates
%HelperEstimateCurrentEgoState Estimates the current ego state from the
%   current ego position. This uses currentLane() MATLAB API. This system
%   object takes scenario object and ego actor id as inputs.
%
%   helperEstimateCurrentEgoState = HelperEstimateCurrentEgoState creates a
%   system object, helperEstimateCurrentEgoState, that estimates the
%   current ego state from the current ego position.
%
%   helperEstimateCurrentEgoState = HelperEstimateCurrentEgoState(Name,Value) 
%   creates a system object, helperEstimateCurrentEgoState, with additional 
%   options specified by one or more Name,Value pair arguments:
%
%   'Scenario'          -  Driving scenario Object
%                          Default: 1
%   'EgoActorID'        -  Ego Actor ID
%                          Default: 1
%
%   Step method syntax: [LateralPosition, LongitudinalPosition, NoLeftLane,
%   NoRightLane, DeviationOffset] = step(EgoActor, LateralDeviation,
%   RefPath) returns lateral position, LateralPosition, longitudinal
%   position, LongitudinalPosition, flag to check existence of left lane,
%   NoLeftLane, flag to check existence of right lane, NoRightLane, and
%   deviation offset from reference path, DeviationOffset based on ego
%   actor current position, EgoActor, current deviation from reference
%   path, LateralDeviation and reference path, RefPath.
%
%   NOTE: The name of this System Object and it's functionality may change
%   without notice in a future release, or the System Object itself may be
%   removed.

%   Copyright 2019-2020 The MathWorks, Inc.

    % Public, non tunable properties
    properties(Nontunable)
        %Lanewidth width of each lane in the scenario
        LaneWidth = [3.6, 3.6, 3.6];
        
        %EgoActorID Ego Actor ID
        EgoActorID = 1;
        
        %InitialDeviation Initial deviation of ego from reference path
        InitialDeviation = 0;
        
        %InitialEgoLaneNumber Initial lane number of ego vehicle
        InitialEgoLaneNumber = 1;
        
        %LaneType Type of the lane in the scenario. Supported lae types are
        %defined in LaneTypes enumeration
        LaneType = [LaneTypes.Driving, LaneTypes.Driving, LaneTypes.Driving];
    end

    % Pre-computed constants
    properties(Access = private)
        EgoVehicle;
        LaneInfo;
        Planner;
        CurrentEgoLaneNumber;
        DeviationOffset;
    end
    
    %----------------------------------------------------------------------
    % Main algorithm
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj)
            %setupImpl Perform one-time calculations and Initializations.            
            % The setupImpl method is used create trajectoryOptimalFrenet
            % object for the calculation of lateral and longitudinal
            % position of the vehicle with respect to reference path. This
            % method initializes ego vehicle lane number and it's deviation
            % from reference path.
            validator = validatorOccupancyMap;
            map = binaryOccupancyMap(50,50);
            validator.Map = map;
            obj.Planner = trajectoryOptimalFrenet([0,0; 0,600], validator);
            obj.CurrentEgoLaneNumber = obj.InitialEgoLaneNumber;
            obj.DeviationOffset = obj.InitialDeviation;
        end

        %------------------------------------------------------------------
        function [LateralPosition, LongitudinalPosition, NoLeftLane,...
                NoRightLane, DeviationOffset] = stepImpl(obj, EgoActor,...
                                                   LateralDeviation,...
                                                   RefPath)
            %stepImpl Implements the main logic to find required details of
            %the environment based on the current ego position.
            %   The stepImpl method implements the main logic to find the
            %   existent of neighboring lanes,  longitudinal and lateral
            %   position of the vehicle with respect to reference path and
            %   preferred deviation from reference path.
            
            % Get total number of lanes in the scenario.
            NumLanes = numel(obj.LaneWidth);
            
            % Compute current lane of ego vehicle based on lateral
            % deviation from reference path and previous lane number.            
            computeCurrentLane(obj, LateralDeviation);
            
            % Extract lane type from RoadSegments information
            laneType = obj.LaneType;
            
            % Calculate left and right lane numbers from the current ego
            % lane number 
            leftLaneNumber  = obj.CurrentEgoLaneNumber - 1;
            rightLaneNumber = obj.CurrentEgoLaneNumber + 1;
            
            NoLeftLane           = false;
            NoRightLane          = false;

            % Check if left and right lanes exist and update lane info
            if leftLaneNumber < 1               
                NoLeftLane = true;
            elseif ~isequal(laneType(leftLaneNumber), LaneTypes.Driving)
                NoLeftLane = true;
            end
            if rightLaneNumber > NumLanes
                NoRightLane = true;
            elseif ~isequal(laneType(rightLaneNumber), LaneTypes.Driving)
                NoRightLane = true;
            end
            if ~isequal(obj.Planner.Waypoints, RefPath)
                obj.Planner.Waypoints = RefPath;
            end
            
            % Get the current state of ego vehicle
            currentEgoStates = [EgoActor.Position(1), EgoActor.Position(2), ...
                    deg2rad(EgoActor.Yaw), 0, norm(EgoActor.Velocity),...
                    0];
            % Estimate the theta of the vehicle at the end of reference
            % path for the estimation of state of the vehicle at goal
            % position.
            thetaAtGoal = atan2((RefPath(end,2) - RefPath(end-1,2)),...
            (RefPath(end,1) - RefPath(end-1,1)));
        
            % Estimate the state of the vehicle at goal position
            goalStates = [RefPath(end,1), RefPath(end,2), ...
                         thetaAtGoal, 0, 0, 0];
            
            % Transform current and goal states of the vehicle to frenet
            % coordinate system
            frenetStatesCurrent = obj.Planner.cart2frenet(currentEgoStates(1:6));
            goalFrenetStates = obj.Planner.cart2frenet(goalStates(1:6));
            
            % Get lateral position of the vehicle
            LateralPosition      = frenetStatesCurrent(1,4);
            
            % Compute longitudinal position of the ego
            LongitudinalPosition = (goalFrenetStates(1,1) - frenetStatesCurrent(1,1));
            
            % Calculate Deviation from the reference path
            DeviationOffset = ((rem(NumLanes,2) == 0)*(obj.LaneWidth(obj.CurrentEgoLaneNumber)/2)...
                        + (floor((NumLanes+1)/2) - obj.CurrentEgoLaneNumber)*obj.LaneWidth(obj.CurrentEgoLaneNumber));
            if isempty(DeviationOffset)
                DeviationOffset = 0;
            end
            obj.DeviationOffset = DeviationOffset;
        end
        
        function computeCurrentLane(obj, lateralDeviation)
        %computeCurrentLane Computes the current ego lane based on the
        %deviation from reference path and previous lane.
        % computeCurrentLane method uses deviation from reference path and
        % deviation offset and lane width. The DeviationOffset is the value
        % that was fed to the planner. This will be updated during the lane
        % change. The lateralDeviation is the deviation of the vehicle at
        % each time step. This is updated by the motion planner.
            if obj.DeviationOffset == 0
                if lateralDeviation > obj.LaneWidth(obj.CurrentEgoLaneNumber)/2
                    obj.CurrentEgoLaneNumber = obj.CurrentEgoLaneNumber - 1;
                elseif lateralDeviation < -obj.LaneWidth(obj.CurrentEgoLaneNumber)/2
                    obj.CurrentEgoLaneNumber = obj.CurrentEgoLaneNumber + 1;
                end
            else
                if obj.DeviationOffset > lateralDeviation && ((obj.DeviationOffset - lateralDeviation) > obj.LaneWidth(obj.CurrentEgoLaneNumber)/2)
                    obj.CurrentEgoLaneNumber = obj.CurrentEgoLaneNumber + 1;
                elseif obj.DeviationOffset < lateralDeviation && ((lateralDeviation - obj.DeviationOffset) > obj.LaneWidth(obj.CurrentEgoLaneNumber)/2)
                    obj.CurrentEgoLaneNumber = obj.CurrentEgoLaneNumber - 1;
                end
            end
        end        
    end
    
    %----------------------------------------------------------------------
    % Simulink-only methods
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function [lateralPosition, longitudinalPosition,...
                 noLeftLane, noRightLane, deviation] = getOutputSizeImpl(obj)
            % Return size for each output port
            noLeftLane = 1;
            noRightLane = 1; 
            lateralPosition = 1;
            longitudinalPosition = 1;
            deviation = 1;
        end

        %------------------------------------------------------------------
        function [lateralPosition, longitudinalPosition,...
                 noLeftLane, noRightLane, deviation]...
                 = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            noLeftLane           = "boolean";
            noRightLane          = "boolean"; 
            lateralPosition      = "double";
            longitudinalPosition = "double";
            deviation            = "double";
        end

        %------------------------------------------------------------------
        function [lateralPosition, longitudinalPosition,...
                 noLeftLane, noRightLane,...
                 deviation] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            noLeftLane           = false;
            noRightLane          = false; 
            lateralPosition      = false;
            longitudinalPosition = false;
            deviation            = false;     
        end

        %------------------------------------------------------------------
        function [lateralPosition, longitudinalPosition,...
                 noLeftLane, noRightLane,... 
                 deviation] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            noLeftLane           = true;
            noRightLane          = true; 
            lateralPosition      = true;
            longitudinalPosition = true;
            deviation            = true;           
        end
    end
end
