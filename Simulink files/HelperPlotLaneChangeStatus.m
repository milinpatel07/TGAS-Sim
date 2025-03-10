classdef HelperPlotLaneChangeStatus < matlab.System
    %HelperPlotLaneChangeStatus Helps in visualizing the execution of trajectory
    %   using MATLAB figure and birdsEyePlot
    %     This System object helps in visualizing
    %     1. Lane boundaries for straight and curved road scenarios
    %     2. Trajectory of the ego vehicle.
    %     3. Track of the ego vehicle.
    %     4. Other target vehicles in the scenario
    %     5. Detected MIOs
    %
    %   NOTE: The name of this System Object and it's functionality may change
    %   without notice in a future release, or the System Object itself may be
    %   removed.
    %
    
    %   Copyright 2019-2020 The MathWorks, Inc.
    
    % Private properties of the System object
    properties(Access = private)
        % Figure holds the instance of MATLAB figure and their properties.
        Figure
        
        % BirdsEyePlot holds the instance of birdsEyePlot().
        BirdsEyePlot
        
        % OutlinePlotter holds the instance of outlinePlotter().
        OutlinePlotter
        
        % OutlinePlotter to plot mio positions
        MioOutlinePlotter
        
        % LaneMarkingPlotter holds the instance of LaneMarkingPlotter().
        LaneMarkingPlotter
        
        % EgoTrace and EgoTrack a used to plot the track of the ego vehicle
        EgoTrace
        EgoTrack
        
        % EgoPath is used to plot the ego vehicle trajectory.
        EgoPath
        
        % Candidate trajectories
        CandTraj1
        CandTraj2
        CandTraj3
    end

    properties(Nontunable, Logical)
        % Enable visualization window
        EnableVisualization = true;
    end

    % Constants used in the System object.
    properties(Constant, Hidden)
        % Length of the car
        CarLength = 4.7;
        
        % Width of the car
        CarWidth = 1.8;
        
        % OriginOffset defines the distance between rear axle and the rear
        % end of the vehicle
        OriginOffset = [-1.35 0];
        
        % Color of the ego car
        ColorBlue  = [0 0.447 0.741];
        
        % Max number of points in each trajectory
        MaxTrajPoints = 1000;
    end
    
    %----------------------------------------------------------------------
    % Main algorithm
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj)
            %setupImpl Initializes the properties of System object.
            % The setupImpl method initializes the Figure, BirdsEyePlot,
            % OutlinePlotter, LaneMarkingPlotter, EgoTrace, EgoTrack and
            % EgoPath.
            
            % Create and set Figure Properties
            figureName = 'Lane Change Status Plot';
            obj.Figure = findobj('Type','Figure','Name',figureName);
            if isempty(obj.Figure)
                screenSize = double(get(groot,'ScreenSize'));
                obj.Figure = figure('Name',figureName);
                obj.Figure.Position = [3 35 screenSize(3)/4 screenSize(4)*0.93];
                obj.Figure.NumberTitle = 'off';
                obj.Figure.MenuBar = 'none';
                obj.Figure.ToolBar = 'none';
            end
            % Clear figure
            clf(obj.Figure);
            hax = axes(obj.Figure);
            
            % Create birds eye plot
            obj.BirdsEyePlot = birdsEyePlot('Parent', hax,...
                'XLimits', [-60, 60],...
                'YLimits', [-20, 20]);
            
            % Create outline plotter for ego vehicle
            egoOutlinePlotter = outlinePlotter(obj.BirdsEyePlot);
            
            % Plot the ego car in visualization
            plotOutline(egoOutlinePlotter,...
                [0,0], 0, obj.CarLength, obj.CarWidth,...
                'OriginOffset',obj.OriginOffset,...
                'Color',obj.ColorBlue);
            
            % Create trajectory plotter properties
            obj.EgoPath = line(hax, 0, 0,...
                'Color','black',...
                'LineWidth',2,...
                'LineStyle','-');
            set(obj.EgoPath,'XData',[],'YData',[]);
            
            % Create candidate trajectories plotter properties
            obj.CandTraj1 = line(hax, 0, 0,...
                'Color','red',...
                'LineWidth',1,...
                'LineStyle','--');
            set(obj.CandTraj1,'XData',[],'YData',[]);
            obj.CandTraj2 = line(hax, 0, 0,...
                'Color','red',...
                'LineWidth',1,...
                'LineStyle','--');
            set(obj.CandTraj2,'XData',[],'YData',[]);
            obj.CandTraj3 = line(hax, 0, 0,...
                'Color','red',...
                'LineWidth',1,...
                'LineStyle','--');
            set(obj.CandTraj3,'XData',[],'YData',[]);
            
            % Create trace plotter properties
            obj.EgoTrace = line(hax, 0, 0,...
                'Color',obj.ColorBlue,...    % aqua
                'LineWidth',0.1,...
                'LineStyle','-',...
                'MarkerEdgeColor',obj.ColorBlue, ...
                'MarkerFaceColor','none', ...
                'Marker','o', ...
                'MarkerSize',3);
            
            set(obj.EgoTrace,'XData',[],'YData',[]);
            obj.EgoTrack.XData = [];
            obj.EgoTrack.YData = [];
            
            % Clubbing all legend handles
            linehandles = [obj.CandTraj2, ...
                obj.EgoPath, obj.EgoTrace];
            legend(linehandles, {'Candidate Trajectories',...
                'Optimal Trajectory', 'Trajectory Trace'}, ...
                'Location', 'northoutside');
            
            % Create lane marking plotter
            obj.LaneMarkingPlotter = ...
                laneMarkingPlotter(obj.BirdsEyePlot,...
                'DisplayName','Lane boundaries');
            
            % Create outline plotter for target vehicles
            obj.OutlinePlotter = outlinePlotter(obj.BirdsEyePlot);

            if ~obj.EnableVisualization
                % Disabling the visualization window
                set(obj.Figure,"Visible","off");
            else
                % Display warning message dialog when user tries to close
                % the window during simulation.
                 set(obj.Figure,'CloseRequestFcn',...
                    "warndlg({'Unable to close the Lane Change Status Plot during simulation. To disable Lane Change Status Plot, deselect ''Enable visualization window'''},'Warning');");
            end
        end
        
        %------------------------------------------------------------------
        function stepImpl(obj, TargetActors, EgoActor, VisualizationData)
            %stepImpl implements the core logic for visualization at every
            %simulation step
            %
            %   The stepImpl implements the logic to plot lanes, actors,
            %   trajectory, mios and the track of the ego vehicle on birds
            %   eye plot.
            
            % Segregating the planner data
            NumTrajPoints = VisualizationData.signal1;
            TrajectoryList = VisualizationData.signal2;
            trajectoryIdx = VisualizationData.signal4;
            
            % Taking out Optimal trajectory from list of trajectories using
            % trajectory index (trajectoryIdx)
            idx = (trajectoryIdx-1)*obj.MaxTrajPoints + 1;
            trajectoryPoints = TrajectoryList(idx:(idx + NumTrajPoints - 1),1:2);
            obj.EgoPath.XData = trajectoryPoints(:,1) - EgoActor.Position(1);
            obj.EgoPath.YData = trajectoryPoints(:,2) - EgoActor.Position(2);
            [obj.EgoPath.XData,obj.EgoPath.YData] = ...
                rotatePoints(obj.EgoPath.XData,obj.EgoPath.YData,...
                -EgoActor.Yaw);
            
            % Plotting first candidate trajectory if it's non-optimal
            trajectory1Points = TrajectoryList(1:NumTrajPoints,1:2);
            if (~isequal(trajectoryPoints, trajectory1Points))
                obj.CandTraj1.XData = trajectory1Points(:,1) - EgoActor.Position(1);
                obj.CandTraj1.YData = trajectory1Points(:,2) - EgoActor.Position(2);
                [obj.CandTraj1.XData,obj.CandTraj1.YData] = ...
                    rotatePoints(obj.CandTraj1.XData,obj.CandTraj1.YData,...
                    -EgoActor.Yaw);
            else
                set(obj.CandTraj1,'XData',[],'YData',[]);
            end
            
            % Plotting second candidate trajectory if it's non-optimal
            nextTrajIdx = obj.MaxTrajPoints;
            trajectory2Points = TrajectoryList(nextTrajIdx+1:nextTrajIdx...
                + NumTrajPoints,1:2);
            if (~isequal(trajectoryPoints, trajectory2Points))
                obj.CandTraj2.XData = trajectory2Points(:,1) - EgoActor.Position(1);
                obj.CandTraj2.YData = trajectory2Points(:,2) - EgoActor.Position(2);
                [obj.CandTraj2.XData,obj.CandTraj2.YData] = ...
                    rotatePoints(obj.CandTraj2.XData,obj.CandTraj2.YData,...
                    -EgoActor.Yaw);
            else
                set(obj.CandTraj2,'XData',[],'YData',[]);
            end
            
            % Plotting third candidate trajectory if it's non-optimal
            nextTrajIdx = obj.MaxTrajPoints*2;
            trajectory3Points = TrajectoryList(nextTrajIdx+1:nextTrajIdx...
                + NumTrajPoints,1:2);
            if (~isequal(trajectoryPoints, trajectory3Points))
                obj.CandTraj3.XData = trajectory3Points(:,1) - EgoActor.Position(1);
                obj.CandTraj3.YData = trajectory3Points(:,2) - EgoActor.Position(2);
                [obj.CandTraj3.XData,obj.CandTraj3.YData] = ...
                    rotatePoints(obj.CandTraj3.XData,obj.CandTraj3.YData,...
                    -EgoActor.Yaw);
            else
                set(obj.CandTraj3,'XData',[],'YData',[]);
            end
            
            % Plot lane boundaries on the birds eye plot.
            % Update road boundaries and their display
            egoCar = evalin('base', 'scenario.Actors(1)');
            
            % Update ego actor information
            egoCar.Position = EgoActor.Position;
            egoCar.Yaw = EgoActor.Yaw;
            egoCar.Roll = EgoActor.Roll;
            egoCar.Pitch = EgoActor.Pitch;
            % Set current velocity
            egoCar.Velocity = EgoActor.Velocity;
            egoCar.AngularVelocity = EgoActor.AngularVelocity;
            
            [lmv, lmf] = laneMarkingVertices(egoCar);
            plotLaneMarking(obj.LaneMarkingPlotter, lmv, lmf);
            
            % Save ego path in world coordinate
            obj.EgoTrack.XData = [obj.EgoTrack.XData EgoActor.Position(1)];
            obj.EgoTrack.YData = [obj.EgoTrack.YData EgoActor.Position(2)];
            
            % Calculate ego path w.r.t. current ego position
            EgoTraceX = obj.EgoTrack.XData - EgoActor.Position(1);
            EgoTraceY = obj.EgoTrack.YData - EgoActor.Position(2);
            [obj.EgoTrace.XData,obj.EgoTrace.YData] = ...
                rotatePoints(EgoTraceX,EgoTraceY,-EgoActor.Yaw);
            
            % Number of actors must remain the same between steps
            numActors = TargetActors.NumActors;
            numActors = numActors + 1; % add ego car
            
            positions = zeros(numActors,2);
            yaws      = zeros(numActors,1);
            lengths   = ones(numActors,1) * obj.CarLength;
            widths    = ones(numActors,1) * obj.CarWidth;
            originOffsets = ones(numActors,1) * obj.OriginOffset;
            
            % Add ego vehicle at origin
            positions(1,:) = [0,0];
            yaws(1) = 0;
            
            % Plot other target vehicles on birds eye plot.
            for n = 2:numActors
                positions(n,:) = TargetActors.Actors(n-1).Position(1:2);
                yaws(n,:)      = TargetActors.Actors(n-1).Yaw;
            end
            % Plot target vehicles on birds eye plot
            plotOutline(obj.OutlinePlotter,...
                positions, yaws, lengths, widths,...
                'OriginOffset',originOffsets);
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink dialog
    %----------------------------------------------------------------------
    methods(Access = protected, Static)
        %------------------------------------------------------------------
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(....
                "Title","HelperPlotLaneChangeStatus",...
                "Text",...
                "Helps in visualizing the execution of trajectory using MATLAB figure and birdsEyePlot" + newline + newline +...
                "Enable visualization window to show intermediate processing for lane change.");
        end
        
        %------------------------------------------------------------------
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
        
        %------------------------------------------------------------------
        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
        
        %------------------------------------------------------------------
        function releaseImpl(obj)
            if obj.EnableVisualization
                % Modifying the CloseRequestFcn callback to its original state
                set(obj.Figure,'CloseRequestFcn','closereq');
            else
                close(obj.Figure);
            end
        end
    end
    
end

%----------------------------------------------------------------------
% Utility functions
%----------------------------------------------------------------------
function [Xout,Yout] = rotatePoints(Xin,Yin,theta)
    %rotatePoints rotates the trajectory points with respect to ego vehicle
    Xout = Xin;
    Yout = Yin;
    R = [cosd(theta) -sind(theta); ...
        sind(theta)  cosd(theta)];
    for i = 1:length(Xin)
        newPt = R * [Xin(i); Yin(i)];
        Xout(i) = newPt(1);
        Yout(i) = newPt(2);
    end
end
