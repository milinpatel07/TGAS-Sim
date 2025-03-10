classdef HelperValidatorBoundingBoxList < nav.StateValidator
%HelperValidatorBoundingBoxList Create validator for BoundingBox list
%
%   HelperValidatorBoundingBoxList creates a validatorBoundingBoxList
%   object for polygonal collision checking. This object propagates
%   obstacle list states in time and validates timestamped query states. It
%   uses separating axis theorem to check for collision for oriented
%   bounding boxes.
%
%   SV = HelperValidatorBoundingBoxList creates a validatorBoundingBoxList
%   object for polygonal collision checking. This object propagates
%   bounding box states in time and validates timestamped query states
%
%   SV = HelperValidatorBoundingBoxList('PropeRtEgoyName', PropeRtEgoyValue, ...)
%   sets each specified propeRtEgoy of the validatorBoundingBoxList object
%   to the specified value upon construction.
%
%   HelperValidatorBoundingBoxList properties:
%       EgoProfile              - Ego vehicle profile specified as 2-by-4 array
%       EgoState                - States of ego vehicle in world frame
%       Frame                   - Coordinate frame used for obstacle states
%
%   HelperValidatorBoundingBoxList methods:
%       addObstacle     - Add an obstacle to the ObstacleList
%       updateObstacle  - Update states of object
%       removeObstacle  - Delete obstacle from the ObstacleList
%       updateEgoState  - Update the current state of ego vehicle
%       isStateValid    - Check if the state is valid    
%
%   NOTE: The name of this System Object and it's functionality may change
%   without notice in a future release, or the System Object itself may be
%   removed.
%

%   Copyright 2019-2020 The MathWorks, Inc.
    
    properties        
        % Structure array containing list of obstacles
        ObstacleList;
        
        % MaxNumObstacles holds maximum number of obstacles
        MaxNumObstacles = 6;
        
        % Dimensions of Ego Vehicle given as a 2-by-4 array of vertices in
        % counter-clockwise direction given as (x,y). Origin of the ego
        % vehicle is assumed to be at the center of this rectangle.
        EgoProfile;
        
        % Ego frame transformation for latest measurement update
        EgoState = struct('Pose',[0 0 0], 'Velocity', [0 0],'AngularVelocity',0);
        
        % Planner object used for Frenet<->Cartesian transformation
        PlannerObj
        
        MaxRange = 70;
    end
    
    properties(Access = private)        
        % Scalar integer maintaining latest index for IDs
        CurrentIDIndex = 1;
        
        % Structure array used to map IDs with indexes of ObstacleList
        MapIndex;
    end
    
    properties (Constant)        
        % Vertices of the car in counter-clockwise direction given as (x,y)
        CarProfile = [3.7,0.9; 3.7,-0.9; -1.0,-0.9; -1.0,0.9]'*1.2;
    end
    %----------------------------------------------------------------------
    % Constructor and methods
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function obj = HelperValidatorBoundingBoxList(egoProfile,waypoints)
            %HelperValidatorBoundingBoxList Create validator for BoundingBox list
            %   validatorBoundingBoxList creates a validatorBoundingBoxList object for
            %   rectangle collision checking. This object propagates obstacle list states in
            %   time and validates timestamped query states.
            
            obj@nav.StateValidator(HelperStateSpaceTrajectoryOptimalFrenet);
            if ~isempty(egoProfile)
                obj.EgoProfile = egoProfile;
            else
                obj.EgoProfile = obj.CarProfile;
            end
            
            % Declare variable size property MapIndex
            coder.varsize('mapIndex','ID');
            ID = 'obstacle_0';
            mapIndex = struct('Name',ID,'Index',0);
            obj.MapIndex = mapIndex;
            
            % Declare variable size property ObstacleList
            coder.varsize('frame');
            frame = 'ego';
            obstacle = struct('Pose',[0 0 0], 'Velocity', [0 0], 'Profile',obj.CarProfile,'Frame',frame);
            coder.varsize('obstacle');
            obj.ObstacleList = repmat(obstacle,obj.MaxNumObstacles,1);
            
            obj.PlannerObj = trajectoryOptimalFrenet(waypoints,validatorOccupancyMap);
            
        end
        
        %------------------------------------------------------------------
        function isValid = isStateValid(obj, queryStates)
            % Check if the timestamped query state is valid
            %   FLAG = isStateValid(SV, QUERYSTATE) determines validity of QUERYSTATE specified
            %   as a N-by-7 matrix in the with states in the order [x, y, theta, kappa, speed,
            %   acceleration, time]. It uses a uniform velocity assumption to determine the
            %   state of the objects in the ObstacleList at the given time from state validator
            %   object SV.
            
            isValid = ~obj.checkCollision(queryStates(:,1:3),queryStates(:,7));
        end
        
        %------------------------------------------------------------------
        %isMotionValid Check if path between states is valid
        function [isValid, lastValidState] = isMotionValid(~, ~, ~)
            %isMotionValid Dummy function performs no operations
            isValid = false;
            lastValidState = zeros(1,7);
        end
        
        %------------------------------------------------------------------
        %COPY Create deep copy of object
        function copyObj = copy(obj)
            
            copyObj = obj;
        end
        
        %------------------------------------------------------------------
        function ID = addObstacle(obj, pose, velocity, obstacleProfile, frame)
            % Add an obstacle to the ObstacleList.
            %   addObstacle(SV, POSE, VELOCITY, OBSTACLEPROFILE) adds an obstacle with its
            %   states POSE and VELOCITY passed as input arguments along with the vertices of
            %   the obstacle specified as a 2-by-4 vertices in counter-clockwise direction as
            %   OBSTACLEPROFILE to state validator object SV and returns its ID.
            
            ID = sprintf('obstacle_%i',int16(obj.CurrentIDIndex));
            
            validatestring(frame, {'ego','world'},"addObstacle");
            
            if obj.CurrentIDIndex == 1
                obj.MapIndex(1) = struct('Name',ID,'Index',obj.CurrentIDIndex);
                obj.ObstacleList(1) = struct('Pose',pose(:)', 'Velocity', velocity(:)', 'Profile',obstacleProfile(1:2,1:4),'Frame',frame);
                
            else
                obj.MapIndex = [obj.MapIndex, struct('Name',ID,'Index',obj.CurrentIDIndex)];
                obj.ObstacleList(obj.CurrentIDIndex) = struct('Pose',pose(:)', 'Velocity', velocity(:)', 'Profile',obstacleProfile(1:2,1:4),'Frame',frame);
            end
            obj.CurrentIDIndex = obj.CurrentIDIndex + 1;
            
        end
        
        %------------------------------------------------------------------
        function removeObstacle(obj, ID)
            % Delete obstacle with ID from the ObstacleList.
            %   removeObstacle(SV, ID) deletes the obstacle identified as ID from the
            %   ObstacleList from state validator object SV.
            
            for i = 1:numel(obj.MapIndex)
                if strcmp(obj.MapIndex(i).Name,ID)
                    obj.MapIndex(i) = [];
                    break
                end
            end
        end
        
        %------------------------------------------------------------------
        function updateObstacle(obj, ID, pose, velocity, obstacleProfile,frame)
            % Update the states of an obstacle in the ObstacleList.
            %   updateObstacle(SV, ID, POSE, VELOCITY, OBSTACLEPROFILE, FRAME) updates obstacle
            %   identified with ID its states POSE and VELOCITY passed as input arguments along
            %   with the vertices of the obstacle specified as a 2-by-4 vertices in
            %   counter-clockwise direction OBSTACLEPROFILE in state validator object SV.
            validatestring(frame, {'ego','world'},"updateObstacle");
            for i = 1:numel(obj.MapIndex)
                if strcmp(obj.MapIndex(i).Name,ID)
                    obj.ObstacleList(obj.MapIndex(i).Index) = struct('Pose',pose(:)', 'Velocity', velocity(:)', 'Profile',obstacleProfile,'Frame',frame);
                    break
                end
            end
        end
        
        %------------------------------------------------------------------
        function updateEgoState(obj,pose, velocity,angularVelocity)
            % Update the states of egoVehicle.
            %   updateEgoState(SV, POSE, VELOCITY,ANGULARVELOCITY) updates ego vehicle states POSE,
            %   VELOCITY specified in world frame coordinates in state validator
            %   object SV.
            obj.EgoState = struct('Pose',pose(:)', 'Velocity',velocity(:)','AngularVelocity',angularVelocity);
        end
        
    end
    
    %----------------------------------------------------------------------
    % Private methods
    %----------------------------------------------------------------------
    methods(Access = private)
        %------------------------------------------------------------------
        function [objectFrenetPose,refPathCartesianPose,thetaObstacle,polygonHomogeneous] = computeFrenetStates(obj,ID)
            % computeFrenetStates Compute frenet states of obstacles
            
            % Form a 3x3 transformation matrix using ego pose
            egoTheta = obj.EgoState.Pose(3);
            RtEgo = [cos(egoTheta) -sin(egoTheta) obj.EgoState.Pose(1);
                sin(egoTheta) cos(egoTheta) obj.EgoState.Pose(2);
                0 0 1];
            
            currentObstacleIndex = 0;
            for i = 1:numel(obj.MapIndex)
                if strcmp(obj.MapIndex(i).Name,ID)
                    currentObstacleIndex = obj.MapIndex(i).Index;
                    break
                end
            end
            
            x = obj.ObstacleList(currentObstacleIndex).Pose(1);
            y = obj.ObstacleList(currentObstacleIndex).Pose(2);
            
            if strcmp(obj.ObstacleList(currentObstacleIndex).Frame,'ego')
                
                % Transform the velocities to world frame
                RtEgoVel = RtEgo;
                RtEgoVel(1:2,3) = obj.EgoState.Velocity(1:2);
                velocity = [obj.ObstacleList(currentObstacleIndex).Velocity, 1] + cross([0 0 obj.EgoState.AngularVelocity(1)], [x y 0]);
                velocity = RtEgoVel * velocity';
                velocityX = velocity(1);
                velocityY = velocity(2);
                
                % Use velocity to estimate heading if non-zero
                if isequal(obj.ObstacleList(currentObstacleIndex).Velocity,[0 0])
                    theta = obj.ObstacleList(currentObstacleIndex).Pose(3);
                else
                    % We assume a holonomic obstacle which is moving with a
                    % uniform velocity its heading direction hence we can
                    % use velocity to estimate the heading of the obstacle
                    % in ego frame
                    theta = atan2(velocityY,velocityX) - egoTheta;
                end
                
                % Form a 3x3 transformation matrix using obstacle pose
                RtObstacle = [cos(theta) -sin(theta) x;
                    sin(theta) cos(theta) y;
                    0 0 1];
                
                RInit = RtEgo * RtObstacle;
            else
                % No need to transform if obstacle is in world frame
                velocityX = obj.ObstacleList(currentObstacleIndex).Velocity(1);
                velocityY = obj.ObstacleList(currentObstacleIndex).Velocity(2);
                
                theta = deg2rad(obj.ObstacleList(currentObstacleIndex).Pose(3));
                
                % Form a 3x3 transformation matrix using obstacle pose
                RtObstacle = [cos(theta) -sin(theta) x;
                    sin(theta) cos(theta) y;
                    0 0 1];
                
                RInit = RtObstacle;
            end
            
            thetaObstacle = atan2(RInit(2,1),RInit(1,1));
            refPathFrenetPose = obj.PlannerObj.cart2frenet([obj.EgoState.Pose(1),obj.EgoState.Pose(2),egoTheta,0,0,0]);
            refPathCartesianPose = obj.PlannerObj.frenet2cart([refPathFrenetPose(1:3),0,0,0]);
            
             if (angdiff(thetaObstacle, refPathCartesianPose(3)) < -pi/2 || angdiff(thetaObstacle, refPathCartesianPose(3)) > pi/2)
                  inverseThetaObstacle = angdiff(pi, thetaObstacle);
                objectFrenetPose = obj.PlannerObj.cart2frenet([RInit(1,3),RInit(2,3),inverseThetaObstacle,0,norm([velocityX, velocityY]),0]);
            else
                objectFrenetPose = obj.PlannerObj.cart2frenet([RInit(1,3),RInit(2,3),thetaObstacle,0,norm([velocityX, velocityY]),0]);
            end
            
            polygon = obj.ObstacleList(currentObstacleIndex).Profile;
            polygonHomogeneous = [polygon(1,:); polygon(2,:); ones(1,size(polygon,2))];
        end
        
        %------------------------------------------------------------------
        function transformedPolygon = transformObstacle(obj, objectFrenetPose,refPathCartesianPose,thetaObstacle, polygonHomogeneous, time)
            % transformObstacle Transform  obstacles to world coordinates
            
            if ((thetaObstacle - refPathCartesianPose(3)) <= -pi/2 || (thetaObstacle - refPathCartesianPose(3)) >= pi/2)
                objectFrenetPose(1) = objectFrenetPose(1) + time*-objectFrenetPose(2);
                objectFrenetPose(5:6) = [0,0];
                finalCartPose = obj.PlannerObj.frenet2cart(objectFrenetPose);
            else
                objectFrenetPose(1) = objectFrenetPose(1) + time*objectFrenetPose(2);
                objectFrenetPose(5:6) = [0,0];
                finalCartPose = obj.PlannerObj.frenet2cart(objectFrenetPose);
            end
            
            Rfinal = [cos(finalCartPose(3)) -sin(finalCartPose(3)) finalCartPose(1);
                sin(finalCartPose(3)) cos(finalCartPose(3)) finalCartPose(2);
                0 0 1];
            
            % Transform the polygon vertices to world frame
            transformedPolygon = Rfinal*polygonHomogeneous;
            transformedPolygon = transformedPolygon(1:2,:)';
        end
        
        %------------------------------------------------------------------
        function isColliding = checkCollision(obj,poses,timestamps)
            % checkCollision Check for collision with all obstacles
            isColliding = false;
            obstacleIDs = repmat({'obstacle_0'},numel(obj.MapIndex),1);
            for i = 1:numel(obj.MapIndex)
                obstacleIDs{i} = obj.MapIndex(i).Name;
            end
            coder.varsize('validIDs');
            validIDs = zeros(0,0);
            for id = 1:numel(obstacleIDs)
                for i = 1:numel(obj.MapIndex)
                    if strcmp(obj.MapIndex(i).Name,obstacleIDs{id})
                        currentObstacleIndex = obj.MapIndex(i).Index;
                        if strcmp(obj.ObstacleList(currentObstacleIndex).Frame,'ego')
                            x = obj.ObstacleList(currentObstacleIndex).Pose(1);
                            y = obj.ObstacleList(currentObstacleIndex).Pose(2);
                            if(norm([x,y]) < obj.MaxRange)
                                validIDs = vertcat(validIDs,id);
                            end
                        else
                            if(norm(obj.ObstacleList(currentObstacleIndex).Pose(1:2) - poses(1,1:2)) < obj.MaxRange)
                                validIDs = vertcat(validIDs,id);
                            end
                        end
                        break
                    end
                end
            end
            % Iterate over timestamps projecting obstacles
            
            for id = 1:numel(validIDs)
                [objectFrenetPose,refPathCartesianPose,thetaObstacle,polygonHomogeneous] = obj.computeFrenetStates(obstacleIDs{validIDs(id)});
                for index = 1:numel(timestamps)
                    % Transform obstacles to world coordinates by
                    % Propagating the obstacle using uniform velocity
                    % assumption
                    obstaclePolygon = obj.transformObstacle(objectFrenetPose,refPathCartesianPose,thetaObstacle,polygonHomogeneous,timestamps(index));

                    % Transform egoVehicle profile vertices
                    egoTheta = poses(index,3);
                    RtEgo = [cos(egoTheta) -sin(egoTheta) poses(index,1);
                        sin(egoTheta) cos(egoTheta) poses(index,2);
                        0 0 1];
                    
                    egoPolygonHomogeneous = RtEgo * [obj.EgoProfile(1,:); obj.EgoProfile(2,:); ones(1,size(obj.EgoProfile,2))];
                    egoPolygon = egoPolygonHomogeneous(1:2,:)';
                    
                    if (HelperValidatorBoundingBoxList.checkIntersections(obstaclePolygon, egoPolygon) == true)
                        isColliding = true;
                        break;
                    end
                end
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Private methods
    %----------------------------------------------------------------------
    methods(Static,Access = private)
        %------------------------------------------------------------------
        function [n1,n2] = getNormals(vertices)
            %getNormals Compute normals for the vertices of a rectangle
            
            n1 = [vertices(2,1) - vertices(1,1),vertices(2,2) - vertices(1,2)];
            n1 = n1/sqrt(n1(1) .* n1(1) + n1(2) .* n1(2));
            n2 = [vertices(3,1) - vertices(2,1),vertices(3,2) - vertices(2,2)];
            n2 = n2/sqrt(n2(1) .* n2(1) + n2(2) .* n2(2));
        end
        
        %------------------------------------------------------------------
        function [minProjection, maxProjection] = getMinMaxProjection(vertices,normal)
            %getMinMaxProjection Compute projection of vertices on normal
            minProjection = inf;
            maxProjection = -inf;
            
            % Loop over all the vertices to find min max projections
            for i = 1:size(vertices,1)
                projection = vertices(i,1) .* normal(1) + vertices(i,2) .*  normal(2);
                if projection < minProjection
                    minProjection = projection;
                end
                if projection > maxProjection
                    maxProjection = projection;
                end
            end
        end
        
        %------------------------------------------------------------------
        function isIntersecting = checkIntersections(polygonA,polygonB)
            %checkIntersections Check intersection between two rectangles
            
            % Compute normals for both polygons
            [normalA1, normalA2] = HelperValidatorBoundingBoxList.getNormals(polygonA);
            [normalB1, normalB2] = HelperValidatorBoundingBoxList.getNormals(polygonB);
            
            % Compute projection on normals of polygon A
            [minAA2, maxAA2] = HelperValidatorBoundingBoxList.getMinMaxProjection(polygonA,normalA2);
            [minBA2, maxBA2] = HelperValidatorBoundingBoxList.getMinMaxProjection(polygonB,normalA2);
            [minAA1, maxAA1] = HelperValidatorBoundingBoxList.getMinMaxProjection(polygonA,normalA1);
            [minBA1, maxBA1] = HelperValidatorBoundingBoxList.getMinMaxProjection(polygonB,normalA1);
            
            % Compute projection on normals of polygon B
            [minAB2, maxAB2] = HelperValidatorBoundingBoxList.getMinMaxProjection(polygonA,normalB2);
            [minBB2, maxBB2] = HelperValidatorBoundingBoxList.getMinMaxProjection(polygonB,normalB2);
            [minAB1, maxAB1] = HelperValidatorBoundingBoxList.getMinMaxProjection(polygonA,normalB1);
            [minBB1, maxBB1] = HelperValidatorBoundingBoxList.getMinMaxProjection(polygonB,normalB1);
            
            % Compare the projections to check for separation line. If it
            % exists then the rectangles are not intersecting
            isIntersecting = ~(maxAA2 <= minBA2 || maxBA2 <= minAA2 || ...
                maxAA1 <= minBA1 || maxBA1 <= minAA1 || ...
                maxAB2 <= minBB2 || maxBB2 <= minAB2 || ...
                maxAB1 <= minBB1 || maxBB1 <= minAB1);
            
        end
    end
end
