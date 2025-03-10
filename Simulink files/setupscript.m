
% Set up Script 

%% 
Ts = 0.1;               % Simulation sample time  (s)

%% simulation stop time
simstoptime = 60;
%Path following Controller Parameters
%all default parameters but we can store them into variables for ease and recalling later
time_gap        = 1;   % time gap               (s)
default_spacing = 5;    % default spacing        (m)

max_ac          = 2;     % Maximum acceleration   (m/s^2)
min_ac          = -3;    % Minimum acceleration   (m/s^2)
max_steer       = 0.26;  % Maximum steering       (rad)
min_steer       = -0.26; % Minimum steering       (rad) 
PredictionHorizon = 30;  % Prediction horizon     


% %% Create driving scenario
% % The scenario name is a scenario file created by the Driving Scenario Designer App. 
 scenariosNames = {
     'mySim_LF_Curve2.mat',...                                % ScenarioId =1
         'customlc.mat'};                                   % scenarioId = 2
     
scenarioId = 2;

% The scenario file is converted to a drivingScenario object
% initial conditions of ego car and actor profiles
[scenario,egoCar,actor_Profiles] = helperSessionToScenario(scenariosNames{scenarioId});

 v_set = 33;

% Default assessments
assessment.TimeGap = 0.8;
assessment.LongitudinalJerkMax = 5;
assessment.LateralJerkMax = 5;

% Initial condition for the ego car in ISO 8855 coordinates
v0_ego   = egoCar.v0;          % Initial speed of the ego car(initial longitudinal velocity)        (m/s)
x0_ego   = egoCar.x0;          % Initial x position of ego car (initial longitudinal position)         (m)
y0_ego   = egoCar.y0;          % Initial y position of ego car(initial lateral displacement)          (m)
yaw0_ego = egoCar.yaw0;        % Initial yaw angle of ego car           (rad)

if y0_ego >0
    initial_lane = 1; % the vehicle is in the left lane
else
    initial_lane = 0; % the vehicle is in the right lane
end

% Convert ISO 8855 to SAE J670E coordinates
y0_ego = -y0_ego;
yaw0_ego = -yaw0_ego;
%% Extract scenario information
% Road center information
assignin('base', 'globalPlanPoints', scenario.RoadSegments.RoadCenters);
evalin('base', 'globalPlanPoints(:,3) = [];');

% Get Ego information from scenario
EgoActor = scenario.Actors;
initialEgoX = EgoActor(1).Position(1);
initialEgoY = EgoActor(1).Position(2);
initialEgoYaw = deg2rad(EgoActor(1).Yaw(1));
% Ego set speed (m/s)
 egoSetVelocity = hypot(EgoActor(1).Velocity(1), EgoActor(1).Velocity(2));

% Minimum threshold distance to evaluate isGoalReached
minDistThreshold = 3.7;
%assignin('base', 'minDistThreshold', minDistThreshold);


% %% Tracking and Sensor Fusion Parameters                        Units

 M           = 2;        % Tracker M value for M-out-of-N logic  (N/A)
 N           = 3;        % Tracker M value for M-out-of-N logic  (N/A)



velSelector = [0,1,0,0,0,0; 0,0,0,1,0,0]; % Velocity selector   (N/A)


% Lane Change maneuver

velocity_right_lane_max = 35; % maximal speed allowed on the right lane
velocity_left_lane_max = 35; % maximal speed allowed on the left lane
man_or_auto = 1; % 0:manual mode,1:autonomous mode
VB= 30.5556;


%% Bus Creation
% Load the Simulink model
modelName = 'TGASMODEL';
wasModelLoaded = bdIsLoaded(modelName);
if ~wasModelLoaded
    load_system(modelName)
end

% % Create buses for lane sensor and lane sensor boundaries
createLaneSensorBuses;

% load the bus for scenario reader
blk=find_system(modelName,'System','driving.scenario.internal.ScenarioReader');
s = get_param(blk{1},'PortHandles');
get(s.Outport(1),'SignalHierarchy');

% Set the scenario reader file name to the selected scenario
set_param(blk{1},'ScenarioFileName',scenariosNames{scenarioId});
% Set the scenario reader file name to the selected scenario
% set_param(blk{1},'ScenarioFileName',scenariosNames{scenarioId});


% Create the bus of tracks (output from referenced model)
refModel = 'RefMdl';
 wasReModelLoaded = bdIsLoaded(refModel);
if ~wasReModelLoaded
     load_system(refModel)
    blk=find_system(refModel,'System','multiObjectTracker');
    multiObjectTracker.createBus(blk{1});
    close_system(refModel)
else
    blk=find_system(refModel,'System','multiObjectTracker');
    multiObjectTracker.createBus(blk{1});
end
if ~wasModelLoaded
    close_system(modelName)
end



