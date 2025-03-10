% helperCreateLCBusActorsEgoObject function takes number of actors in the
% scenario as input and creates a bus object: BusActorsEgo required for the
% test bench model in MATLAB base workspace

% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2019-2020 The MathWorks, Inc.

function helperCreateLCBusActorsEgoObject(numActors)

BusActorsEgo = Simulink.Bus;
BusActorsEgo.Description = '';
BusActorsEgo.DataScope = 'Auto';
BusActorsEgo.HeaderFile = '';
BusActorsEgo.Alignment = -1;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'NumActors';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.SampleTime = -1;
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'Time';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [1 1];
saveVarsTmp{1}(2, 1).DataType = 'double';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).SampleTime = -1;
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
saveVarsTmp{1}(3, 1) = Simulink.BusElement;
saveVarsTmp{1}(3, 1).Name = 'Actors';
saveVarsTmp{1}(3, 1).Complexity = 'real';
saveVarsTmp{1}(3, 1).Dimensions = [numActors 1];
saveVarsTmp{1}(3, 1).DataType = 'Bus: BusActors1Actors';
saveVarsTmp{1}(3, 1).Min = [];
saveVarsTmp{1}(3, 1).Max = [];
saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(3, 1).SampleTime = -1;
saveVarsTmp{1}(3, 1).DocUnits = '';
saveVarsTmp{1}(3, 1).Description = '';
BusActorsEgo.Elements = saveVarsTmp{1};
clear saveVarsTmp;

assignin('base', 'BusActorsEgo', BusActorsEgo);

end

