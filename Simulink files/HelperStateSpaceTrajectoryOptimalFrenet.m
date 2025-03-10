classdef HelperStateSpaceTrajectoryOptimalFrenet < nav.StateSpace
    %HelperStateSpaceTrajectoryOptimalFrenet SE(2) state space with dynamic
    % states
    % The HelperStateSpaceTrajectoryOptimalFrenet object stores
    % parameters and has functions for the state space composed of a
    % state vector with 7 variables: [x y theta kappa speed acceleration
    % time].
    %
    %   NOTE: The name of this System Object and it's functionality may 
    %   change without notice in a future release, 
    %   or the System Object itself may be removed.
    %

    %   Copyright 2019-2020 The MathWorks, Inc.

    properties(Constant, Access = protected)
        %DefaultStateBounds Default bounds for trajectoryOptimalFrenet state space
        DefaultStateBounds = [-inf, inf; ...
                -inf, inf; ...
                -pi,  pi; ...
                -inf, inf;...
                -inf, inf; ...
                -inf, inf; ...
                -inf, inf];
    end
    methods
        function obj = HelperStateSpaceTrajectoryOptimalFrenet()
            %HelperStateSpaceTrajectoryOptimalFrenet Construct State Space
            
            bounds = HelperStateSpaceTrajectoryOptimalFrenet.DefaultStateBounds;
            obj@nav.StateSpace("HelperStateSpaceTrajectoryOptimalFrenet",7,bounds)
        end
    end
    methods
        % Dummy abstract methods inherited from nav.StateSpace
        function distance(~)
        end
        function interpolate(~)
        end
        function sampleGaussian(~)
        end
        function sampleUniform(~)
        end
        function enforceStateBounds(~)
        end
        function copyObj = copy(obj)
            copyObj = obj;
        end
        
    end
end
