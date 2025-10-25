function params = loadSystemParameters()
    % Load system parameters
    params = struct();
    
    % Physical parameters
    params.m = 0.028;      % Mass [kg]
    params.g = 9.81;       % Gravity [m/s^2]
    
    % Inertial parameters
    params.Ixx = 16.6e-6;  % Moment of inertia around x-axis
    params.Iyy = 16.6e-6;  % Moment of inertia around y-axis
    params.Izz = 29.3e-6;  % Moment of inertia around z-axis
end