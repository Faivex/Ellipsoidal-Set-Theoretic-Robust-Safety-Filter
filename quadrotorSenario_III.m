clc;
clear;
close all;

%% Linearize the system
% Load system parameters
params = loadSystemParameters();

% Define equilibrium point
[x_eq, u_eq] = defineEquilibriumPoint;

% Calculate linearized model
[A, B] = linearizeModel(@F, @G, x_eq, u_eq, params);
n = size(A,1);
m = size(B,2);

% Attitude control only (ignore position and velocity)
A_attitude = A(7:12, 7:12);
B_attitude = B(7:12, :);

%% Design robust controller using LMI
u_max = 1*[1e-4; 1e-4; 1e-4]; % max control input for each channel
x_max = [40*pi/180; 40*pi/180; pi; 1; 1; 1]; % max roll and pitch angles in rad
w_max = 1e-5; % disturbance gain
E_attitude = w_max*B_attitude; % assume disturbance enters like control input
E = w_max*B; % full disturbance matrix

max_delta_max = 0.65;
E_nl = sqrt(2)*[E_attitude  max_delta_max*eye(6)];
[K, P] = robustControlDesign(A_attitude, B_attitude, E_nl, u_max, x_max);

% Compute linearization error bound for attitude dynamics
delta_max = computeAttitudeLinearizationErrorBound(params, P);

if delta_max > max_delta_max
    error(['Computed linearization error bound delta_max = ', num2str(delta_max), ...
             ' exceeds allowable maximum of ', num2str(max_delta_max)]);
else
    disp(['Computed linearization error bound delta_max = ', num2str(delta_max), ...
          ' is within allowable maximum of ', num2str(max_delta_max)]);
end

%% Simulation
% Initial condition
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % initial state
% Simulation time
T = 10; % total time
dt = 0.01; % time step
time = 0:dt:T; % time vector
% Preallocate state and control input arrays for both scenarios
x_safe = zeros(length(x0), length(time));
u_safe = zeros(m, length(time));
x_unsafe = zeros(length(x0), length(time));
u_unsafe = zeros(m, length(time));
x_safe(:,1) = x0; % set initial condition
x_unsafe(:,1) = x0; % set initial condition

% Define alpha as a function handle before simulation loop
h_max = 1.0;
h_min = 0.2;
alpha_fun = @(x) min(max((x'*P*x - h_min)/(h_max - h_min), 0), 1);

% Preallocate desired position storage over time
pos_des = zeros(2, length(time)-1);

% Simulation loop
for k = 1:length(time)-1
    % Run both scenarios in parallel
    for scenario = 1:2
        if scenario == 1
            x_current = x_safe(:,k);
        else
            x_current = x_unsafe(:,k);
        end
        
        % Desired trajectory
        % Circular trajectory
        radius = 5; % radius of the circle
        omega_traj = 1; % angular speed
        x_des = radius * cos(omega_traj * time(k));
        y_des = radius * sin(omega_traj * time(k));
        % Store desired position for plotting
        pos_des(:, k) = [x_des; y_des];
        
        % Simple PD controller for position to generate desired roll and pitch
        Kp_y = -0.2; % position proportional gain
        Kd_y = -0.2; % position derivative gain
        Kp_x = 0.2; % position proportional gain
        Kd_x = 0.2; % position derivative gain
        vx = x_current(4);
        vy = x_current(5);
        ex = x_des - x_current(1);
        ey = y_des - x_current(2);
        theta_des = Kp_x*ex - Kd_x*vx;
        phi_des = Kp_y*ey - Kd_y*vy;

        % Limit desired angles
        phi_des = max(min(phi_des, 60*pi/180), -60*pi/180);
        theta_des = max(min(theta_des, 60*pi/180), -60*pi/180);

        % Attitude control
        eta = x_current(7:9); % current angles [phi; theta; psi]
        eta_des = [phi_des; theta_des; 0]; % desired angles
        e_eta = eta - eta_des; % angle error
        omega = x_current(10:12); % current angular rates [p; q; r]
        k_p_phi = -1e-3; k_d_phi = 2e-4;
        k_p_theta = -1e-3; k_d_theta = 2e-4;
        k_p_psi = -3e-4; k_d_psi = 1e-4;
        u_nominal = [k_p_phi*e_eta(1) + k_d_phi*(0 - omega(1));
                    k_p_theta*e_eta(2) + k_d_theta*(0 - omega(2));
                    k_p_psi*e_eta(3) + k_d_psi*(0 - omega(3))];

        % Saturate control inputs
        for i = 1:length(u_nominal)
            if abs(u_nominal(i)) > u_max(i)
                u_nominal(i) = sign(u_nominal(i)) * u_max(i);
            end
        end        
    
        for i = 1:length(u_nominal)
            if abs(u_nominal(i)) > u_max(i)
                u_nominal(i) = sign(u_nominal(i)) * u_max(i);
            end
        end
        if scenario == 1
            % With safety filter
            alpha = alpha_fun(x_current(7:12));
            u_current = (alpha)*K * x_current(7:12)+(1 - alpha)*u_nominal;
        else
            % Without safety filter
            u_current = u_nominal;
        end

        % Store saturated control inputs
        if scenario == 1
            u_safe(:,k) = u_current;
        else
            u_unsafe(:,k) = u_current;
        end

        % Define disturbance values
        w = @(t) [sin(2*t); cos(2*t); 0]; % define w as a function of time
        w_t = w(time(k)); % evaluate w at current time

        % Simulate system
        ode_fun = @(t, state) F(state, params) + G(state, params)*u_current + E*w_t;
        [~, x_next] = ode45(ode_fun, [0 dt], x_current);
        
        if scenario == 1
            x_safe(:,k+1) = x_next(end,:)';
        else
            x_unsafe(:,k+1) = x_next(end,:)';
        end
    end
end

% Plot results comparing both scenarios
plotResults(time, x_safe, x_unsafe, u_safe, u_unsafe, w, w_max, P, alpha_fun, ...
           u_max, x_max, pos_des, 1);

%% Functions
function F_x = F(state, params)
    % Dynamics function F
    % state = [x; y; z; vx; vy; vz; phi; theta; psi; p; q; r]

    % Extract state variables
    vx = state(4);
    vy = state(5);
    vz = state(6);
    phi = state(7);
    theta = state(8);
    psi = state(9);
    p = state(10);
    q = state(11);
    r = state(12);

    % Gravitational acceleration
    g = params.g;
    m = params.m;

    % State derivatives
    % dot{p} = v 
    p_dot = [vx; vy; vz];

    % Compute trigonometric functions
    c_phi = cos(phi);
    s_phi = sin(phi);
    c_theta = cos(theta);
    s_theta = sin(theta);
    c_psi = cos(psi);
    s_psi = sin(psi);

    % Rotation matrix (body to inertial)
    C_IB = [c_theta*c_psi, c_theta*s_psi, -s_theta;
            -c_phi*s_psi + s_phi*s_theta*c_psi, c_phi*c_psi + s_phi*s_theta*s_psi, s_phi*c_theta;
            s_phi*s_psi + c_phi*s_theta*c_psi, -s_phi*c_psi + c_phi*s_theta*s_psi, c_phi*c_theta];
    % dot{v} = g*e_3 - \frac{1}{m}R(\eta)(Fe_3)
    e3 = [0; 0; 1];
    v_dot = g*e3 - (1/m)*C_IB*(m*g*e3); % Assuming Fe_3 = mg*e_3 at hover

    % dot{\eta} = W(\eta)\omega
    W = [1, s_phi*tan(theta), c_phi*tan(theta);
         0, c_phi, -s_phi;
         0, s_phi/cos(theta), c_phi/cos(theta)];
    eta_dot = W * [p; q; r];

    % dot{\omega} = J^{-1}(-\omega \times J\omega)
    J = diag([params.Ixx, params.Iyy, params.Izz]);
    omega = [p; q; r];
    omega_dot = J \ (-cross(omega, J*omega)); % Assuming no control

    % Combine all derivatives
    F_x = [p_dot; v_dot; eta_dot; omega_dot];
end

function G_x = G(~, params)
    % Control input matrix G
    Ixx = params.Ixx;
    Iyy = params.Iyy;
    Izz = params.Izz;

    % Control inputs affect acceleration and angular acceleration
    G_x = zeros(12, 3);

    % Torques affect angular accelerations
    G_x(10, 1) = 1/Ixx; % Roll torque
    G_x(11, 2) = 1/Iyy; % Pitch torque
    G_x(12, 3) = 1/Izz; % Yaw torque
end