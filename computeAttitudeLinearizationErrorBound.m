function [delta_max, worst_case_state] = computeAttitudeLinearizationErrorBound(params, P)
    % Compute maximum bound on linearization error for attitude dynamics
    % The linearization error Delta(t) is given by:
    % Delta(t) = [(W(eta) - I_3)*omega; -J^(-1)*(omega x J*omega)]
    %
    % Inputs:
    %   params - system parameters containing inertia matrix J
    %   P - Lyapunov matrix defining the ellipsoidal region x'Px <= 1
    % Outputs:
    %   delta_max - maximum linearization error bound for attitude states
    %   worst_case_state - state [phi; theta; psi; p; q; r] that produces maximum error
    
    % System parameters
    J = diag([params.Ixx, params.Iyy, params.Izz]);  % Inertia matrix
    J_inv = inv(J);
    
    % Objective function: minimize negative of linearization error (to maximize error)
    objective = @(x) -evaluateLinearizationErrorScalar(x, J, J_inv);
    
    % Constraint function: x'*P*x <= 1
    constraint = @(x) ellipsoidConstraint(x, P);
    
    % Optimization options
    options = optimoptions('fmincon', ...
        'Display', 'off', ...
        'Algorithm', 'interior-point', ...
        'MaxIterations', 1000, ...
        'MaxFunctionEvaluations', 5000, ...
        'OptimalityTolerance', 1e-8, ...
        'ConstraintTolerance', 1e-8);
    
    % Multiple starting points to find global maximum
    n_starts = 20;
    best_delta = -inf;
    best_state = zeros(6, 1);
    
    % Generate diverse starting points
    starting_points = generateStartingPoints(P, n_starts);
    
    for i = 1:n_starts
        x0 = starting_points(:, i);
        
        try
            % Run optimization
            [x_opt, fval, exitflag] = fmincon(objective, x0, [], [], [], [], [], [], constraint, options);
            
            if exitflag > 0  % Successful convergence
                current_delta = -fval;  % Convert back to positive value
                
                if current_delta > best_delta
                    best_delta = current_delta;
                    best_state = x_opt;
                end
            end
        catch ME
            % Skip this starting point if optimization fails
            continue;
        end
    end
    
    % Additional optimization starting from eigenvector directions
    [V, D] = eig(P);
    eigenvals = diag(D);
    
    for i = 1:6
        % Start from scaled eigenvector (on boundary)
        x0 = V(:, i) / sqrt(eigenvals(i)) * 0.9;  % slightly inside boundary
        
        try
            [x_opt, fval, exitflag] = fmincon(objective, x0, [], [], [], [], [], [], constraint, options);
            
            if exitflag > 0
                current_delta = -fval;
                
                if current_delta > best_delta
                    best_delta = current_delta;
                    best_state = x_opt;
                end
            end
        catch ME
            continue;
        end
    end
    
    delta_max = best_delta;
    worst_case_state = best_state;
    
    % Compute individual components for reporting
    [~, delta_1_norm, delta_2_norm] = evaluateLinearizationError(worst_case_state, J, J_inv);
    
    % Display detailed results
    fprintf('\n=== Attitude Linearization Error Analysis (using fmincon) ===\n');
    fprintf('Maximum error in eta_dot component: %.6f\n', delta_1_norm);
    fprintf('Maximum error in omega_dot component: %.6f\n', delta_2_norm);
    fprintf('Combined maximum linearization error bound: %.6f\n', delta_max);
    fprintf('Optimization conducted within ellipsoidal region x''Px <= 1\n');
    fprintf('\nWorst-case state that maximizes linearization error:\n');
    fprintf('  phi   = %8.4f rad (%7.2f°)\n', worst_case_state(1), worst_case_state(1)*180/pi);
    fprintf('  theta = %8.4f rad (%7.2f°)\n', worst_case_state(2), worst_case_state(2)*180/pi);
    fprintf('  psi   = %8.4f rad (%7.2f°)\n', worst_case_state(3), worst_case_state(3)*180/pi);
    fprintf('  p     = %8.4f rad/s\n', worst_case_state(4));
    fprintf('  q     = %8.4f rad/s\n', worst_case_state(5));
    fprintf('  r     = %8.4f rad/s\n', worst_case_state(6));
    fprintf('Constraint value at worst case: x''Px = %.6f\n', worst_case_state' * P * worst_case_state);
    fprintf('This bound represents max deviation from linearized attitude dynamics\n');
    fprintf('within the ellipsoidal constraint region\n');
    fprintf('===============================================\n\n');
end

function starting_points = generateStartingPoints(P, n_points)
    % Generate diverse starting points within the ellipsoid
    starting_points = zeros(6, n_points);
    
    for i = 1:n_points
        % Generate random point in unit sphere
        u = randn(6, 1);
        u = u / norm(u);
        
        % Scale by random radius
        r = rand()^(1/6) * 0.8;  % Stay within 80% of boundary
        u = r * u;
        
        % Transform to ellipsoid
        try
            L = chol(P, 'lower');
            starting_points(:, i) = L \ u;
        catch
            [V, D] = eig(P);
            starting_points(:, i) = V * sqrt(D) \ u;
        end
    end
end

function [c, ceq] = ellipsoidConstraint(x, P)
    % Nonlinear constraint: x'*P*x <= 1
    c = x' * P * x - 1;  % <= 0 form required by fmincon
    ceq = [];  % No equality constraints
end

function delta = evaluateLinearizationErrorScalar(x, J, J_inv)
    % Helper function that returns scalar linearization error
    [delta, ~, ~] = evaluateLinearizationError(x, J, J_inv);
end

function [delta, delta_1_norm, delta_2_norm] = evaluateLinearizationError(x, J, J_inv)
    % Helper function to evaluate linearization error at a given state
    
    phi = x(1);
    theta = x(2);
    % psi = x(3);
    p = x(4);
    q = x(5);
    r = x(6);
    
    omega = [p; q; r];
    
    % Compute W(eta) matrix
    c_phi = cos(phi);
    s_phi = sin(phi);
    c_theta = cos(theta);
    % s_theta = sin(theta);
    t_theta = tan(theta);
    
    % Avoid singularities at theta = ±π/2
    if abs(c_theta) < 1e-6
        delta = 0;
        delta_1_norm = 0;
        delta_2_norm = 0;
        return;
    end
    
    W = [1, s_phi*t_theta, c_phi*t_theta;
         0, c_phi, -s_phi;
         0, s_phi/c_theta, c_phi/c_theta];
    
    I3 = eye(3);
    
    % Compute first component of linearization error: (W(eta) - I_3)*omega
    delta_1_vec = (W - I3) * omega;
    delta_1_norm = norm(delta_1_vec);
    
    % Compute second component: -J^(-1)*(omega × J*omega)
    J_omega = J * omega;
    omega_cross_J_omega = cross(omega, J_omega);
    delta_2_vec = -J_inv * omega_cross_J_omega;
    delta_2_norm = norm(delta_2_vec);
    
    % Combined error
    delta = sqrt(delta_1_norm^2 + delta_2_norm^2);
end