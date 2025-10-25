function [A, B] = linearizeModel(F, G, x_eq, u_eq, params)
% Linearize a nonlinear model of the form dx/dt = F(x) + G(x)u
% around equilibrium point (x_eq, u_eq)
%
% Inputs:
%   F: Function handle for F(x), e.g., @(x) [x(2); sin(x(1))]
%   G: Function handle for G(x), e.g., @(x) [0; 1]
%   x_eq: Equilibrium state vector
%   u_eq: Equilibrium input
%
% Outputs:
%   A: State matrix of linearized model
%   B: Input matrix of linearized model

% Get state dimension
n = length(x_eq);
% Get input dimension
% m = length(u_eq);

% Initialize matrices
A = zeros(n, n);
% B = zeros(n, m);

% Small perturbation for numerical differentiation
h = 1e-3;

% Calculate A matrix (∂F/∂x + ∂G/∂x * u_eq)
for i = 1:n
    x_perturbed = x_eq;
    x_perturbed(i) = x_eq(i) + h;

    % Calculate partial derivative of F with respect to x_i
    dF = (F(x_perturbed,params) - F(x_eq,params)) / h;

    % Calculate partial derivative of G*u with respect to x_i
    dG = (G(x_perturbed,params) - G(x_eq,params)) / h;
    A(:,i) = dF + dG * u_eq;
end

% Calculate B matrix (G(x_eq))
B = G(x_eq,params);

% Verify equilibrium condition
dx_eq = F(x_eq,params) + G(x_eq,params)*u_eq;
if norm(dx_eq) > 1e-6
    warning('Specified point might not be an equilibrium: norm(dx_eq) = %e', norm(dx_eq));
end
end