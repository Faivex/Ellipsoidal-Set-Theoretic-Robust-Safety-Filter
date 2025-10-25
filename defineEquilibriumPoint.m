function [x_eq, u_eq] = defineEquilibriumPoint
    % Define equilibrium point
    x_eq = zeros(12,1);  % Hover
    
    % Equilibrium control inputs
    u1 = 0;
    u2 = 0;
    u3 = 0;
    
    u_eq = [u1; u2; u3];
end