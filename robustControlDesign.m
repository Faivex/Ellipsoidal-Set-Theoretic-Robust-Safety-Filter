function [K, P] = robustControlDesign(A, B, D, u_max, x_max)
    % A unified approach to robust control desgin
    % System with unknown but bounded disturbance
    % x_dot = A*x + B*u + D*w, ||w|| <= 1
    % state = [x; y; z; vx; vy; vz; phi; theta; psi; p; q; r]
    % Solve this LMI:
    % AQ + QA' + alpha*Q + B*Y + (BY)' + 1/alpha*D*D' <= 0, Q>0 , Y = K*P, alpha>0
    % for bounded input we add:
    % [Q Y'; Y u_max^2*I] >= 0, u_max>0
    % Define variables
    n = size(A,1);
    m = size(B,2); %#ok<NASGU>
    tolQmin = 0.01;
    cvx_solver mosek
    cvx_begin sdp quiet
        cvx_precision high
        variable Q(n,n) symmetric
        variable Y(m,n)
        Q >= tolQmin*eye(n);    % force PD (tiny margin)
        % LMI
        % AQ + QA' + alpha*Q + B*Y + (BY)' + 1/alpha*D*D' <= 0, Q>0 , Y = K*P, alpha>0
        alpha = 1.5;
        M = A*Q + Q*A' + alpha*Q + B*Y + (B*Y)' + (1/alpha)*(D*D');
        M <= 0;
        % Bounded input constraint
        % [Q, Y'; Y, u_max^2*eye(m)] >= 0
        for i = 1:size(B,2)  % Loop through input dimensions
            ei = zeros(size(B,2),1);
            ei(i) = 1;
            [Q, Y'*ei; ei'*Y, u_max(i)^2] >= 0;
        end
        % Bounded output constraint
        % Bounded output constraint for each state dimension
        for i = 1:length(x_max)
            ei = zeros(n,1);
            ei(i) = 1;
            [Q, Q'*ei; ei'*Q, x_max(i)^2] >= 0; %#ok<*VUNUS>
        end
        % Maximize the volume of the ellipsoid x'Px<=1
        % maximize (log_det(Q))
        maximize (trace(Q))
    cvx_end
    if strcmp(cvx_status,'Solved') || strcmp(cvx_status,'Inaccurate/Solved')
        P = inv(Q);
        % disp('Q (unified) = ');
        % disp(Q);
        % disp('Y = ');
        % disp(Y);
        K = Y/Q; % since P = inv(Q)
        % disp('K_unified = ');
        % disp(K);
    else
        error('LMI problem (unified) not solved');
    end
end