function plotResults(time, x_safe, x_unsafe, u_safe, u_unsafe, w, w_max, P, alpha_fun, ...
                    u_max, x_max, pos_des, disturbance_plot_off)
    % Plot the results of the quadrotor simulation
    % time: time vector
    % x_safe: state trajectory with safety filter
    % x_unsafe: state trajectory without safety filter
    % u_safe: control inputs with safety filter
    % u_unsafe: control inputs without safety filter
    % w: disturbance function handle
    % w_max: maximum disturbance values
    % P: ellipoid matrix
    % alpha_fun: safety filter blending function handle
    % u_max: maximum control input values
    % x_max: maximum state values for orientation (phi, theta)
    % pos_des: desired position [x_des; y_des]
    % savePath: path to save the figures

    % Define color palette
    c1 = [0, 0.447, 0.741];    % Blue
    c2 = [0.85, 0.325, 0.098]; % Red
    c3 = [0.929, 0.694, 0.125]; % Yellow/Gold
    c4 = [0.494, 0.184, 0.556]; % Purple
    c5 = [0.466, 0.674, 0.188]; % Green
    limit_color = [0.3 0.3 0.3]; % Dark gray for limits

    % Unpack state variables for plotting
    pos_safe = x_safe(1:3, :);
    pos_unsafe = x_unsafe(1:3, :);
    angles_safe = x_safe(7:9, :);
    angles_unsafe = x_unsafe(7:9, :);

    % valid_idx: indices where index>unstablity_index
    % if contains(savePath, 'III')
    %     t_unsatable = 10; % seconds
    % else
    t_unsatable = time(end); % seconds
    % end
    unstablity_index = find(time >= t_unsatable, 1);
    valid_idx = 1:unstablity_index;

    % valid_idx = find_valid_indices(x_unsafe, u_unsafe, x_max, u_max, inf);

    % Create single figure for all plots
    fig = figure;
    fig.Units = 'inches';
    
    % Current 2x2 Configuration
    fig.Position(3:4) = [7 2.5]; 
    d_h = 0.25; % horizontal distance between subplots
    s_h = 0.18; % subplot width

    % Position subplot (top-left)
    % subplot('Position', [0.08, 0.60, 0.40, 0.35]); 
    subplot('Position', [0.07, 0.20, s_h, 0.70]);  % Position
        % Check if pos_des is a single point or trajectory
    if size(pos_des, 2) == 1
        hold on;
        plot(time(valid_idx), pos_unsafe(1,valid_idx), ':', 'Color', c1, 'HandleVisibility', 'off','Linewidth',0.5);
        plot(time(valid_idx), pos_unsafe(2,valid_idx), ':', 'Color', c2, 'HandleVisibility', 'off','Linewidth',0.5);
        plot(time, pos_des(1)*ones(size(time)), '-.', 'Color', limit_color, 'DisplayName', 'Setpoint');
        plot(time, pos_des(2)*ones(size(time)), '-.', 'Color', limit_color, 'HandleVisibility', 'off');
        plot(time, pos_safe(1,:), '-', 'Color', c1, 'DisplayName', 'x');
        plot(time, pos_safe(2,:), '-', 'Color', c2, 'DisplayName', 'y');
        xlabel('Time (s)');
        ylim([0, 2*max(pos_des)]);  % Set y limits
        xlim([time(1) time(end)]);
        xticks(linspace(0, time(end), 6));
        ylabel('Position (m)');
        lgd = legend('Location', 'southoutside', 'Orientation', 'horizontal', ...
            'Interpreter', 'latex', 'Box', 'off', 'NumColumns', 2);
    else
        % if pos_des is a trajectory, plot x-y graph
        hold on;
        plot(pos_unsafe(1,valid_idx), pos_unsafe(2,valid_idx), ':', 'Color', c1, 'HandleVisibility', 'off');
        plot(pos_des(1,:), pos_des(2,:), '-', 'Color', limit_color, 'DisplayName', 'Desired Traj.');
        plot(pos_safe(1,:), pos_safe(2,:), '-', 'Color', c1, 'DisplayName', 'Traj.');
        xlabel('X Position (m)');
        ylabel('Y Position (m)');
        axis equal;
        xlim(2*[min(pos_des(1,:)), max(pos_des(1,:))]);
        ylim(2*[min(pos_des(2,:)), max(pos_des(2,:))]);
        lgd = legend('Location', 'southoutside', 'Orientation', 'horizontal', ...
            'Interpreter', 'latex', 'Box', 'off', 'NumColumns', 1);
    end

    lgd.ItemTokenSize = [15, 18];
    grid on;

    % Orientation subplot (top-right)
    % subplot('Position', [0.55, 0.60, 0.40, 0.35]);  
    subplot('Position', [0.07 + d_h, 0.20, s_h, 0.70]);  % Orientation
    hold on;
    plot(time(valid_idx), angles_unsafe(1,valid_idx)*180/pi, ':', 'Color', [c1 0.5], 'HandleVisibility', 'off','Linewidth',0.5);
    plot(time(valid_idx), angles_unsafe(2,valid_idx)*180/pi, ':', 'Color', [c2 0.5], 'HandleVisibility', 'off','Linewidth',0.5);
    plot(time, x_max(1)*ones(size(time))*180/pi, '--', 'Color', limit_color, 'DisplayName', 'Constraint');
    plot(time, -x_max(1)*ones(size(time))*180/pi, '--', 'Color', limit_color, 'HandleVisibility', 'off');
    plot(time, angles_safe(1,:)*180/pi, '-', 'Color', c1, 'DisplayName', '$\phi$');
    plot(time, angles_safe(2,:)*180/pi, '-', 'Color', c2, 'DisplayName', '$\theta$');
    xlabel('Time (s)');
    ylabel('Orientation (deg)');
    ylim([-1.55*x_max(1)*180/pi, 1.55*x_max(1)*180/pi]);  % Set y limits
    xlim([time(1) time(end)]);
    xticks(linspace(0, time(end), 6));
    lgd = legend('Location', 'southoutside', 'Orientation', 'horizontal', ...
                'Interpreter', 'latex', 'Box', 'off', 'NumColumns', 2);
    lgd.ItemTokenSize = [20, 18];
    grid on;

    % Control inputs and disturbance subplot (bottom-left)
    % subplot('Position', [0.08, 0.25, 0.40, 0.35]); 
    subplot('Position', [0.07 + 2*d_h, 0.20, s_h, 0.70]);  % Control/Disturbance
    hold on;    
    plot(time(valid_idx), u_unsafe(1,valid_idx), ':', 'Color', [c1 0.5], 'HandleVisibility', 'off', 'Linewidth', 0.5);
    plot(time(valid_idx), u_unsafe(2,valid_idx), ':', 'Color', [c2 0.5], 'HandleVisibility', 'off', 'Linewidth', 0.5);
    plot(time, u_max(1)*ones(size(time)), '--', 'Color', limit_color, 'HandleVisibility', 'off', 'LineWidth', 0.5);
    plot(time, -u_max(1)*ones(size(time)), '--', 'Color', limit_color, 'HandleVisibility', 'off', 'LineWidth', 0.5);
    plot(time, u_safe(1,:), '-', 'Color', c1, 'DisplayName', '$\tau_1$');
    plot(time, u_safe(2,:), '-', 'Color', c2, 'DisplayName', '$\tau_2$');
    
    % Turn off the disturbance plot if savepath contains 'II'
    if disturbance_plot_off
        % Do nothing, skip disturbance plot
        lgd = legend('Location', 'southoutside', 'Orientation', 'horizontal', ...
            'NumColumns', 1, 'Interpreter', 'latex', 'Box', 'off');
    else
        % Plot disturbance
        w_values = zeros(3, length(time));
        for k = 1:length(time)
            w_values(:,k) = w_max.*w(time(k));
        end
            plot(time, w_values(1,:), '-', 'Color', c3, 'DisplayName', '$w_1$');
            plot(time, w_values(2,:), '-', 'Color', c4, 'DisplayName', '$w_2$');
        lgd = legend('Location', 'southoutside', 'Orientation', 'horizontal', ...
            'NumColumns', 2, 'Interpreter', 'latex', 'Box', 'off');
    end
    ylabel('Magnitude (N.m)');
    xlabel('Time (s)');
    ylim([-1.25*u_max(1), 1.25*u_max(1)]);  % Set y limits
    xlim([time(1) time(end)]);
    xticks(linspace(0, time(end), 6));
    lgd.ItemTokenSize = [15, 18];
    grid on;

    % Safety measure and alpha subplot (bottom-right)
    % subplot('Position', [0.55, 0.25, 0.40, 0.35]); 
    subplot('Position', [0.07 + 3*d_h, 0.20, s_h/1.2, 0.70]);  % Safety/Alpha
    V_safe = zeros(1, length(time));
    V_unsafe = zeros(1, length(time));
    alpha_values = zeros(1, length(time));
    for k = 1:length(time)
        V_safe(k) = x_safe(7:12,k)'*P*x_safe(7:12,k);
        V_unsafe(k) = x_unsafe(7:12,k)'*P*x_unsafe(7:12,k);
        alpha_values(k) = alpha_fun(x_safe(7:12,k));
    end
    plot(time, V_safe, '-', 'Color', c4, 'DisplayName', '$h$');
    hold on;
    plot(time(valid_idx), V_unsafe(valid_idx), ':', 'Color', c4, 'HandleVisibility', 'off');
    plot(time, alpha_values, '-', 'Color', c5, 'LineWidth', 1.5, 'DisplayName', '$\alpha$');
    ylabel('Value');
    xlabel('Time (s)');
    ylim([0 1.25]);  % Set y limits
    xlim([time(1) time(end)]);
    xticks(linspace(0, time(end), 6));
    lgd = legend('Location', 'southoutside', 'Orientation', 'horizontal', ...
                'Interpreter', 'latex', 'Box', 'off','NumColumns', 1);
    lgd.ItemTokenSize = [15, 18];
    grid on;

    formatFigureIEEE;
end