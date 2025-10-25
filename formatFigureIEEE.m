% filepath: /Users/rezapordal/Desktop/Phavex/Robust_Safety_Filter/utils/formatFigureIEEE.m
function formatFigureIEEE(fig)
    if nargin < 1
        fig = gcf;
    end
    
    % Set figure size to IEEE column width (3.5 inches)
    fig.Units = 'inches';
    % fig.Position(3:4) = [3.5 2.5]; % width=3.5", height=2.5"
    
    % Get all axes in the figure
    ax = findall(fig, 'type', 'axes');
    
    for i = 1:length(ax)
        % Set font family
        set(ax(i), 'FontName', 'Times New Roman');
        
        % Set font sizes
        set(ax(i), 'FontSize', 8); % Base font size
        
        % Set title font size and interpreter
        if ~isempty(ax(i).Title)
            set(ax(i).Title, 'FontSize', 8, 'Interpreter', 'latex');
        end
        
        % Set label font sizes and interpreters
        set(ax(i).XLabel, 'FontSize', 8, 'Interpreter', 'latex');
        set(ax(i).YLabel, 'FontSize', 8, 'Interpreter', 'latex');
        
        % Set tick label interpreter
        set(ax(i), 'TickLabelInterpreter', 'latex');
        
        % Set line widths
        set(ax(i), 'LineWidth', 1);
        lines = findall(ax(i), 'Type', 'Line');
        for j = 1:length(lines)
            set(lines(j), 'LineWidth', 1);
        end
        
        % Set grid properties
        set(ax(i), 'GridLineStyle', ':');
        set(ax(i), 'GridAlpha', 0.25);
        
        % Set box properties
        set(ax(i), 'Box', 'off');
        
        % Set tick label properties
        set(ax(i), 'TickDir', 'out');
        set(ax(i), 'TickLength', [0.02 0.02]);
    end
    
    % Set background to white
    set(fig, 'Color', 'white');
end