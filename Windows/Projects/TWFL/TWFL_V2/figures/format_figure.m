fig = gcf;                    % figura activa
theme(fig,"light")            % tema claro (UI)
set(fig,'Color','w')          % fondo blanco de la figura

% --- Ejes ---
ax = findall(fig,'Type','axes');
for k = 1:numel(ax)
    set(ax(k), ...
        'Color','w', ...
        'XColor','k','YColor','k','ZColor','k', ...
        'LineWidth',1.5, ...
        'FontSize',24)

    % --- Labels: solo si están vacíos ---
    if isempty(ax(k).XLabel.String)
        xlabel(ax(k),'Time (sec)','Color','k','FontSize',24)
    else
        set(ax(k).XLabel,'Color','k','FontSize',24)
    end

    if isempty(ax(k).YLabel.String)
        ylabel(ax(k),'Y','Color','k','FontSize',24)
    else
        set(ax(k).YLabel,'Color','k','FontSize',24)
    end

    if isempty(ax(k).ZLabel.String)
        zlabel(ax(k),'Z','Color','k','FontSize',24)
    else
        set(ax(k).ZLabel,'Color','k','FontSize',16)
    end

    % --- Título ---
    if ~isempty(ax(k).Title.String)
        set(ax(k).Title, ...
            'Color','k', ...
            'FontSize',18, ...
            'FontWeight','bold')
    end
end

% --- Líneas ---
ln = findall(fig,'Type','line');
set(ln,'LineWidth',3)

% --- Legend: crear si no existe ---
leg = findall(fig,'Type','legend');
if isempty(leg) && ~isempty(ln)
    % Asegurar DisplayName
    for i = 1:numel(ln)
        if isempty(ln(i).DisplayName)
            ln(i).DisplayName = sprintf('Line %d', i);
        end
    end
    leg = legend(ax(1),'show');
end

% --- Formato de la legend ---
for k = 1:numel(leg)
    set(leg(k), ...
        'TextColor','k', ...
        'Color','w', ...
        'EdgeColor','k', ...
        'FontSize',13, ...
        'Box','on')
end

% --- Exportación correcta ---
set(fig,'InvertHardcopy','off')
