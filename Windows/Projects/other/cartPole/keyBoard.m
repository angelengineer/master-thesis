classdef keyBoard < matlab.System
    % KeyRightReader System object for Simulink
    % Outputs 1 when the Right Arrow key is detected, otherwise 0.

    properties (Access = private)
        hFig      % handle to the figure used to capture key presses
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Create a hidden figure or visible (visible is easier for focusing)
            obj.hFig = figure('Name','KeyRightReader - focus this window',...
                              'NumberTitle','off', ...
                              'MenuBar','none', ...
                              'ToolBar','none', ...
                              'KeyPressFcn',@(src,event) setappdata(src,'lastKey',event.Key));
            % initialize stored key
            setappdata(obj.hFig,'lastKey','');
        end

        function y = stepImpl(obj)
            % Read lastKey from the figure appdata
            if isempty(obj.hFig) || ~isvalid(obj.hFig)
                y = 0;
                return
            end

            k = getappdata(obj.hFig,'lastKey');
            % Some MATLAB versions return 'rightarrow' for right arrow key
            % check both 'rightarrow' and 'rightarrow' just in case
            if ischar(k) && (strcmp(k,'rightarrow') || strcmp(k,'rightarrow'))
                y = 1;
            else
                y = 0;
            end
            % Optionally clear lastKey so it only shows once per press:
            setappdata(obj.hFig,'lastKey','');
        end

        function resetImpl(~)
            % nothing required for reset
        end

        function releaseImpl(obj)
            % Close the figure when simulation stops or System object is released
            if ~isempty(obj.hFig) && isvalid(obj.hFig)
                try
                    close(obj.hFig)
                catch
                end
            end
            obj.hFig = [];
        end
    end
end
