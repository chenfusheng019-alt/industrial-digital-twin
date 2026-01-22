classdef ArmControlApp < matlab.apps.AppBase
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        ArmAxes                    matlab.ui.control.UIAxes
        StateTable                 matlab.ui.control.Table
        LogArea                    matlab.ui.control.TextArea
        JointSliders
        ActionButtons              matlab.ui.control.Button
        RunButton                  matlab.ui.control.Button
        StopButton                 matlab.ui.control.Button
        ResetButton                matlab.ui.control.Button
        SaveButton                 matlab.ui.control.Button
        LoadButton                 matlab.ui.control.Button
        StatusSwitch               matlab.ui.control.Switch
        StatusTrack                matlab.ui.container.Panel
        StatusKnob                 matlab.ui.control.Button
        StatusIsConnected logical = false
        StatusTrackAx              matlab.ui.control.UIAxes
        StatusTrackPatch           matlab.graphics.primitive.Rectangle
        StatusLabelOff             matlab.ui.control.Label
        StatusLabelOn              matlab.ui.control.Label
    end

    properties (Access = private)
        RLSubscriber
        ActionPublisher
        NetworkPublisher
        UISubscriptionHandle
        ROSInitialized logical = false
        LastState double = zeros(1,4)
        LastAction double = -1
        LastReward double = 0
        Episode double = 1
        EpisodeStep double = 0
        TotalSteps double = 0
        LastSliderValues double = [500 500 500]
        ArmLinks matlab.graphics.chart.primitive.Line
        ArmJoints
        ArmImage matlab.graphics.primitive.Image
        ArmImageData
        ActionNames string = ["关节1+","关节1-","关节2+","关节2-","关节3+","关节3-"]
        JointColors double = [0.6 0.6 0.6; 0 0.6 0; 0 0 1; 1 0.6 0]  % 基座灰, 舵机2绿, 舵机3蓝, 舵机4黄
        StatusSwitchContainer matlab.ui.container.Panel
    end

    methods (Access = private)

        function createComponents(app)
            app.UIFigure = uifigure('Name','Arm Control Center','Position',[100 100 1300 720]);
            gl = uigridlayout(app.UIFigure,[2 3]);
            gl.RowHeight = {'1x',200};
            gl.ColumnWidth = {350,'1x',320};

            app.ArmAxes = uiaxes(gl);
            app.ArmAxes.Layout.Row = [1 2];
            app.ArmAxes.Layout.Column = 1;
            title(app.ArmAxes,'Arm Preview');
            xlabel(app.ArmAxes,'Xposition (mm)');
            ylabel(app.ArmAxes,'Yposition (mm)');
            axis(app.ArmAxes,'equal');
            xlim(app.ArmAxes,[-200 200]);
            ylim(app.ArmAxes,[-50 300]);
            grid(app.ArmAxes,'on');
            app.ArmAxes.GridColor = [0.5 0.5 0.5];
            app.ArmAxes.GridAlpha = 0.3;
            app.ArmAxes.GridLineStyle = '-';
            app.ArmAxes.XColor = [0 0 0];
            app.ArmAxes.YColor = [0 0 0];
            app.ArmAxes.Box = 'on';
            hold(app.ArmAxes,'on');
            
            % 初始化图片数据为空
            app.ArmImageData = [];

            centerPanel = uipanel(gl,'Title','实时数据');
            centerPanel.Layout.Row = 1;
            centerPanel.Layout.Column = 2;
            cg = uigridlayout(centerPanel,[1 1]);
            app.StateTable = uitable(cg,'ColumnName',{'state1','state2','state3','distance','action','reward','done','episode','step'},...
                'Data',zeros(0,9));

            rightPanel = uipanel(gl,'Title','交互控制');
            rightPanel.Layout.Row = [1 2];
            rightPanel.Layout.Column = 3;
            rg = uigridlayout(rightPanel,[5 2]);
            rg.RowHeight = {120,150,120,80,'1x'};

            actionPanel = uipanel(rg,'Title','动作命令');
            actionPanel.Layout.Row = 1;
            actionPanel.Layout.Column = [1 2];
            ag = uigridlayout(actionPanel,[3 2]);
            for i = 1:6
                btn = uibutton(ag,'Text',app.ActionNames(i));
                btn.Layout.Row = ceil(i/2);
                btn.Layout.Column = mod(i-1,2)+1;
                btn.ButtonPushedFcn = @(~,~)sendAction(app,i-1);
                app.ActionButtons(i) = btn;
            end

            sliderPanel = uipanel(rg,'Title','关节调节');
            sliderPanel.Layout.Row = 2;
            sliderPanel.Layout.Column = [1 2];
            sg = uigridlayout(sliderPanel,[3 2]);
            sg.RowHeight = {'1x','1x','1x'};
            sg.ColumnWidth = {60,'1x'};
            app.JointSliders = gobjects(1,3);
            for i = 1:3
                lbl = uilabel(sg,'Text',sprintf('关节%d',i));
                lbl.Layout.Row = i;
                lbl.Layout.Column = 1;
                lbl.HorizontalAlignment = 'left';
                sld = uislider(sg,'Limits',[0 1000],'Value',500);
                sld.Layout.Row = i;
                sld.Layout.Column = 2;
                sld.ValueChangedFcn = @(src,~)jointSliderChanged(app,i,src.Value);
                app.JointSliders(i) = sld;
            end

            controlPanel = uipanel(rg,'Title','运行控制');
            controlPanel.Layout.Row = 3;
            controlPanel.Layout.Column = [1 2];
            cg2 = uigridlayout(controlPanel,[2 3]);
            app.RunButton = uibutton(cg2,'Text','运行','ButtonPushedFcn',@(~,~)app.logMsg('运行'));
            app.StopButton = uibutton(cg2,'Text','停止','ButtonPushedFcn',@(~,~)app.logMsg('停止'));
            app.ResetButton = uibutton(cg2,'Text','重置','ButtonPushedFcn',@(~,~)app.resetEnvironment());
            app.SaveButton = uibutton(cg2,'Text','保存动作','ButtonPushedFcn',@(~,~)app.logMsg('保存当前动作脚本'));
            app.LoadButton = uibutton(cg2,'Text','加载动作','ButtonPushedFcn',@(~,~)app.logMsg('加载动作脚本'));

            bottomPanel = uipanel(gl,'Title','日志');
            bottomPanel.Layout.Row = 2;
            bottomPanel.Layout.Column = 2;
            logGrid = uigridlayout(bottomPanel,[1 1]);
            logGrid.RowHeight = {'1x'};
            logGrid.ColumnWidth = {'1x'};
            app.LogArea = uitextarea(logGrid,'Value',{'系统初始化'});

            statusPanel = uipanel(gl,'Title','状态');
            statusPanel.Layout.Row = 2;
            statusPanel.Layout.Column = 3;
            % 右下角开关：放大到"状态栏约 1/3"并垂直居中
            % 采用 3x3：中间行给足高度，中间列占约 1/3 宽
            stGrid = uigridlayout(statusPanel,[3 3]);
            stGrid.RowHeight = {'1x', 120, '1x'};     % 中间行高度决定开关"大小"
            stGrid.ColumnWidth = {'1x','1x','1x'};  % 中间列约 1/3 宽
            stGrid.Padding = [8 8 8 8];
            stGrid.RowSpacing = 0;
            stGrid.ColumnSpacing = 0;

            app.StatusSwitchContainer = uipanel(stGrid);
            app.StatusSwitchContainer.Layout.Row = 2;
            app.StatusSwitchContainer.Layout.Column = 2;
            app.StatusSwitchContainer.BorderType = 'none';
            % 设置为与窗体相同的背景色，避免出现灰色方块（只显示椭圆轨道）
            app.StatusSwitchContainer.BackgroundColor = app.UIFigure.Color;

            cgStatus = uigridlayout(app.StatusSwitchContainer,[1 1]);
            cgStatus.RowHeight = {'1x'};
            cgStatus.ColumnWidth = {'1x'};
            cgStatus.Padding = [12 12 12 12]; % 给开关留白，让整体更"厚"
            cgStatus.RowSpacing = 0;
            cgStatus.ColumnSpacing = 0;

            app.StatusSwitch = uiswitch(cgStatus,'Items',{'OFF','ON'},'Value','OFF');
            app.StatusSwitch.FontSize = 18;   % 放大开关显示
            app.StatusSwitch.FontWeight = 'bold';
            app.StatusSwitch.FontColor = [0 0 0]; % ON/OFF 文字保持黑色
            app.StatusSwitch.Enable = 'off'; % keep hidden/disabled, use custom visible control below

            % --- 创建自定义可见滑动开关（确保在 .m 运行时可见） ---
            % 轨道（灰/绿背景）
            % 使用像素坐标创建自定义可见滑动开关（更兼容旧版 MATLAB .m 运行）
            % enlarge track panel and knob for better visibility
            % make panel background match the figure so only the ellipse patch is visible
            % Create a track panel that fills the container (use normalized to avoid clipping)
            app.StatusTrack = uipanel(app.StatusSwitchContainer, ...
                'Units','normalized','Position',[0 0 1 1], ...
                'BorderType','none', 'BackgroundColor',app.UIFigure.Color);

            % 创建用于绘制轨道的隐藏坐标轴（绘制圆角矩形作为轨道），使用规范坐标使补丁占满axes
            app.StatusTrackAx = uiaxes(app.StatusTrack,'Units','normalized','Position',[0 0 1 1]);
            axis(app.StatusTrackAx,'off');
            app.StatusTrackAx.Color = 'none';
            % 轨道（圆角矩形）——使用 [0 0 1 1] 数据坐标填满axes
            xlim(app.StatusTrackAx,[0 1]);
            ylim(app.StatusTrackAx,[0 1]);
            app.StatusTrackPatch = rectangle(app.StatusTrackAx,'Position',[0 0 1 1], ...
                'Curvature',[1 1],'FaceColor',[0.7 0.7 0.7],'EdgeColor','none','FaceAlpha',1);

            % 左右文本标签（像素位置）——创建并保存句柄以便后面控制和置顶
            app.StatusLabelOff = uilabel(app.StatusTrack,'Text','OFF', ...
                'Position',[12 44 48 32],'HorizontalAlignment','left','FontWeight','bold','BackgroundColor','none');
            app.StatusLabelOn = uilabel(app.StatusTrack,'Text','ON', ...
                'Position',[220 44 40 32],'HorizontalAlignment','right','FontWeight','bold','BackgroundColor','none');

            % 滑块（白色圆形视觉：使用按钮作为可点击控件，初始在左侧）
            % 滑块（白色圆形视觉：使用按钮作为可点击控件，初始在左侧）
            % Compute pixel positions for labels and knob based on actual pixel size
            drawnow; % ensure layout finalized
            try
                trackPix = app.StatusTrack.Position;
                w = trackPix(3); h = trackPix(4);
                lblOffPos = [round(w*0.08) round(h*0.36) 48 24];
                lblOnPos = [round(w*0.78) round(h*0.36) 40 24];
                knobPosLeft = [round(w*0.08) round(h*0.06) round(h*0.88) round(h*0.88)];
            catch
                lblOffPos = [12 44 48 32];
                lblOnPos = [220 44 40 32];
                knobPosLeft = [20 22 96 96];
            end
            app.StatusLabelOff.Position = lblOffPos;
            app.StatusLabelOn.Position = lblOnPos;
            app.StatusKnob = uibutton(app.StatusTrack,'Text','', ...
                'Position',knobPosLeft, ...
                'BackgroundColor',[1 1 1], 'ButtonPushedFcn',@(~,~)app.toggleCustomSwitch());
            app.StatusKnob.FontSize = 1; % 隐藏文本显示
            % 使滑块与标签位于最上层，避免被补丁或其他控件覆盖
            uistack(app.StatusKnob,'top');
            uistack(app.StatusLabelOff,'top');
            uistack(app.StatusLabelOn,'top');
        end

        function initializeROS(app)
            try
                if ~app.isROSNodeActive()
                    rosinit('192.168.110.46');
                else
                    app.logMsg('检测到已有ROS节点，直接复用当前连接');
                end
                app.ActionPublisher = rospublisher('/rl_action','std_msgs/Int32');
                app.NetworkPublisher = rospublisher('/network_update','std_msgs/Float32MultiArray');
                app.RLSubscriber = rossubscriber('/rl_data','std_msgs/Float32MultiArray',@(~,msg)app.handleRLData(msg),'BufferSize',5);
                app.ROSInitialized = true;
                app.updateStatusSwitch(true);
                app.logMsg('ROS连接成功');
            catch ME
                app.logMsg(['ROS初始化失败: ' ME.message]);
                app.updateStatusSwitch(false);
            end
        end

        function active = isROSNodeActive(~)
            try
                rosnode('list');
                active = true;
            catch
                active = false;
            end
        end

        function handleRLData(app,msg)
            data = double(msg.Data(:)');
            if numel(data) < 9
                app.logMsg('收到数据不完整');
                return;
            end

            state = data(1:4);
            action = data(5);
            reward = data(6);
            done = logical(data(7));
            episode = data(8);
            step = data(9);

            app.LastState = state;
            app.LastAction = action;
            app.LastReward = reward;
            app.Episode = episode;
            app.EpisodeStep = step;

            app.updateStateTable(state,action,reward,done,episode,step);
            app.updateArmDrawing(state);
            app.updateSliders(state(1:3));
            
            % 推送数据到后端（用于网页显示）
            try
                mode = 'TRAINING';
                if done
                    mode = 'IDLE';
                end
                push_to_backend(state, reward, mode, episode, step);
                app.logMsg('✓ 数据已推送到后端');
            catch ME
                app.logMsg(['✗ 后端推送失败: ' ME.message]);
            end
            
            % 推送图像到后端（每5步推送一次，避免过于频繁）
            if mod(step, 5) == 0
                try
                    % 使用 exportgraphics 导出 uiaxes 到临时文件，再读取上传，避免 getframe 在 uiaxes 上不稳定的问题
                    script_dir = fileparts(mfilename('fullpath'));
                    temp_file = fullfile(script_dir, 'temp_image.jpg');
                    try
                        exportgraphics(app.ArmAxes, temp_file, 'Resolution', 150);
                    catch exportErr
                        % exportgraphics 在某些 UI 布局下可能失败（多个容器），此时采用备选方案：
                        % 在新的不可见图窗中复制 axes 内容并导出
                        try
                            tmpFig = figure('Visible','off','Renderer','painters');
                            tmpAx = axes(tmpFig);
                            % 复制子对象到新 axes（保持可视化）
                            copyobj(allchild(app.ArmAxes), tmpAx);
                            % 复制轴属性（limits, aspect）
                            tmpAx.XLim = app.ArmAxes.XLim;
                            tmpAx.YLim = app.ArmAxes.YLim;
                            tmpAx.NextPlot = 'add';
                            tmpAx.Box = app.ArmAxes.Box;
                            exportgraphics(tmpAx, temp_file, 'Resolution', 150);
                            close(tmpFig);
                        catch innerErr
                            % 如果备选也失败，抛出原始错误以记录日志
                            rethrow(exportErr);
                        end
                    end

                    img = imread(temp_file);
                    push_image_to_backend(img);
                    % 删除临时文件（如果存在）
                    if exist(temp_file, 'file')
                        delete(temp_file);
                    end
                    app.logMsg('✓ 图像已推送到后端');
                catch ME
                    app.logMsg(['✗ 图像推送失败: ' ME.message]);
                end
            end

            if action ~= -1
                app.TotalSteps = app.TotalSteps + 1;
                dataStruct = struct('state',state,'action',action,'reward',reward,...
                    'done',done,'episode',episode,'episode_step',step);
                try
                    [nextAction, weights, shouldUpdate] = call_python_dqn(dataStruct, app.TotalSteps); %#ok<NASGU>
                    app.logMsg(['下一个动作: ' num2str(nextAction)]);
                    app.sendAction(nextAction);
                    if shouldUpdate
                        msgWeights = rosmessage('std_msgs/Float32MultiArray');
                        msgWeights.Data = double(weights);
                        send(app.NetworkPublisher,msgWeights);
                        app.logMsg('已发送网络更新');
                    end
                catch ME
                    app.logMsg(['调用Python失败: ' ME.message]);
                end
            end

            if done
                app.logMsg(sprintf('Episode %d 结束，步骤 %d',episode,step));
            end
        end

        function updateStateTable(app,state,action,reward,done,episode,step)
            newRow = [state,action,reward,double(done),episode,step];
            data = app.StateTable.Data;
            data = [data; newRow];
            if size(data,1) > 200
                data = data(end-199:end,:);
            end
            app.StateTable.Data = data;
        end

        function updateArmDrawing(app,state)
            % 每次新数据到来：直接清空坐标系，确保不残留“历史曲线/文字/节点”
            cla(app.ArmAxes);
            title(app.ArmAxes,'Arm Preview');
            xlabel(app.ArmAxes,'Xposition (mm)');
            ylabel(app.ArmAxes,'Yposition (mm)');
            axis(app.ArmAxes,'equal');
            % 将视图左侧留更多空间，使连杆向左展开，基座位于右下角
            xlim(app.ArmAxes,[-300 100]);
            ylim(app.ArmAxes,[-50 300]);
            grid(app.ArmAxes,'on');
            app.ArmAxes.GridColor = [0.5 0.5 0.5];
            app.ArmAxes.GridAlpha = 0.3;
            app.ArmAxes.GridLineStyle = '-';
            app.ArmAxes.XColor = [0 0 0];
            app.ArmAxes.YColor = [0 0 0];
            app.ArmAxes.Box = 'on';
            app.ArmAxes.NextPlot = 'add'; % 等价于 hold on，更稳定
            hold(app.ArmAxes,'on');

            % 按四节点连杆绘制（不再使用图片）
            app.drawArmLines(state);
            drawnow limitrate;
        end
        
        function drawArmImage(app, state)
            % 使用图片绘制机械臂，根据关节角度进行旋转和定位
            % 检查图片数据是否存在
            if isempty(app.ArmImageData)
                app.drawArmLines(state);
                return;
            end
            
            angles = deg2rad(state(1:3));  % 转换为弧度
            lengths = [80 80 80];  % 三段连杆长度（mm）
            
            % 计算各关节位置
            x = zeros(1, 4);
            y = zeros(1, 4);
            x(1) = 0; y(1) = 0;  % 基座位置
            
            current_angle = 0;
            for i = 1:3
                current_angle = current_angle + angles(i);
                x(i+1) = x(i) + lengths(i) * cos(current_angle);
                y(i+1) = y(i) + lengths(i) * sin(current_angle);
            end
            
            % 获取图片尺寸
            [imgH, imgW, imgC] = size(app.ArmImageData);
            
            % 计算机械臂总长度（用于缩放）
            total_length = sum(lengths);
            % 根据图片尺寸和实际长度计算缩放因子
            img_diagonal = sqrt(imgW^2 + imgH^2);
            scale_factor = total_length / img_diagonal * 1.2;  % 缩放因子，稍微放大
            
            % 计算机械臂总角度（从基座到末端的累积角度）
            total_angle = sum(angles);
            
            % 旋转图片（以图片中心为旋转中心）
            rotated_img = imrotate(app.ArmImageData, rad2deg(total_angle), 'bilinear', 'loose');
            [rotH, rotW, ~] = size(rotated_img);
            
            % 计算旋转后图片的显示尺寸（在坐标系中的mm单位）
            display_width = rotW * scale_factor;
            display_height = rotH * scale_factor;
            
            % 图片基座位置（原点）
            base_x = x(1);
            base_y = y(1);
            
            % 计算图片显示范围（图片底部中心对齐到基座）
            % 假设图片中基座在底部中心
            x_range = [base_x - display_width/2, base_x + display_width/2];
            y_range = [base_y, base_y + display_height];
            
            % 显示图片
            app.ArmImage = image(app.ArmAxes, x_range, y_range, rotated_img);
            app.ArmImage.AlphaData = 0.95;  % 设置透明度，稍微透明以便看到网格
            
            % 绘制关节位置标记（4个不同颜色的圆点）
            % 颜色顺序：红、绿、蓝、黄
            joint_colors = [1 0 0; 0 1 0; 0 0 1; 1 1 0];  % 红、绿、蓝、黄
            app.ArmJoints = gobjects(1, 4);
            for i = 1:4
                app.ArmJoints(i) = plot(app.ArmAxes, x(i), y(i), 'o', ...
                    'MarkerSize', 10, 'MarkerFaceColor', joint_colors(i, :), ...
                    'MarkerEdgeColor', [0 0 0], 'LineWidth', 2);
            end
        end
        
        function drawArmLines(app, state)
            % 使用线条绘制四节点机械臂：基座在右下角，连杆向左延伸
            lengths = [100 90 80];  % 三段连杆长度（mm），第一段稍长以便视觉效果

            % state(1:3) 为舵机位置(0~1000)，映射到角度（-120~120度），更接近你之前的版本
            angles_deg = (state(1:3) - 500) / 500 * 120;
            angles = deg2rad(angles_deg);

            % 基座位置（右下角）
            % 把基座放在坐标系右下角（接近原点的右侧）
            base_x = 0;
            base_y = 0;

            % 计算各关节位置（4个节点：舵机1~4）
            x = zeros(1, 4);
            y = zeros(1, 4);
            x(1) = base_x;
            y(1) = base_y;

            % 初始朝向向左（pi），让机械臂从右向左展开
            current_angle = pi;
            for i = 1:3
                current_angle = current_angle + angles(i);
                x(i+1) = x(i) + lengths(i) * cos(current_angle);
                y(i+1) = y(i) + lengths(i) * sin(current_angle);
            end

            % 绘制连杆（四节点连接），先画线再画节点以保证节点在上层
            app.ArmLinks = plot(app.ArmAxes, x, y, '-', ...
                'LineWidth', 3, 'Color', [0.12 0.12 0.12]);

            % 绘制关节节点（4个不同颜色）——使用更明显的实心标记并放到线之上
            joint_colors = app.JointColors;
            app.ArmJoints = gobjects(1, 4);
            
            % 基座（节点1）：使用方块，灰色填充，较大尺寸
            hold(app.ArmAxes,'on');
            app.ArmJoints(1) = plot(app.ArmAxes, x(1), y(1), 's', ...
                'MarkerSize', 20, ...
                'MarkerFaceColor', joint_colors(1, :), ...
                'MarkerEdgeColor', [0 0 0], ...
                'LineWidth', 1.5);
            
            % 其余三个节点：使用圆圈，不同颜色，较大尺寸
            for i = 2:4
                app.ArmJoints(i) = plot(app.ArmAxes, x(i), y(i), 'o', ...
                    'MarkerSize', 14, ...
                    'MarkerFaceColor', joint_colors(i, :), ...
                    'MarkerEdgeColor', [0 0 0], ...
                    'LineWidth', 1.5);
            end

            % 节点名称与坐标标签（每个节点单独显示，坐标随数据变化）
            name_offsets = [  0  24;   0  24;   0  24;   0  24];  % 名称统一在上方
            coord_offsets = [  0 -26;   0 -26;   0 -26;   0 -26]; % 坐标统一在下方
            for i = 1:4
                % 名称：舵机1~4
                text(app.ArmAxes, x(i)+name_offsets(i,1), y(i)+name_offsets(i,2), ...
                    sprintf('舵机%d', i), ...
                    'HorizontalAlignment','center', ...
                    'FontWeight','bold', ...
                    'FontSize',11, ...
                    'Color',[0 0 0], ...
                    'BackgroundColor',[1 1 1], ...
                    'Margin',2, ...
                    'Clipping','on','Units','data');

                % 坐标：(x,y) —— 显示到 1 位小数，并去掉整数的 .0
                xs = sprintf('%.1f', x(i));
                ys = sprintf('%.1f', y(i));
                coordStr = sprintf('(%s, %s)', xs, ys);
                text(app.ArmAxes, x(i)+coord_offsets(i,1), y(i)+coord_offsets(i,2), ...
                    coordStr, ...
                    'HorizontalAlignment','center', ...
                    'FontSize',10, ...
                    'Color',[0 0 0], ...
                    'BackgroundColor',[1 1 1], ...
                    'Margin',2, ...
                    'Clipping','on','Units','data');
            end
        end
        
        function updateStatusSwitch(app, isConnected)
            % 更新状态开关：连接成功时ON（绿底），失败时OFF（灰底）
            % 保证属性存在
            app.StatusIsConnected = logical(isConnected);
            % 不修改容器背景，所有可见效果由补丁与标签控制
            if ~isempty(app.StatusSwitchContainer) && isvalid(app.StatusSwitchContainer)
                % keep container background unchanged
            end

            % 更新自定义轨道与滑块显示（确保在 .m 运行时可见）
            try
                if ~isempty(app.StatusTrack) && isvalid(app.StatusTrack)
            if isConnected
                        % 轨道补丁设绿，滑块移右
                        if ~isempty(app.StatusTrackPatch) && isvalid(app.StatusTrackPatch)
                            app.StatusTrackPatch.FaceColor = [0.2 0.8 0.2];
                        else
                            app.StatusTrack.BackgroundColor = [0.2 0.8 0.2];
                        end
                        app.StatusKnob.Position = [152 22 96 96];
                        % highlight ON label
                        if ~isempty(app.StatusLabelOn) && isvalid(app.StatusLabelOn)
                            app.StatusLabelOn.FontColor = [0 0.2 0];
                            app.StatusLabelOn.FontWeight = 'bold';
                        end
                        if ~isempty(app.StatusLabelOff) && isvalid(app.StatusLabelOff)
                            app.StatusLabelOff.FontColor = [0.4 0.4 0.4];
                            app.StatusLabelOff.FontWeight = 'normal';
                end
            else
                        if ~isempty(app.StatusTrackPatch) && isvalid(app.StatusTrackPatch)
                            app.StatusTrackPatch.FaceColor = [0.7 0.7 0.7];
                        else
                            app.StatusTrack.BackgroundColor = [0.7 0.7 0.7];
                        end
                        app.StatusKnob.Position = [20 22 96 96];
                        % highlight OFF label
                        if ~isempty(app.StatusLabelOn) && isvalid(app.StatusLabelOn)
                            app.StatusLabelOn.FontColor = [0.4 0.4 0.4];
                            app.StatusLabelOn.FontWeight = 'normal';
                        end
                        if ~isempty(app.StatusLabelOff) && isvalid(app.StatusLabelOff)
                            app.StatusLabelOff.FontColor = [0 0 0];
                            app.StatusLabelOff.FontWeight = 'bold';
                        end
                    end
                end
            catch
                % 忽略显示更新错误
            end
        end
        
        function animateSwitch(app, isConnected)
            % 开关动画效果：模拟从左到右滑动，颜色变化
            try
                % 先设置为白色（中间状态）
                app.StatusSwitch.FontColor = [0 0 0];
                drawnow;
                pause(0.1);  % 短暂延迟模拟动画
                
                if isConnected
                    % ON状态：从左到右，白变绿
                    app.StatusSwitch.Value = 'ON';
                    app.StatusSwitch.FontColor = [0 0 0];
                else
                    % OFF状态：从左到右
                    app.StatusSwitch.Value = 'OFF';
                    app.StatusSwitch.FontColor = [0 0 0];
                end
                drawnow;
            catch
                % 如果动画失败，至少设置颜色和值
                if isConnected
                    app.StatusSwitch.Value = 'ON';
                    app.StatusSwitch.FontColor = [0 0 0];
                else
                    app.StatusSwitch.Value = 'OFF';
                    app.StatusSwitch.FontColor = [0 0 0];
                end

            end
        end

        function updateSliders(app,jointValues)
            for i = 1:3
                if isvalid(app.JointSliders(i))
                    app.JointSliders(i).Value = jointValues(i);
                    app.LastSliderValues(i) = jointValues(i);
                end
            end
        end

        function statusSwitchToggled(app, src)
            % 用户切换开关时的处理：仅作日志并同步容器颜色
            try
                isOn = strcmp(src.Value, 'ON');
                if isOn
                    app.logMsg('用户将状态开关置为 ON');
                else
                    app.logMsg('用户将状态开关置为 OFF');
                end
                % 使用统一的更新函数同步颜色和值（避免递归）
                app.updateStatusSwitch(isOn);
            catch ME
                app.logMsg(['处理开关切换失败: ' ME.message]);
            end
        end

        function toggleCustomSwitch(app)
            % 自定义滑动开关被点击：切换状态并更新显示
            try
                newState = ~app.StatusIsConnected;
                app.updateStatusSwitch(newState);
                if newState
                    app.logMsg('用户通过自定义开关切换为 ON');
                else
                    app.logMsg('用户通过自定义开关切换为 OFF');
                end
            catch ME
                app.logMsg(['toggleCustomSwitch 失败: ' ME.message]);
            end
        end

        function jointSliderChanged(app,jointIndex,value)
            prevValue = app.LastSliderValues(jointIndex);
            app.LastSliderValues(jointIndex) = value;
            app.logMsg(sprintf('关节%d 调整到 %.1f',jointIndex,value));
            deltaPositive = value >= prevValue;
            if deltaPositive
                action = (jointIndex-1)*2;      % 正方向
            else
                action = (jointIndex-1)*2 + 1;  % 反方向
            end
            app.sendAction(action);
        end

        function sendAction(app,action)
            if ~app.ROSInitialized
                app.logMsg('ROS未连接，无法发送动作');
                return;
            end
            msg = rosmessage('std_msgs/Int32');
            msg.Data = int32(action);
            send(app.ActionPublisher,msg);
            app.logMsg(['发送动作: ' num2str(action)]);
        end

        function resetEnvironment(app)
            app.TotalSteps = 0;
            app.Episode = 1;
            app.EpisodeStep = 0;
            app.logMsg('环境已重置');
            app.sendAction(-1);
        end

        function loadArmImage(app)
            % 加载机械臂图片
            try
                imagePath = fullfile(pwd, '机械臂.png');
                if exist(imagePath, 'file')
                    app.ArmImageData = imread(imagePath);
                    [h, w, c] = size(app.ArmImageData);
                    app.logMsg(sprintf('机械臂图片加载成功: %dx%dx%d', h, w, c));
                else
                    app.logMsg(['警告: 未找到机械臂图片: ' imagePath]);
                    app.ArmImageData = [];
                end
            catch ME
                app.logMsg(['加载机械臂图片失败: ' ME.message]);
                app.ArmImageData = [];
            end
        end
        
        function logMsg(app,msg)
            app.LogArea.Value = [app.LogArea.Value; {sprintf('[%s] %s',datestr(now,'HH:MM:SS'),msg)}];
            app.LogArea.Value = app.LogArea.Value(max(1,end-200):end);
        end
    end

    methods (Access = public)
        function app = ArmControlApp
            createComponents(app);
            % 初始化状态开关为未连接（OFF，灰色轨道），再尝试连接 ROS
            app.updateStatusSwitch(false);
            % 不再加载机械臂图片：左侧统一采用四节点连杆绘制
            initializeROS(app);
            registerApp(app, app.UIFigure);
        end

        function delete(app)
            if ~isempty(app.RLSubscriber)
                clear app.RLSubscriber;
            end
            if app.ROSInitialized && app.isROSNodeActive()
                rosshutdown;
            end
            delete(app.UIFigure);
        end
    end
end

