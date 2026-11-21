function app = robot_viewer_ui(cfg)

%% =====================================================
% Robot Viewer UI
% 输入:
%   cfg.urdf_path
%   cfg.robot_name
%   cfg.workspace_m
%   cfg.joint_limit_deg
%       []          -> 从 URDF 读取 limit
%       [-180 180]  -> 所有关节统一覆盖
%       n x 2       -> 每个关节单独覆盖
%   cfg.home_deg
%   cfg.show_frames
%% =====================================================

%% ------------------------------
% 参数检查
%% ------------------------------

if ~isfield(cfg,'urdf_path')
    error("缺少 cfg.urdf_path");
end

if ~isfield(cfg,'robot_name')
    cfg.robot_name = "Robot URDF Viewer";
end

if ~isfield(cfg,'workspace_m')
    cfg.workspace_m = [
        -1 1;
        -1 1;
        -0.25 1.2
    ];
end

if ~isfield(cfg,'joint_limit_deg')
    cfg.joint_limit_deg = [];
end

if ~isfield(cfg,'show_frames')
    cfg.show_frames = false;
end

%% ------------------------------
% 读取机器人
%% ------------------------------

robot = importrobot(cfg.urdf_path);
robot.DataFormat = "row";

joint_names = {};
joint_body_names = {};
joint_limits_urdf_rad = [];

for i = 1:numel(robot.Bodies)
    joint = robot.Bodies{i}.Joint;

    if ~strcmp(joint.Type, "fixed")
        joint_names{end+1} = joint.Name;
        joint_body_names{end+1} = robot.Bodies{i}.Name;

        lim = joint.PositionLimits;

        if numel(lim) ~= 2 || any(~isfinite(lim)) || lim(1) >= lim(2)
            lim = [-pi pi];
        end

        joint_limits_urdf_rad(end+1,:) = lim;
    end
end

n = numel(joint_names);

if n == 0
    error("URDF 中没有可动关节。");
end

%% ------------------------------
% 关节范围
%% ------------------------------

if isempty(cfg.joint_limit_deg)

    % 默认从 URDF 读取
    joint_limits_deg = rad2deg(joint_limits_urdf_rad);

elseif size(cfg.joint_limit_deg,1) == 1

    % 一个范围覆盖所有关节
    joint_limits_deg = repmat(cfg.joint_limit_deg,n,1);

else

    % 每个关节单独设置
    if size(cfg.joint_limit_deg,1) ~= n || size(cfg.joint_limit_deg,2) ~= 2
        error("cfg.joint_limit_deg 尺寸应为 1x2 或 nx2。");
    end

    joint_limits_deg = cfg.joint_limit_deg;

end

%% ------------------------------
% Home pose
%% ------------------------------

if isfield(cfg,'home_deg') && numel(cfg.home_deg) == n
    q_home = deg2rad(cfg.home_deg);
else
    q_home = zeros(1,n);
end

q = q_home;

tcp_body = robot.BodyNames{end};

x_range = cfg.workspace_m(1,:);
y_range = cfg.workspace_m(2,:);
z_range = cfg.workspace_m(3,:);

%% =====================================================
% UI 总体布局
%% =====================================================

fig = uifigure( ...
    "Name",cfg.robot_name, ...
    "Position",[80 60 1280 860], ...
    "Color",[0.95 0.95 0.95]);

%% =====================================================
% 左侧 3D 视图
%% =====================================================

ax = uiaxes(fig, ...
    "Position",[25 25 900 810], ...
    "BackgroundColor",[1 1 1]);

hold(ax,"on");
grid(ax,"on");
axis(ax,"equal");
axis(ax,"vis3d");

xlabel(ax,"X / m");
ylabel(ax,"Y / m");
zlabel(ax,"Z / m");

xlim(ax,x_range);
ylim(ax,y_range);
zlim(ax,z_range);

view(ax,45,25);
title(ax,"Robot Kinematic Viewer");

try
    ax.Projection = "perspective";
catch
end

try
    axtoolbar(ax,{'rotate','pan','zoomin','zoomout','restoreview'});
catch
end

%% =====================================================
% 地面
%% =====================================================

patch(ax, ...
    [x_range(1) x_range(2) x_range(2) x_range(1)], ...
    [y_range(1) y_range(1) y_range(2) y_range(2)], ...
    [0 0 0 0], ...
    [0.90 0.90 0.90], ...
    "FaceAlpha",0.25, ...
    "EdgeColor",[0.78 0.78 0.78]);

grid_step = 0.25;

for v = x_range(1):grid_step:x_range(2)
    plot3(ax,[v v],y_range,[0 0], ...
        "Color",[0.86 0.86 0.86], ...
        "LineWidth",0.5);
end

for v = y_range(1):grid_step:y_range(2)
    plot3(ax,x_range,[v v],[0 0], ...
        "Color",[0.86 0.86 0.86], ...
        "LineWidth",0.5);
end

%% =====================================================
% 世界坐标轴
%% =====================================================

world_axis_len = 0.22;

quiver3(ax,0,0,0,world_axis_len,0,0, ...
    "Color",[1 0 0], ...
    "LineWidth",2, ...
    "MaxHeadSize",0.5);

quiver3(ax,0,0,0,0,world_axis_len,0, ...
    "Color",[0 0.6 0], ...
    "LineWidth",2, ...
    "MaxHeadSize",0.5);

quiver3(ax,0,0,0,0,0,world_axis_len, ...
    "Color",[0 0 1], ...
    "LineWidth",2, ...
    "MaxHeadSize",0.5);

text(ax,world_axis_len+0.03,0,0,"X", ...
    "Color",[1 0 0], ...
    "FontWeight","bold");

text(ax,0,world_axis_len+0.03,0,"Y", ...
    "Color",[0 0.6 0], ...
    "FontWeight","bold");

text(ax,0,0,world_axis_len+0.03,"Z", ...
    "Color",[0 0 1], ...
    "FontWeight","bold");

%% =====================================================
% 机器人骨架对象
%% =====================================================

h_link = plot3(ax,0,0,0, ...
    "-o", ...
    "LineWidth",5, ...
    "MarkerSize",8, ...
    "Color",[0.05 0.22 0.78], ...
    "MarkerFaceColor",[0.95 0.35 0.05], ...
    "MarkerEdgeColor",[0.1 0.1 0.1]);

h_tcp = scatter3(ax,0,0,0,110, ...
    "filled", ...
    "MarkerFaceColor",[0.85 0 0], ...
    "MarkerEdgeColor",[0.1 0.1 0.1]);

frame_len = 0.08;

h_fx = gobjects(n+1,1);
h_fy = gobjects(n+1,1);
h_fz = gobjects(n+1,1);
h_label = gobjects(n+1,1);

for i = 1:n+1
    h_fx(i) = quiver3(ax,0,0,0,0,0,0, ...
        "Color",[1 0 0], ...
        "LineWidth",1.1, ...
        "MaxHeadSize",0.8);

    h_fy(i) = quiver3(ax,0,0,0,0,0,0, ...
        "Color",[0 0.6 0], ...
        "LineWidth",1.1, ...
        "MaxHeadSize",0.8);

    h_fz(i) = quiver3(ax,0,0,0,0,0,0, ...
        "Color",[0 0 1], ...
        "LineWidth",1.1, ...
        "MaxHeadSize",0.8);

    h_label(i) = text(ax,0,0,0,"", ...
        "FontSize",10, ...
        "FontWeight","bold", ...
        "Color",[0.05 0.05 0.05]);
end

%% =====================================================
% 右侧：Robot Info
%% =====================================================

info_panel = uipanel(fig, ...
    "Title","Robot Info", ...
    "FontWeight","bold", ...
    "Position",[945 575 310 260], ...
    "BackgroundColor",[0.98 0.98 0.98]);

tcp_label = uilabel(info_panel, ...
    "Position",[18 135 260 90], ...
    "FontSize",14, ...
    "Text","TCP:");

joint_label = uilabel(info_panel, ...
    "Position",[18 40 170 85], ...
    "FontSize",12, ...
    "Text","Joint:");

frames_cb = uicheckbox(info_panel, ...
    "Text","Show Frames / Labels", ...
    "Value",cfg.show_frames, ...
    "Position",[18 10 170 25], ...
    "ValueChangedFcn",@(src,event) updateFrameVisibility());

uibutton(info_panel, ...
    "Text","Iso", ...
    "Position",[205 182 42 28], ...
    "ButtonPushedFcn",@(src,event) setIsoView());

uibutton(info_panel, ...
    "Text","Front", ...
    "Position",[252 182 48 28], ...
    "ButtonPushedFcn",@(src,event) setFrontView());

uibutton(info_panel, ...
    "Text","Side", ...
    "Position",[205 145 42 28], ...
    "ButtonPushedFcn",@(src,event) setSideView());

uibutton(info_panel, ...
    "Text","Top", ...
    "Position",[252 145 48 28], ...
    "ButtonPushedFcn",@(src,event) setTopView());

uibutton(info_panel, ...
    "Text","Reset", ...
    "Position",[205 85 95 35], ...
    "FontWeight","bold", ...
    "ButtonPushedFcn",@(src,event) resetRobot());

uibutton(info_panel, ...
    "Text","Home", ...
    "Position",[205 40 95 35], ...
    "FontWeight","bold", ...
    "ButtonPushedFcn",@(src,event) homeRobot());

%% =====================================================
% 右侧：Joint Control
%% =====================================================

ctrl_panel = uipanel(fig, ...
    "Title","Joint Control", ...
    "FontWeight","bold", ...
    "Position",[945 25 310 530], ...
    "BackgroundColor",[0.98 0.98 0.98]);

sliders = cell(n,1);
angle_edit = cell(n,1);
limit_left_label = cell(n,1);
limit_right_label = cell(n,1);

for i = 1:n

    row_top = 470 - (i-1)*78;

    joint_min = joint_limits_deg(i,1);
    joint_max = joint_limits_deg(i,2);

    uilabel(ctrl_panel, ...
        "Position",[18 row_top 70 22], ...
        "Text",joint_names{i}, ...
        "FontWeight","bold", ...
        "FontSize",12);

    angle_edit{i} = uieditfield(ctrl_panel,"numeric", ...
        "Position",[228 row_top 60 24], ...
        "Limits",[joint_min joint_max], ...
        "Value",rad2deg(q(i)), ...
        "RoundFractionalValues","off", ...
        "ValueChangedFcn",@(src,event) editChanged(i,src.Value));

    uilabel(ctrl_panel, ...
        "Position",[292 row_top 20 22], ...
        "Text","°", ...
        "FontSize",12);

    sliders{i} = uislider(ctrl_panel, ...
        "Position",[45 row_top-16 200 3], ...
        "Limits",[joint_min joint_max], ...
        "Value",rad2deg(q(i)), ...
        "MajorTicks",[joint_min joint_max], ...
        "MajorTickLabels",{'',''}, ...
        "MinorTicks",[]);

    sliders{i}.ValueChangingFcn = ...
        @(src,event) sliderChanging(i,event.Value);

    sliders{i}.ValueChangedFcn = ...
        @(src,event) sliderChanged(i,src.Value);

    limit_left_label{i} = uilabel(ctrl_panel, ...
        "Position",[24 row_top-38 42 18], ...
        "Text",formatDegLabel(joint_min), ...
        "FontSize",10, ...
        "HorizontalAlignment","center", ...
        "FontColor",[0.25 0.25 0.25]);

    limit_right_label{i} = uilabel(ctrl_panel, ...
        "Position",[222 row_top-38 36 18], ...
        "Text",formatDegLabel(joint_max), ...
        "FontSize",10, ...
        "HorizontalAlignment","center", ...
        "FontColor",[0.25 0.25 0.25]);
end

%% 初次更新
updateScene();

%% =====================================================
% slider 拖动中实时更新
%% =====================================================

    function sliderChanging(index,value_deg)
        q(index) = deg2rad(value_deg);
        angle_edit{index}.Value = value_deg;
        updateScene();
    end

%% =====================================================
% slider 松开后更新
%% =====================================================

    function sliderChanged(index,value_deg)
        q(index) = deg2rad(value_deg);
        angle_edit{index}.Value = value_deg;
        updateScene();
    end

%% =====================================================
% 数值框输入
%% =====================================================

    function editChanged(index,value_deg)

        lim = joint_limits_deg(index,:);
        value_deg = min(max(value_deg,lim(1)),lim(2));

        angle_edit{index}.Value = value_deg;
        sliders{index}.Value = value_deg;

        q(index) = deg2rad(value_deg);

        updateScene();
    end

%% =====================================================
% Reset
%% =====================================================

    function resetRobot()
        q(:) = 0;

        for k = 1:n
            sliders{k}.Value = 0;
            angle_edit{k}.Value = 0;
        end

        updateScene();
    end

%% =====================================================
% Home
%% =====================================================

    function homeRobot()
        q = q_home;

        for k = 1:n
            value_deg = rad2deg(q(k));
            sliders{k}.Value = value_deg;
            angle_edit{k}.Value = value_deg;
        end

        updateScene();
    end

%% =====================================================
% 更新场景
%% =====================================================

    function updateScene()

        P = zeros(3,n+1);
        R_all = zeros(3,3,n+1);

        P(:,1) = [0;0;0];
        R_all(:,:,1) = eye(3);

        for k = 1:n
            T = getTransform(robot,q,joint_body_names{k});
            P(:,k+1) = tform2trvec(T)';
            R_all(:,:,k+1) = tform2rotm(T);
        end

        T_tcp = getTransform(robot,q,tcp_body);
        tcp = tform2trvec(T_tcp)';

        if norm(tcp - P(:,end)) > 1e-8
            P_show = [P tcp];
        else
            P_show = P;
        end

        set(h_link, ...
            "XData",P_show(1,:), ...
            "YData",P_show(2,:), ...
            "ZData",P_show(3,:));

        set(h_tcp, ...
            "XData",tcp(1), ...
            "YData",tcp(2), ...
            "ZData",tcp(3));

        for k = 1:n+1

            origin = P(:,k);
            R = R_all(:,:,k);

            x_axis = R(:,1) * frame_len;
            y_axis = R(:,2) * frame_len;
            z_axis = R(:,3) * frame_len;

            set(h_fx(k), ...
                "XData",origin(1), ...
                "YData",origin(2), ...
                "ZData",origin(3), ...
                "UData",x_axis(1), ...
                "VData",x_axis(2), ...
                "WData",x_axis(3));

            set(h_fy(k), ...
                "XData",origin(1), ...
                "YData",origin(2), ...
                "ZData",origin(3), ...
                "UData",y_axis(1), ...
                "VData",y_axis(2), ...
                "WData",y_axis(3));

            set(h_fz(k), ...
                "XData",origin(1), ...
                "YData",origin(2), ...
                "ZData",origin(3), ...
                "UData",z_axis(1), ...
                "VData",z_axis(2), ...
                "WData",z_axis(3));

            if k == 1
                label_text = "Base";
            else
                label_text = string(joint_names{k-1});
            end

            set(h_label(k), ...
                "String",label_text, ...
                "Position",origin' + [0.025 0.025 0.025]);
        end

        tcp_label.Text = sprintf( ...
            "TCP Body: %s\nX: %.1f mm\nY: %.1f mm\nZ: %.1f mm", ...
            tcp_body, ...
            tcp(1)*1000, ...
            tcp(2)*1000, ...
            tcp(3)*1000);

        joint_text = "";

        for k = 1:n
            joint_text = joint_text + sprintf("%s: %.1f deg\n", ...
                joint_names{k}, rad2deg(q(k)));
        end

        joint_label.Text = joint_text;

        updateFrameVisibility();

        drawnow limitrate;
    end

%% =====================================================
% frame 显示/隐藏
%% =====================================================

    function updateFrameVisibility()

        if frames_cb.Value
            state = "on";
        else
            state = "off";
        end

        for k = 1:n+1
            h_fx(k).Visible = state;
            h_fy(k).Visible = state;
            h_fz(k).Visible = state;
            h_label(k).Visible = state;
        end
    end

%% =====================================================
% 视角按钮
%% =====================================================

    function setIsoView()
        xlim(ax,x_range);
        ylim(ax,y_range);
        zlim(ax,z_range);
        view(ax,45,25);
    end

    function setFrontView()
        xlim(ax,x_range);
        ylim(ax,y_range);
        zlim(ax,z_range);
        view(ax,0,0);
    end

    function setSideView()
        xlim(ax,x_range);
        ylim(ax,y_range);
        zlim(ax,z_range);
        view(ax,90,0);
    end

    function setTopView()
        xlim(ax,x_range);
        ylim(ax,y_range);
        zlim(ax,z_range);
        view(ax,0,90);
    end

%% =====================================================
% 角度标签格式
%% =====================================================

    function s = formatDegLabel(v)
        if abs(v - round(v)) < 1e-6
            s = sprintf("%.0f",v);
        else
            s = sprintf("%.1f",v);
        end
    end

%% =====================================================
% 输出 app 句柄，方便后续调试
%% =====================================================

app.fig = fig;
app.ax = ax;
app.robot = robot;
app.cfg = cfg;
app.joint_limits_deg = joint_limits_deg;

end
