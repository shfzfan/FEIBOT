function feibot_workspace
clc;
clear;
close all;

%% =====================================================
% Feibot Workspace Generator
%
% 功能:
%   1. 读取 ./urdf/feibot.urdf
%   2. 自动读取 URDF 里的 joint limit
%   3. 随机采样关节空间
%   4. 计算 TCP 可达点云
%   5. 绘制 3D 工作空间和 XY / XZ / YZ 投影
%   6. 可选保存 workspace 点云数据
%
% 注意:
%   这是运动学工作空间，不考虑碰撞、负载、关节干涉、线缆、力矩限制。
%% =====================================================

cfg = struct();

% URDF 路径
script_dir = fileparts(mfilename('fullpath'));
cfg.urdf_path = fullfile(script_dir, 'urdf', 'feibot.urdf');

% TCP body 名称。留空则默认使用 URDF 最后一个 body。
cfg.tcp_body = "";

% 随机采样数量。越大越密，但越慢。
cfg.sample_count = 50000;

% 随机种子，保证每次结果基本一致。
cfg.random_seed = 1;

% 如果某个 joint 没有合法 limit，就使用这个范围，单位 deg。

% 显示范围，单位 m。
% X/Y: -1000 ~ 1000 mm
% Z:   -250  ~ 1200 mm
cfg.workspace_m = [
    -1.00   1.00;
    -1.00   1.00;
    -0.25   1.20
];

% 是否尝试绘制 alphaShape 外壳。
% 点很多时可能稍慢；如果失败会自动跳过。
cfg.show_alpha_shape = true;

% alphaShape 参数，单位 m。
% 越小边界越贴合，但可能碎；越大边界更平滑。
cfg.alpha_radius = 0.12;

% 是否保存结果。
cfg.save_result = true;
cfg.output_dir = fullfile(script_dir, 'workspace_output');

%% =====================================================
% 读取机器人
%% =====================================================

robot = importrobot(cfg.urdf_path);
robot.DataFormat = "row";

joint_names = {};
joint_body_names = {};
joint_limits_rad = [];

for i = 1:numel(robot.Bodies)
    joint = robot.Bodies{i}.Joint;

    if ~strcmp(joint.Type, "fixed")
        joint_names{end+1} = joint.Name; %#ok<AGROW>
        joint_body_names{end+1} = robot.Bodies{i}.Name; %#ok<AGROW>

        lim = joint.PositionLimits;

        if numel(lim) ~= 2 || any(~isfinite(lim)) || lim(1) >= lim(2)
            lim = deg2rad(cfg.fallback_limit_deg);
        end

        joint_limits_rad(end+1,:) = lim; %#ok<AGROW>
    end
end

n = numel(joint_names);

if n == 0
    error("URDF 中没有可动关节。请检查 joint type。");
end

if strlength(cfg.tcp_body) == 0
    tcp_body = robot.BodyNames{end};
else
    tcp_body = cfg.tcp_body;
end

fprintf("Loaded robot: %s\n", cfg.urdf_path);
fprintf("TCP body: %s\n", tcp_body);
fprintf("Movable joints: %d\n\n", n);

fprintf("Joint limits from URDF:\n");
for k = 1:n
    fprintf("  %s: %.1f deg  to  %.1f deg\n", ...
        joint_names{k}, ...
        rad2deg(joint_limits_rad(k,1)), ...
        rad2deg(joint_limits_rad(k,2)));
end
fprintf("\n");

%% =====================================================
% 随机采样关节空间
%% =====================================================

N = cfg.sample_count;
points = zeros(N,3);
q_samples = zeros(N,n);

rng(cfg.random_seed);

lower = joint_limits_rad(:,1)';
upper = joint_limits_rad(:,2)';
range = upper - lower;

fprintf("Sampling %d configurations...\n", N);
tic;

for i = 1:N
    q = lower + rand(1,n) .* range;
    q_samples(i,:) = q;

    T = getTransform(robot, q, tcp_body);
    points(i,:) = tform2trvec(T);

    if mod(i, max(1,round(N/10))) == 0
        fprintf("  %.0f%%\n", i/N*100);
    end
end

elapsed = toc;
fprintf("Sampling finished. Time: %.2f s\n\n", elapsed);

points_mm = points * 1000;

%% =====================================================
% 统计工作空间范围
%% =====================================================

x_min = min(points_mm(:,1)); x_max = max(points_mm(:,1));
y_min = min(points_mm(:,2)); y_max = max(points_mm(:,2));
z_min = min(points_mm(:,3)); z_max = max(points_mm(:,3));

rho = sqrt(points_mm(:,1).^2 + points_mm(:,2).^2);
reach = sqrt(sum(points_mm.^2,2));

fprintf("Workspace range, TCP position only:\n");
fprintf("  X: %.1f mm  to  %.1f mm\n", x_min, x_max);
fprintf("  Y: %.1f mm  to  %.1f mm\n", y_min, y_max);
fprintf("  Z: %.1f mm  to  %.1f mm\n", z_min, z_max);
fprintf("  Horizontal radius rho: %.1f mm  to  %.1f mm\n", min(rho), max(rho));
fprintf("  3D reach from base: %.1f mm  to  %.1f mm\n", min(reach), max(reach));
fprintf("\n");

%% =====================================================
% 3D 工作空间点云
%% =====================================================

fig1 = figure( ...
    "Name", "Feibot Workspace - 3D", ...
    "Color", "w", ...
    "Position", [100 100 1000 800]);

ax1 = axes(fig1);
hold(ax1,"on");
grid(ax1,"on");
axis(ax1,"equal");
axis(ax1,"vis3d");

xlabel(ax1,"X / mm");
ylabel(ax1,"Y / mm");
zlabel(ax1,"Z / mm");
title(ax1,"Feibot TCP Workspace Point Cloud");

xlim(ax1,cfg.workspace_m(1,:) * 1000);
ylim(ax1,cfg.workspace_m(2,:) * 1000);
zlim(ax1,cfg.workspace_m(3,:) * 1000);
view(ax1,45,25);

% 地面
patch(ax1, ...
    [cfg.workspace_m(1,1) cfg.workspace_m(1,2) cfg.workspace_m(1,2) cfg.workspace_m(1,1)]*1000, ...
    [cfg.workspace_m(2,1) cfg.workspace_m(2,1) cfg.workspace_m(2,2) cfg.workspace_m(2,2)]*1000, ...
    [0 0 0 0], ...
    [0.90 0.90 0.90], ...
    "FaceAlpha",0.25, ...
    "EdgeColor",[0.75 0.75 0.75]);

scatter3(ax1, ...
    points_mm(:,1), ...
    points_mm(:,2), ...
    points_mm(:,3), ...
    3, ...
    points_mm(:,3), ...
    "filled", ...
    "MarkerFaceAlpha",0.18, ...
    "MarkerEdgeAlpha",0.18);

colormap(ax1,"turbo");
cb = colorbar(ax1);
cb.Label.String = "Z / mm";

% 世界坐标轴
axis_len_mm = 250;
quiver3(ax1,0,0,0,axis_len_mm,0,0,"r","LineWidth",2,"MaxHeadSize",0.5);
quiver3(ax1,0,0,0,0,axis_len_mm,0,"g","LineWidth",2,"MaxHeadSize",0.5);
quiver3(ax1,0,0,0,0,0,axis_len_mm,"b","LineWidth",2,"MaxHeadSize",0.5);
text(ax1,axis_len_mm+30,0,0,"X","Color","r","FontWeight","bold");
text(ax1,0,axis_len_mm+30,0,"Y","Color","g","FontWeight","bold");
text(ax1,0,0,axis_len_mm+30,"Z","Color","b","FontWeight","bold");

%% =====================================================
% alphaShape 外壳
%% =====================================================

if cfg.show_alpha_shape
    try
        shp = alphaShape(points(:,1), points(:,2), points(:,3), cfg.alpha_radius);

        fig2 = figure( ...
            "Name", "Feibot Workspace - Boundary", ...
            "Color", "w", ...
            "Position", [150 120 1000 800]);

        ax2 = axes(fig2);
        hold(ax2,"on");
        grid(ax2,"on");
        axis(ax2,"equal");
        axis(ax2,"vis3d");

        xlabel(ax2,"X / mm");
        ylabel(ax2,"Y / mm");
        zlabel(ax2,"Z / mm");
        title(ax2,"Approximate Workspace Boundary");

        xlim(ax2,cfg.workspace_m(1,:) * 1000);
        ylim(ax2,cfg.workspace_m(2,:) * 1000);
        zlim(ax2,cfg.workspace_m(3,:) * 1000);
        view(ax2,45,25);

        % alphaShape 是 m 单位，转 mm 只能手动画 boundary facets
        [bf, p] = boundaryFacets(shp);
        trisurf(bf, p(:,1)*1000, p(:,2)*1000, p(:,3)*1000, ...
            "Parent", ax2, ...
            "FaceAlpha",0.18, ...
            "EdgeAlpha",0.08, ...
            "FaceColor",[0.05 0.35 0.95], ...
            "EdgeColor",[0.05 0.05 0.05]);

        scatter3(ax2, points_mm(:,1), points_mm(:,2), points_mm(:,3), ...
            1, [0.1 0.1 0.1], "filled", "MarkerFaceAlpha",0.05);

        fprintf("alphaShape boundary generated. Alpha radius: %.3f m\n\n", cfg.alpha_radius);
    catch ME
        warning("alphaShape boundary skipped: %s", ME.message);
    end
end

%% =====================================================
% 2D 投影
%% =====================================================

fig3 = figure( ...
    "Name", "Feibot Workspace - Projections", ...
    "Color", "w", ...
    "Position", [200 150 1350 420]);

tiledlayout(fig3,1,3,"Padding","compact","TileSpacing","compact");

% XY
ax_xy = nexttile;
scatter(ax_xy, points_mm(:,1), points_mm(:,2), 2, "filled", "MarkerFaceAlpha",0.12);
grid(ax_xy,"on"); axis(ax_xy,"equal");
xlabel(ax_xy,"X / mm"); ylabel(ax_xy,"Y / mm"); title(ax_xy,"XY Projection");
xlim(ax_xy,cfg.workspace_m(1,:) * 1000);
ylim(ax_xy,cfg.workspace_m(2,:) * 1000);

% XZ
ax_xz = nexttile;
scatter(ax_xz, points_mm(:,1), points_mm(:,3), 2, "filled", "MarkerFaceAlpha",0.12);
grid(ax_xz,"on"); axis(ax_xz,"equal");
xlabel(ax_xz,"X / mm"); ylabel(ax_xz,"Z / mm"); title(ax_xz,"XZ Projection");
xlim(ax_xz,cfg.workspace_m(1,:) * 1000);
ylim(ax_xz,cfg.workspace_m(3,:) * 1000);

% YZ
ax_yz = nexttile;
scatter(ax_yz, points_mm(:,2), points_mm(:,3), 2, "filled", "MarkerFaceAlpha",0.12);
grid(ax_yz,"on"); axis(ax_yz,"equal");
xlabel(ax_yz,"Y / mm"); ylabel(ax_yz,"Z / mm"); title(ax_yz,"YZ Projection");
xlim(ax_yz,cfg.workspace_m(2,:) * 1000);
ylim(ax_yz,cfg.workspace_m(3,:) * 1000);

%% =====================================================
% 保存结果
%% =====================================================

if cfg.save_result
    if ~exist(cfg.output_dir,"dir")
        mkdir(cfg.output_dir);
    end

    timestamp = datestr(now,"yyyymmdd_HHMMSS");

    mat_path = fullfile(cfg.output_dir, "feibot_workspace_" + timestamp + ".mat");
    csv_path = fullfile(cfg.output_dir, "feibot_workspace_points_mm_" + timestamp + ".csv");

    workspace_data = struct();
    workspace_data.points_m = points;
    workspace_data.points_mm = points_mm;
    workspace_data.q_samples_rad = q_samples;
    workspace_data.q_samples_deg = rad2deg(q_samples);
    workspace_data.joint_names = joint_names;
    workspace_data.joint_limits_deg = rad2deg(joint_limits_rad);
    workspace_data.tcp_body = tcp_body;
    workspace_data.urdf_path = cfg.urdf_path;
    workspace_data.stats_mm = struct( ...
        "x_min",x_min,"x_max",x_max, ...
        "y_min",y_min,"y_max",y_max, ...
        "z_min",z_min,"z_max",z_max, ...
        "rho_min",min(rho),"rho_max",max(rho), ...
        "reach_min",min(reach),"reach_max",max(reach));

    save(mat_path,"workspace_data");
    writematrix(points_mm,csv_path);

    fprintf("Saved workspace data:\n");
    fprintf("  %s\n", mat_path);
    fprintf("  %s\n", csv_path);
end

end
