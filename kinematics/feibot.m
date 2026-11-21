clear;
clc;
close all;

%% =====================================================
% Feibot Viewer
% 主程序：这里只输入机器人参数
%% =====================================================

cfg = struct();

% URDF 路径
cfg.urdf_path = fullfile(fileparts(mfilename('fullpath')), 'urdf', 'feibot.urdf');

% 窗口标题
cfg.robot_name = "Feibot URDF Viewer";

% 工作空间范围，单位 m
% X: -1000 ~ 1000 mm
% Y: -1000 ~ 1000 mm
% Z: -250  ~ 1200 mm
cfg.workspace_m = [
    -1.00   1.00;    % X
    -1.00   1.00;    % Y
    -0.25   1.20     % Z
];

% 关节角度范围：
% [] 表示读取 URDF 里面每个 joint 的 <limit lower="..." upper="..."/>
% 如果你想临时覆盖所有关节范围，可以写成 [-180 180]
cfg.joint_limit_deg = [];

% Home pose，单位 deg
cfg.home_deg = [0 0 0 0 0 0];

% 默认是否显示坐标系
cfg.show_frames = false;

% 启动 UI
robot_viewer_ui(cfg);
