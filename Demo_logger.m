clear; clc; close all;

set(groot, 'defaultFigureColor', 'w');
set(groot, 'defaultAxesColor', 'w');
set(groot, 'defaultAxesXColor', 'k');
set(groot, 'defaultAxesYColor', 'k');
set(groot, 'defaultAxesZColor', 'k');
set(groot, 'defaultTextColor', 'k');
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultLegendFontName', 'Times New Roman');

%% Global config

baseLogDir = fullfile(pwd, 'log');

globalCfg = struct();

globalCfg.thrust_margin = 22.0;

globalCfg.smooth.thrust      = 0.02;
globalCfg.smooth.pos_z       = 0.08;
globalCfg.smooth.roll_act    = 0.08;
globalCfg.smooth.roll_des    = 0.05;
globalCfg.smooth.roll_mrg    = 0.05;
globalCfg.smooth.tau         = 0.05;

% true  -> crop 이후 시작 시간을 0초로 맞춤
% false -> 원래 로그 시간 유지
globalCfg.time.zero_after_crop = true;

% z plot direction
% true  -> z-axis is plotted upside down
% false -> normal y-axis direction
globalCfg.z_reverse = false;

% integrated plot limits
% all cases use these limits
globalCfg.lim.thrust = [0 35];
globalCfg.lim.tau    = [-8 8];
globalCfg.lim.z      = [-0.35 0.15];
globalCfg.lim.roll   = [-40 40];

% z hovering bound box
globalCfg.z_bound.ylim  = [-0.12 0.12];
globalCfg.z_bound.label = 'Hovering bound';
globalCfg.z_bound.color = [0.95 0.95 0.95];
globalCfg.z_bound.alpha = 0.35;

% common figure ratio
globalCfg.figPos = [80 40 640 980];
globalCfg.mainPos = [0.100 0.065 0.860 0.900];

% legend setting
globalCfg.legend.location = 'southwest';
globalCfg.legend.orientation = 'vertical';

%% Case configs

cases = struct([]);

% -------------------------------------------------------------------------
% Case 1: demo10
cases(1).name = 'demo10';
cases(1).file = fullfile(baseLogDir, 'demo10.npz');
cases(1).tLim = [60 75];

cases(1).phase.regions = {
    73.2,  75.0,  'Moving', [0.86 0.92 0.98];
};

cases(1).phase.alpha = 0.55;

cases(1).objectGrip.t = 70.8;
cases(1).objectGrip.label = 'Object grip';
cases(1).objectGrip.color = [0.20 0.20 0.20];
cases(1).objectGrip.yFrac = 0.12;

% -------------------------------------------------------------------------
% Case 2: demo13
cases(2).name = 'demo13';
cases(2).file = fullfile(baseLogDir, 'demo13.npz');
cases(2).tLim = [88 103];

cases(2).phase.regions = {
    100.5, 103.0,  'Moving', [0.86 0.92 0.98];
};

cases(2).phase.alpha = 0.55;

cases(2).objectGrip.t = 98.45;
cases(2).objectGrip.label = 'Object grip';
cases(2).objectGrip.color = [0.20 0.20 0.20];
cases(2).objectGrip.yFrac = 0.12;

% -------------------------------------------------------------------------
% Case 3: demo5
cases(3).name = 'demo5';
cases(3).file = fullfile(baseLogDir, 'demo5.npz');
cases(3).tLim = [76.5 91.5];

cases(3).phase.regions = {
    88.2,  95.0,  'Moving', [0.86 0.92 0.98];
};

cases(3).phase.alpha = 0.55;

cases(3).objectGrip.t = 86.78;
cases(3).objectGrip.label = 'Object grip';
cases(3).objectGrip.color = [0.20 0.20 0.20];
cases(3).objectGrip.yFrac = 0.12;

%% Run all cases

for k = 1:numel(cases)
    cfg = merge_case_config(globalCfg, cases(k));
    plot_one_case(cfg);
end

%% local functions

function plot_one_case(cfg)
    npz_file = cfg.file;

    if ~isfile(npz_file)
        warning('NPZ file not found, skip:\n%s', npz_file);
        return;
    end

    D = load_npz_all(npz_file);

    fprintf('\n\n============================================================\n');
    fprintf('Case: %s\n', cfg.name);
    fprintf('File: %s\n', npz_file);
    fprintf('============================================================\n');

    disp('===== Loaded fields =====');
    disp(fieldnames(D));

    [t, t_name] = pickField(D, {'t', 'time'});

    if isempty(t)
        error('Time field not found in %s.', cfg.name);
    end

    t = t(:);

    fprintf('\nTime field : %s\n', t_name);
    fprintf('Original time range : %.3f ~ %.3f s\n', t(1), t(end));

    %% Pick fields

    [pos, pos_name] = pickField(D, {'pos'});
    [pos_d, pos_d_name] = pickField(D, {'pos_d', 'pos_des'});

    [rpy, rpy_name] = pickField(D, {'rpy'});
    [rpy_raw, rpy_raw_name] = pickField(D, {'rpy_raw', 'rpy_des_raw'});
    [rpy_d, rpy_d_name] = pickField(D, {'rpy_d', 'rpy_des'});

    % thrust
    % F_raw : saturation before / unconstrained thrust
    [F_raw, F_raw_name] = pickField(D, {'f_thrst', 'f_thrust', 'f_thrust_num', 'f_thrst_num'});

    [tau_d, tau_d_name] = pickField(D, {'tau_d', 'tau_des'});
    [tau_thrust, tau_thrust_name] = pickField(D, {'tau_thrust'});
    [tau_off, tau_off_name] = pickField(D, {'tau_off'});

    fprintf('\n===== Field map =====\n');
    print_map('pos', pos_name);
    print_map('pos_d', pos_d_name);
    print_map('rpy', rpy_name);
    print_map('rpy_raw', rpy_raw_name);
    print_map('rpy_d', rpy_d_name);
    print_map('thrust raw', F_raw_name);
    print_map('tau_d', tau_d_name);
    print_map('tau_thrust', tau_thrust_name);
    print_map('tau_off', tau_off_name);

    %% Crop time

    if isempty(cfg.tLim)
        idx = true(size(t));
    else
        idx = (t >= cfg.tLim(1)) & (t <= cfg.tLim(2));
    end

    t = t(idx);

    if isempty(t)
        error('No data after time crop. Check cfg.tLim for %s.', cfg.name);
    end

    pos = crop_if(pos, idx);
    pos_d = crop_if(pos_d, idx);
    rpy = crop_if(rpy, idx);
    rpy_raw = crop_if(rpy_raw, idx);
    rpy_d = crop_if(rpy_d, idx);
    F_raw = crop_if(F_raw, idx);
    tau_d = crop_if(tau_d, idx);
    tau_thrust = crop_if(tau_thrust, idx);
    tau_off = crop_if(tau_off, idx);

    t_crop_start = t(1);

    if cfg.time.zero_after_crop
        t = t - t_crop_start;
        cfg = shift_time_config_after_crop(cfg, t_crop_start);
    end

    fprintf('\nAfter crop: N = %d\n', numel(t));
    fprintf('Cropped time starts from %.3f s in original log\n', t_crop_start);
    fprintf('Plot time range : %.3f ~ %.3f s\n', t(1), t(end));

    %% Data arrangement

    N = numel(t);

    % z axis
    z = ensure_len(pick_col(pos, 3), N);
    z_d = ensure_len(pick_col(pos_d, 3), N);

    % z error: desired - actual
    z_err = z_d - z;

    % roll
    roll_act = ensure_len(pick_col(rpy, 1), N);
    roll_raw = ensure_len(pick_col(rpy_raw, 1), N);
    roll_mrg = ensure_len(pick_col(rpy_d, 1), N);

    if all(isnan(roll_raw)) && ~all(isnan(roll_mrg))
        roll_raw = roll_mrg;
    end

    if all(isnan(roll_mrg)) && ~all(isnan(roll_raw))
        roll_mrg = roll_raw;
    end

    % thrust raw / unconstrained
    if isempty(F_raw) || size(F_raw, 2) < 4
        F1_raw = nan(N, 1);
        F2_raw = nan(N, 1);
        F3_raw = nan(N, 1);
        F4_raw = nan(N, 1);
    else
        F1_raw = ensure_len(F_raw(:, 1), N);
        F2_raw = ensure_len(F_raw(:, 2), N);
        F3_raw = ensure_len(F_raw(:, 3), N);
        F4_raw = ensure_len(F_raw(:, 4), N);
    end

    % raw thrust만 사용:
    % 1) 먼저 raw thrust에 LPF 적용
    % 2) LPF된 raw thrust를 virtual margin에서 잘라서 실선 saturation 모양 생성
    % 3) LPF된 raw thrust 중 virtual margin 위쪽만 점선으로 표시
    F1_raw_plot = F1_raw;
    F2_raw_plot = F2_raw;
    F3_raw_plot = F3_raw;
    F4_raw_plot = F4_raw;

    % roll torque direction
    tau_x_des = ensure_len(pick_col(tau_d, 1), N);
    tau_x_off = ensure_len(pick_col(tau_off, 1), N);

    % tau_thrust는 로그값을 쓰지 않고 무조건 des - tau_off로 계산
    tau_x_thrust = tau_x_des - tau_x_off;

    %% Smoothing

    F1_raw_s = lpf1(F1_raw_plot, cfg.smooth.thrust);
    F2_raw_s = lpf1(F2_raw_plot, cfg.smooth.thrust);
    F3_raw_s = lpf1(F3_raw_plot, cfg.smooth.thrust);
    F4_raw_s = lpf1(F4_raw_plot, cfg.smooth.thrust);

    F1_con_s = min(F1_raw_s, cfg.thrust_margin);
    F2_con_s = min(F2_raw_s, cfg.thrust_margin);
    F3_con_s = min(F3_raw_s, cfg.thrust_margin);
    F4_con_s = min(F4_raw_s, cfg.thrust_margin);

    F1_raw_over_s = F1_raw_s;
    F2_raw_over_s = F2_raw_s;
    F3_raw_over_s = F3_raw_s;
    F4_raw_over_s = F4_raw_s;

    F1_raw_over_s(F1_raw_over_s < cfg.thrust_margin) = nan;
    F2_raw_over_s(F2_raw_over_s < cfg.thrust_margin) = nan;
    F3_raw_over_s(F3_raw_over_s < cfg.thrust_margin) = nan;
    F4_raw_over_s(F4_raw_over_s < cfg.thrust_margin) = nan;

    z_err_s = lpf1(z_err, cfg.smooth.pos_z);

    roll_act_s = rad2deg(lpf1(roll_act, cfg.smooth.roll_act));
    roll_raw_s = rad2deg(lpf1(roll_raw, cfg.smooth.roll_des));
    roll_mrg_s = rad2deg(lpf1(roll_mrg, cfg.smooth.roll_mrg));

    tau_x_des_s = lpf1(tau_x_des, cfg.smooth.tau);
    tau_x_thrust_s = lpf1(tau_x_thrust, cfg.smooth.tau);
    tau_x_off_s = lpf1(tau_x_off, cfg.smooth.tau);

    %% Colors

    C.F1 = [0.00 0.35 0.85];
    C.F2 = [0.85 0.15 0.15];
    C.F3 = [0.15 0.70 0.35];
    C.F4 = [0.90 0.65 0.10];

    C.des = [0.10 0.10 0.10];
    C.act = [0.00 0.35 0.85];
    C.mrg = [0.90 0.15 0.15];

    C.tau_des = [0.10 0.10 0.10];
    C.tau_thrust = [0.15 0.45 0.90];
    C.tau_off = [0.85 0.35 0.65];

    %% Figure

    fig = figure('Color', 'w', 'Position', cfg.figPos, 'Name', cfg.name);
    set(fig, 'InvertHardcopy', 'off');

    TL = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    TL.Position = cfg.mainPos;

    % 1) Rotor thrusts
    ax1 = nexttile(TL, 1);
    hold(ax1, 'on');

    % constrained / saturated solid from LPF raw thrust
    p1c = plot(ax1, t, F1_con_s, 'LineWidth', 1.9, 'Color', C.F1);
    p2c = plot(ax1, t, F2_con_s, 'LineWidth', 1.9, 'Color', C.F2);
    p3c = plot(ax1, t, F3_con_s, 'LineWidth', 1.9, 'Color', C.F3);
    p4c = plot(ax1, t, F4_con_s, 'LineWidth', 1.9, 'Color', C.F4);

    % LPF raw / unconstrained only above thrust margin
    p1r = plot(ax1, t, F1_raw_over_s, '--', 'LineWidth', 1.5, 'Color', C.F1);
    p2r = plot(ax1, t, F2_raw_over_s, '--', 'LineWidth', 1.5, 'Color', C.F2);
    p3r = plot(ax1, t, F3_raw_over_s, '--', 'LineWidth', 1.5, 'Color', C.F3);
    p4r = plot(ax1, t, F4_raw_over_s, '--', 'LineWidth', 1.5, 'Color', C.F4);

    yline(ax1, cfg.thrust_margin, '--', 'Thrust margin', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.4, 'LabelHorizontalAlignment', 'left');

    grid(ax1, 'on');
    ylabel(ax1, 'Thrust [N]');
    title(ax1, 'Rotor thrusts');

    xlim_auto_or_cfg(ax1, t, []);
    apply_ylim(ax1, cfg.lim.thrust);

    hmove1 = add_phase_regions(ax1, cfg.phase.regions, cfg.phase.alpha);
    hog1 = add_vertical_event_line(ax1, cfg.objectGrip.t, cfg.objectGrip.label, cfg.objectGrip.color, cfg.objectGrip.yFrac);

    uistack([p1c p2c p3c p4c p1r p2r p3r p4r hog1], 'top');

    legend_items = [p1c p2c p3c p4c p1r];
    legend_names = {'F_1', 'F_2', 'F_3', 'F_4', 'F_{i,des}'};

    if ~isempty(hmove1)
        hmove1_leg = patch(ax1, nan, nan, cfg.phase.regions{1, 4}, ...
            'FaceAlpha', 0.85, ...
            'EdgeColor', [0.25 0.25 0.25], ...
            'LineWidth', 0.8, ...
            'DisplayName', 'Moving');

        legend_items = [legend_items hmove1_leg];
        legend_names = [legend_names {'Moving'}];
    end

    legend(ax1, legend_items, legend_names, ...
        'Location', 'southwest', ...
        'NumColumns', 4, ...
        'Box', 'on');

    % 2) Roll torque
    ax2 = nexttile(TL, 2);
    hold(ax2, 'on');

    ht1 = plot(ax2, t, tau_x_des_s, '--', 'LineWidth', 1.5, 'Color', C.tau_des);
    ht2 = plot(ax2, t, tau_x_thrust_s, 'LineWidth', 1.8, 'Color', C.tau_thrust);
    ht3 = plot(ax2, t, tau_x_off_s, 'LineWidth', 1.8, 'Color', C.tau_off);

    yline(ax2, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');

    grid(ax2, 'on');
    ylabel(ax2, '\tau_x [N m]');
    title(ax2, 'Roll torque');

    xlim_auto_or_cfg(ax2, t, []);
    apply_ylim(ax2, cfg.lim.tau);

    hmove2 = add_phase_regions(ax2, cfg.phase.regions, cfg.phase.alpha);
    hog2 = add_vertical_event_line(ax2, cfg.objectGrip.t, cfg.objectGrip.label, cfg.objectGrip.color, cfg.objectGrip.yFrac);

    uistack([ht1 ht2 ht3 hog2], 'top');

    legend_items = [ht1 ht2 ht3];
    legend_names = {'\tau_{x,des}', '\tau_{x,thrust}', '\tau_{x,off}'};

    if ~isempty(hmove2)
        hmove2_leg = patch(ax2, nan, nan, cfg.phase.regions{1, 4}, ...
            'FaceAlpha', 0.85, ...
            'EdgeColor', [0.25 0.25 0.25], ...
            'LineWidth', 0.8, ...
            'DisplayName', 'Moving');

        legend_items = [legend_items hmove2_leg];
        legend_names = [legend_names {'Moving'}];
    end

    legend(ax2, legend_items, legend_names, ...
        'Location', cfg.legend.location, ...
        'Orientation', cfg.legend.orientation, ...
        'Box', 'on');

    % 3) Z-axis tracking error
    ax3 = nexttile(TL, 3);
    hold(ax3, 'on');

    hz = plot(ax3, t, z_err_s, 'LineWidth', 1.8, 'Color', C.des);

    grid(ax3, 'on');
    ylabel(ax3, 'z_{des}-z [m]');
    title(ax3, 'Z-axis tracking');

    xlim_auto_or_cfg(ax3, t, []);
    apply_ylim(ax3, cfg.lim.z);

    if cfg.z_reverse
        set(ax3, 'YDir', 'reverse');
    else
        set(ax3, 'YDir', 'normal');
    end

    hb = add_y_bound_box(ax3, cfg.z_bound.ylim, cfg.z_bound.label, cfg.z_bound.color, cfg.z_bound.alpha);
    hmove3 = add_phase_regions(ax3, cfg.phase.regions, cfg.phase.alpha);
    hog3 = add_vertical_event_line(ax3, cfg.objectGrip.t, cfg.objectGrip.label, cfg.objectGrip.color, cfg.objectGrip.yFrac);

    if ~isempty(hb)
        uistack(hb, 'bottom');
    end
    uistack([hz hog3], 'top');

    legend_items = hz;
    legend_names = {'z_{des}-z'};

    if ~isempty(hmove3)
        hmove3_leg = patch(ax3, nan, nan, cfg.phase.regions{1, 4}, ...
            'FaceAlpha', 0.85, ...
            'EdgeColor', [0.25 0.25 0.25], ...
            'LineWidth', 0.8, ...
            'DisplayName', 'Moving');

        legend_items = [legend_items hmove3_leg];
        legend_names = [legend_names {'Moving'}];
    end

    legend(ax3, legend_items, legend_names, ...
        'Location', cfg.legend.location, ...
        'Orientation', cfg.legend.orientation, ...
        'Box', 'on');

    % 4) Roll attitude
    ax4 = nexttile(TL, 4);
    hold(ax4, 'on');

    hr1 = plot(ax4, t, roll_raw_s, '--', 'LineWidth', 1.5, 'Color', C.des);
    hr2 = plot(ax4, t, roll_mrg_s, 'LineWidth', 1.8, 'Color', C.mrg);
    hr3 = plot(ax4, t, roll_act_s, 'LineWidth', 1.8, 'Color', C.act);

    grid(ax4, 'on');
    xlabel(ax4, 'Time [s]');
    ylabel(ax4, 'roll [deg]');
    title(ax4, 'Roll tracking');

    xlim_auto_or_cfg(ax4, t, []);
    apply_ylim(ax4, cfg.lim.roll);

    hmove4 = add_phase_regions(ax4, cfg.phase.regions, cfg.phase.alpha);
    hog4 = add_vertical_event_line(ax4, cfg.objectGrip.t, cfg.objectGrip.label, cfg.objectGrip.color, cfg.objectGrip.yFrac);

    uistack([hr1 hr2 hr3 hog4], 'top');

    legend_items = [hr1 hr2 hr3];
    legend_names = {'roll_{raw}', 'roll_{MRG}', 'roll'};

    if ~isempty(hmove4)
        hmove4_leg = patch(ax4, nan, nan, cfg.phase.regions{1, 4}, ...
            'FaceAlpha', 0.85, ...
            'EdgeColor', [0.25 0.25 0.25], ...
            'LineWidth', 0.8, ...
            'DisplayName', 'Moving');

        legend_items = [legend_items hmove4_leg];
        legend_names = [legend_names {'Moving'}];
    end

    legend(ax4, legend_items, legend_names, ...
        'Location', 'southwest', ...
        'NumColumns', 3, ...
        'Box', 'on');

    linkaxes([ax1 ax2 ax3 ax4], 'x');

    apply_paper_style(fig);

    %% print summary

    fprintf('\n===== Plot summary: %s =====\n', cfg.name);
    fprintf('Thrust margin clip = %.3f N\n', cfg.thrust_margin);

    if ~all(isnan(F1_raw))
        fprintf('Max F1 raw         = %.3f N\n', max(F1_raw, [], 'omitnan'));
        fprintf('Max F2 raw         = %.3f N\n', max(F2_raw, [], 'omitnan'));
        fprintf('Max F3 raw         = %.3f N\n', max(F3_raw, [], 'omitnan'));
        fprintf('Max F4 raw         = %.3f N\n', max(F4_raw, [], 'omitnan'));
    end
end

function cfg = merge_case_config(globalCfg, caseCfg)
    cfg = globalCfg;

    cfg.name = caseCfg.name;
    cfg.file = caseCfg.file;
    cfg.tLim = caseCfg.tLim;

    cfg.phase = caseCfg.phase;
    cfg.objectGrip = caseCfg.objectGrip;
end

function cfg = shift_time_config_after_crop(cfg, t_crop_start)
    if ~isempty(cfg.phase.regions)
        for i = 1:size(cfg.phase.regions, 1)
            cfg.phase.regions{i, 1} = cfg.phase.regions{i, 1} - t_crop_start;
            cfg.phase.regions{i, 2} = cfg.phase.regions{i, 2} - t_crop_start;
        end
    end

    cfg.objectGrip.t = cfg.objectGrip.t - t_crop_start;
end

function D = load_npz_all(npz_file)
    np = py.importlib.import_module('numpy');
    data = np.load(npz_file, pyargs('allow_pickle', true));

    keys_cell = cell(data.files);
    keys = cellfun(@char, keys_cell, 'UniformOutput', false);

    D = struct();

    for i = 1:numel(keys)
        key = keys{i};
        key_valid = matlab.lang.makeValidName(key);
        arr_py = data.get(key);

        try
            arr = double(arr_py);
            D.(key_valid) = arr;
        catch
            try
                D.(key_valid) = char(arr_py);
            catch
                D.(key_valid) = arr_py;
            end
        end
    end
end

function [data, used_name] = pickField(D, candidates)
    data = [];
    used_name = '';

    for k = 1:numel(candidates)
        f = matlab.lang.makeValidName(candidates{k});
        if isfield(D, f)
            data = D.(f);
            used_name = f;
            return;
        end
    end
end

function y = crop_if(x, idx)
    if isempty(x)
        y = x;
        return;
    end

    if isnumeric(x) && ~isscalar(x)
        if size(x, 1) == numel(idx)
            y = x(idx, :);
        else
            y = x;
        end
    else
        y = x;
    end
end

function c = pick_col(x, n)
    if isempty(x) || ~isnumeric(x)
        c = [];
        return;
    end

    if isvector(x)
        x = x(:);

        if n == 1
            c = x;
        else
            c = nan(size(x));
        end

        return;
    end

    if size(x, 2) >= n
        c = x(:, n);
    else
        c = nan(size(x, 1), 1);
    end
end

function y = ensure_len(x, n)
    if isempty(x)
        y = nan(n, 1);
        return;
    end

    x = double(x);
    x = x(:);

    if numel(x) == n
        y = x;
    elseif numel(x) > n
        y = x(1:n);
    else
        y = nan(n, 1);
        y(1:numel(x)) = x;
    end
end

function y = lpf1(x, alpha)
    if isempty(x)
        y = x;
        return;
    end

    x = double(x);

    if all(isnan(x))
        y = x;
        return;
    end

    if isempty(alpha) || alpha <= 0 || alpha > 1
        y = x;
        return;
    end

    y = x;

    first_valid = find(~isnan(x), 1, 'first');

    if isempty(first_valid)
        y = x;
        return;
    end

    y(1:first_valid) = x(first_valid);

    for i = first_valid+1:numel(x)
        xi = x(i);
        yim1 = y(i-1);

        if isnan(xi)
            xi = yim1;
        end

        y(i) = alpha * xi + (1 - alpha) * yim1;
    end
end

function y = lpf1_nan_preserve(x, alpha)
    if isempty(x)
        y = x;
        return;
    end

    x = double(x);

    if all(isnan(x))
        y = x;
        return;
    end

    y = x;

    valid_segment = ~isnan(x);

    if ~any(valid_segment)
        return;
    end

    idx = find(valid_segment);
    segment_start = idx([true; diff(idx(:)) > 1]);
    segment_end = idx([diff(idx(:)) > 1; true]);

    for s = 1:numel(segment_start)
        i1 = segment_start(s);
        i2 = segment_end(s);

        y(i1:i2) = lpf1(x(i1:i2), alpha);
    end
end

function h = add_phase_regions(ax, regions, alpha_val)
    h = [];

    if isempty(regions)
        return;
    end

    hold(ax, 'on');
    xl = xlim(ax);
    yl = ylim(ax);

    for i = 1:size(regions, 1)
        x1 = regions{i, 1};
        x2 = regions{i, 2};
        color_in = regions{i, 4};

        if x2 <= xl(1) || x1 >= xl(2)
            continue;
        end

        xp1 = max(x1, xl(1));
        xp2 = min(x2, xl(2));

        p = patch(ax, [xp1 xp2 xp2 xp1], [yl(1) yl(1) yl(2) yl(2)], color_in, ...
            'FaceAlpha', alpha_val, ...
            'EdgeColor', 'none', ...
            'HandleVisibility', 'on', ...
            'HitTest', 'off');

        uistack(p, 'bottom');
        h = [h p];
    end
end

function h = add_y_bound_box(ax, y_bound, label_txt, color_in, alpha_val)
    h = [];

    if isempty(y_bound) || numel(y_bound) ~= 2
        return;
    end

    hold(ax, 'on');

    xl = xlim(ax);
    yl = ylim(ax);

    y1 = min(y_bound);
    y2 = max(y_bound);

    if y2 <= min(yl) || y1 >= max(yl)
        return;
    end

    yp1 = max(y1, min(yl));
    yp2 = min(y2, max(yl));

    h = patch(ax, [xl(1) xl(2) xl(2) xl(1)], [yp1 yp1 yp2 yp2], color_in, ...
        'FaceAlpha', alpha_val, ...
        'EdgeColor', 'none', ...
        'LineStyle', 'none', ...
        'HandleVisibility', 'off', ...
        'HitTest', 'off');

    uistack(h, 'bottom');

    text(ax, 0.5 * (xl(1) + xl(2)), 0.5 * (yp1 + yp2), label_txt, ...
        'FontName', 'Times New Roman', ...
        'FontSize', 18, ...
        'FontWeight', 'bold', ...
        'Color', [0.45 0.45 0.45], ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'Clipping', 'on');
end

function h = add_vertical_event_line(ax, t_event, label_txt, color_in, y_frac)
    hold(ax, 'on');

    h = xline(ax, t_event, '--', ...
        'Color', color_in, ...
        'LineWidth', 1.2, ...
        'HandleVisibility', 'off');

    yl = ylim(ax);
    text_y = yl(1) + y_frac * (yl(2) - yl(1));

    text(ax, t_event + 0.12, text_y, label_txt, ...
        'FontName', 'Times New Roman', ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'Color', color_in, ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'middle', ...
        'Clipping', 'on');
end

function xlim_auto_or_cfg(ax, t, tLim)
    if isempty(tLim)
        xlim(ax, [t(1) t(end)]);
    else
        xlim(ax, tLim);
    end
end

function apply_ylim(ax, yLim)
    if isempty(yLim)
        return;
    end

    ylim(ax, yLim);
end

function apply_paper_style(fig)
    set(fig, 'Color', 'w');
    set(fig, 'InvertHardcopy', 'off');

    ax_all = findall(fig, 'Type', 'axes');

    for i = 1:numel(ax_all)
        ax = ax_all(i);

        set(ax, 'Color', 'w');
        set(ax, 'XColor', 'k');
        set(ax, 'YColor', 'k');
        set(ax, 'ZColor', 'k');
        set(ax, 'FontName', 'Times New Roman');
        set(ax, 'FontSize', 10);
        set(ax, 'LineWidth', 0.8);
        set(ax, 'Box', 'on');

        grid(ax, 'on');
    end

    txt_all = findall(fig, 'Type', 'text');

    for i = 1:numel(txt_all)
        set(txt_all(i), 'FontName', 'Times New Roman');
    end

    leg_all = findall(fig, 'Type', 'legend');

    for i = 1:numel(leg_all)
        set(leg_all(i), 'Color', 'w');
        set(leg_all(i), 'TextColor', 'k');
        set(leg_all(i), 'EdgeColor', [0.20 0.20 0.20]);
        set(leg_all(i), 'LineWidth', 0.8);
        set(leg_all(i), 'FontName', 'Times New Roman');
    end
end

function print_map(name, used_name)
    if isempty(used_name)
        fprintf('%-14s : [not found]\n', name);
    else
        fprintf('%-14s : %s\n', name, used_name);
    end
end