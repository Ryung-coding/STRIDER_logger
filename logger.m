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

%% file

npz_file = fullfile(pwd, 'log', 'take22.npz');

if ~isfile(npz_file)
    error('NPZ file not found:\n%s', npz_file);
end

%% Config

cfg = struct();

cfg.tLim = [70 114];   % [] -> use whole range

% smoothing alpha
% smaller alpha -> stronger smoothing
cfg.smooth.thrust       = 0.01;
cfg.smooth.tau_des      = 0.01;
cfg.smooth.tau_thrust   = 0.03;
cfg.smooth.tau_off      = 0.99;
cfg.smooth.pos          = 0.10;
cfg.smooth.roll_act     = 0.08;
cfg.smooth.roll_des     = 0.05;
cfg.smooth.roll_mrg     = 0.05;
cfg.smooth.arm          = 1.0;
cfg.smooth.cot          = 0.05;
cfg.smooth.delta_theta  = 0.9;

% plot offsets
cfg.offset.pos_y      = -0.7;
cfg.offset.roll_mrg   = -5.0;
cfg.offset.roll_des   = -5.0;

% virtual margin on thrust plot
cfg.virtual_margin = 20;

% each row: {start_time, end_time, label, color}
cfg.phase.regions = {
    87.2,    89.0,  'Morphing',     [0.93 0.88 0.95];
    99.0,   101.0,  'Translation',  [0.90 0.95 0.90];
    110.0,  112.0,  'Translation',  [0.90 0.95 0.90];
};

cfg.phase.alpha = 0.7;

% MPC Activate marker / region
cfg.mpcOn.t = 87.2;
cfg.mpcOn.color = [1.00 0.96 0.78];
cfg.mpcOn.alpha = 0.3;
cfg.mpcOn.label = 'MPC Activate';

% figure / layout
cfg.figPos   = [40 60 1450 760];
cfg.leftPos  = [0.045 0.085 0.50 0.84];
cfg.rightPos = [0.57 0.07 0.41 0.85];

% rotor 2D map options
cfg.xyMap.snapshot_dt      = 2.0;   % seconds
cfg.xyMap.circle_radius_mm = 12;    % snapshot circle radius
cfg.xyMap.face_alpha_min   = 0.05;
cfg.xyMap.face_alpha_max   = 0.22;
cfg.xyMap.edge_alpha_min   = 0.12;
cfg.xyMap.edge_alpha_max   = 0.45;
cfg.xyMap.line_width       = 1.4;
cfg.xyMap.wide_aspect      = [2.6 1.0 1.0];   % horizontal long rectangle
cfg.xyMap.xpad_mm          = 35;
cfg.xyMap.ypad_mm          = 18;

%% Load data
D = load_npz_all(npz_file);

disp('===== Loaded fields =====');
disp(fieldnames(D));

[t, t_name] = pickField(D, {'t', 'time'});

if isempty(t)
    error('Time field not found.');
end

t = t(:);

fprintf('\nTime field : %s\n', t_name);
fprintf('Time range : %.3f ~ %.3f s\n', t(1), t(end));

%% select data for plot

% thrust
[F, F_name] = pickField(D, {'f_thrst', 'f_thrst_con', 'f_thrust', 'f_thrust_con'});

if isempty(F)
    warning('No thrust field found.');
    F = nan(numel(t), 4);
end

% torque
[tau_des, tau_des_name] = pickField(D, {'tau_d', 'tau_des'});
[tau_thrust, tau_thrust_name] = pickField(D, {'tau_thrust'});
[tau_off, tau_off_name] = pickField(D, {'tau_off'});

% position
[pos, pos_name] = pickField(D, {'pos'});
[pos_d, pos_d_name] = pickField(D, {'pos_d', 'pos_des'});

% attitude
[rpy, rpy_name] = pickField(D, {'rpy'});
[rpy_raw, rpy_raw_name] = pickField(D, {'rpy_raw', 'rpy_des_raw'});
[rpy_d, rpy_d_name] = pickField(D, {'rpy_d', 'rpy_des'});

% cot
[r_cot, r_cot_name] = pickField(D, {'r_cot', 'cot'});

% rotor / arm positions
[r1, r1_name] = pickField(D, {'r_rotor1', 'r1'});
[r2, r2_name] = pickField(D, {'r_rotor2', 'r2'});
[r3, r3_name] = pickField(D, {'r_rotor3', 'r3'});
[r4, r4_name] = pickField(D, {'r_rotor4', 'r4'});

% time setting
if isempty(cfg.tLim)
    idx = true(size(t));
else
    idx = (t >= cfg.tLim(1)) & (t <= cfg.tLim(2));
end

t = t(idx);

fprintf('After crop: N = %d\n', numel(t));

if isempty(t)
    error('No data after time crop. Check cfg.tLim.');
end

F = crop_if(F, idx);
tau_des = crop_if(tau_des, idx);
tau_thrust = crop_if(tau_thrust, idx);
tau_off = crop_if(tau_off, idx);
pos = crop_if(pos, idx);
pos_d = crop_if(pos_d, idx);
rpy = crop_if(rpy, idx);
rpy_raw = crop_if(rpy_raw, idx);
rpy_d = crop_if(rpy_d, idx);
r_cot = crop_if(r_cot, idx);
r1 = crop_if(r1, idx);
r2 = crop_if(r2, idx);
r3 = crop_if(r3, idx);
r4 = crop_if(r4, idx);

% thrust F1~F4
if size(F, 2) >= 4
    F1 = F(:, 1);
    F2 = F(:, 2);
    F3 = F(:, 3);
    F4 = F(:, 4);
else
    F1 = nan(size(t));
    F2 = F1;
    F3 = F1;
    F4 = F1;
end

%% thrust sum offset correction
F_sum_raw = F1 + F2 + F3 + F4;
F_sum_target = median(F_sum_raw, 'omitnan');

F_sum_err = F_sum_raw - F_sum_target;
F_sum_err(F_sum_err < 0) = 0;

F_common_offset = F_sum_err / 4.0;

F1_raw = F1;
F2_raw = F2;
F3_raw = F3;
F4_raw = F4;

F1 = F1 - F_common_offset;
F2 = F2 - F_common_offset;
F3 = F3 - F_common_offset;
F4 = F4 - F_common_offset;

fprintf('\n===== Thrust sum offset correction =====\n');
fprintf('Median raw Fsum      = %.3f N\n', F_sum_target);
fprintf('Mean positive offset = %.3f N / motor\n', mean(F_common_offset, 'omitnan'));
fprintf('Max positive offset  = %.3f N / motor\n', max(F_common_offset, [], 'omitnan'));

% tau_x
tau_des_x    = ensure_len(pick_col(tau_des, 1), numel(t));
tau_thrust_x = ensure_len(pick_col(tau_thrust, 1), numel(t));
tau_off_x    = ensure_len(pick_col(tau_off, 1), numel(t));

% pos y
pos_y   = ensure_len(pick_col(pos, 2), numel(t));
pos_d_y = ensure_len(pick_col(pos_d, 2), numel(t));

% roll
roll_act = ensure_len(pick_col(rpy, 1), numel(t));
roll_des = ensure_len(pick_col(rpy_raw, 1), numel(t));
roll_mrg = ensure_len(pick_col(rpy_d, 1), numel(t));

if all(isnan(roll_des)) && ~all(isnan(roll_mrg))
    roll_des = roll_mrg;
end

if all(isnan(roll_mrg)) && ~all(isnan(roll_des))
    roll_mrg = roll_des;
end

% raw arm positions
r1_x_raw = ensure_len(pick_col(r1, 1), numel(t));
r2_x_raw = ensure_len(pick_col(r2, 1), numel(t));
r3_x_raw = ensure_len(pick_col(r3, 1), numel(t));
r4_x_raw = ensure_len(pick_col(r4, 1), numel(t));

r1_y_raw = ensure_len(pick_col(r1, 2), numel(t));
r2_y_raw = ensure_len(pick_col(r2, 2), numel(t));
r3_y_raw = ensure_len(pick_col(r3, 2), numel(t));
r4_y_raw = ensure_len(pick_col(r4, 2), numel(t));

% raw CoT
cot_x_raw = ensure_len(pick_col(r_cot, 1), numel(t));
cot_y_raw = ensure_len(pick_col(r_cot, 2), numel(t));

% Arm position from data directly
r1_x = r1_x_raw;
r2_x = r2_x_raw;
r3_x = r3_x_raw;
r4_x = r4_x_raw;

r1_y = r1_y_raw;
r2_y = r2_y_raw;
r3_y = r3_y_raw;
r4_y = r4_y_raw;

% CoT from data directly
cot_x = cot_x_raw;
cot_y = cot_y_raw;

%% smoothing
F1s = lpf1(F1, cfg.smooth.thrust);
F2s = lpf1(F2, cfg.smooth.thrust);
F3s = lpf1(F3, cfg.smooth.thrust);
F4s = lpf1(F4, cfg.smooth.thrust);

tau_des_x_s    = lpf1(tau_des_x, cfg.smooth.tau_des);
tau_thrust_x_s = lpf1(tau_thrust_x, cfg.smooth.tau_thrust);
tau_off_x_s    = lpf1(tau_off_x, cfg.smooth.tau_off);

pos_y_s   = lpf1(pos_y, cfg.smooth.pos);
pos_d_y_s = lpf1(pos_d_y, cfg.smooth.pos);

roll_act_s = rad2deg(lpf1(roll_act, cfg.smooth.roll_act));
roll_des_s = rad2deg(lpf1(roll_des, cfg.smooth.roll_des));
roll_mrg_s = rad2deg(lpf1(roll_mrg, cfg.smooth.roll_mrg));

% delta theta 2-norm
rpy_raw_deg = rad2deg(rpy_raw);
rpy_d_deg   = rad2deg(rpy_d);

if isempty(rpy_raw_deg) || size(rpy_raw_deg, 2) < 3 || isempty(rpy_d_deg) || size(rpy_d_deg, 2) < 3
    delta_theta_norm = nan(numel(t), 1);
else
    delta_rpy_deg = rpy_raw_deg(:, 1:3) - rpy_d_deg(:, 1:3);
    delta_theta_norm = sqrt(sum(delta_rpy_deg.^2, 2));
end

delta_theta_norm_s = lpf1(delta_theta_norm, cfg.smooth.delta_theta);

r1_x_s = lpf1(r1_x, cfg.smooth.arm);
r2_x_s = lpf1(r2_x, cfg.smooth.arm);
r3_x_s = lpf1(r3_x, cfg.smooth.arm);
r4_x_s = lpf1(r4_x, cfg.smooth.arm);

r1_y_s = lpf1(r1_y, cfg.smooth.arm);
r2_y_s = lpf1(r2_y, cfg.smooth.arm);
r3_y_s = lpf1(r3_y, cfg.smooth.arm);
r4_y_s = lpf1(r4_y, cfg.smooth.arm);

cot_x_s = lpf1(cot_x, cfg.smooth.cot);
cot_y_s = lpf1(cot_y, cfg.smooth.cot);

% y arm offset removal by +/-0.2379 only
r1_y_ref = 0.2379 * sign(r1_y_s(1));
r2_y_ref = 0.2379 * sign(r2_y_s(1));
r3_y_ref = 0.2379 * sign(r3_y_s(1));
r4_y_ref = 0.2379 * sign(r4_y_s(1));

if r1_y_ref == 0, r1_y_ref = 0.2379; end
if r2_y_ref == 0, r2_y_ref = 0.2379; end
if r3_y_ref == 0, r3_y_ref = -0.2379; end
if r4_y_ref == 0, r4_y_ref = -0.2379; end

r1_y_mov = r1_y_s - r1_y_ref;
r2_y_mov = r2_y_s - r2_y_ref;
r3_y_mov = r3_y_s - r3_y_ref;
r4_y_mov = r4_y_s - r4_y_ref;

%% color setting
C.F1 = [0.00 0.35 0.85];
C.F2 = [0.85 0.15 0.15];
C.F3 = [0.15 0.70 0.35];
C.F4 = [0.90 0.65 0.10];

C.des = [0.10 0.10 0.10];
C.act = [0.10 0.35 0.85];
C.mrg = [0.90 0.15 0.15];

C.off = [0.85 0.35 0.65];
C.thrust = [0.20 0.45 0.90];
C.delta = [0.20 0.20 0.20];

C.r1 = C.F1;
C.r2 = C.F2;
C.r3 = C.F3;
C.r4 = C.F4;

%% Fig plot
fig = figure('Color', 'w', 'Position', cfg.figPos);
set(fig, 'InvertHardcopy', 'off');

% left tiled plots
TL = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
TL.Position = cfg.leftPos;

% 1) Thrust
ax1 = nexttile(TL, 1, [1 2]);
hold(ax1, 'on');

p1 = plot(ax1, t, F1s, 'LineWidth', 1.8, 'Color', C.F1);
p2 = plot(ax1, t, F2s, 'LineWidth', 1.8, 'Color', C.F2);
p3 = plot(ax1, t, F3s, 'LineWidth', 1.8, 'Color', C.F3);
p4 = plot(ax1, t, F4s, 'LineWidth', 1.8, 'Color', C.F4);

yline(ax1, cfg.virtual_margin, '--', 'Virtual margin', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.5, ...
    'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');

grid(ax1, 'on');
ylabel(ax1, 'Thrust [N]');
title(ax1, 'Motor thrusts');

xlim_auto_or_cfg(ax1, t, cfg.tLim);
add_phase_regions(ax1, cfg.phase.regions, cfg.phase.alpha);

hmpc1 = add_mpc_on_line(ax1, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([p1 p2 p3 p4 hmpc1], 'top');

legend(ax1, [p1 p2 p3 p4], {'F1', 'F2', 'F3', 'F4'}, 'Location', 'southeast', 'Orientation', 'horizontal', 'Box', 'on');

% 2) TAU_X
ax2 = nexttile(TL, 3, [1 2]);
hold(ax2, 'on');

h1 = plot(ax2, t, tau_des_x_s, '--', 'LineWidth', 1.5, 'Color', C.des);
h2 = plot(ax2, t, tau_des_x_s - tau_off_x_s, 'LineWidth', 1.8, 'Color', C.thrust);
h3 = plot(ax2, t, tau_off_x_s, 'LineWidth', 1.8, 'Color', C.off);

grid(ax2, 'on');
ylabel(ax2, '\tau_x [N m]');
title(ax2, 'Roll torque decomposition');

xlim_auto_or_cfg(ax2, t, cfg.tLim);
add_phase_regions(ax2, cfg.phase.regions, cfg.phase.alpha);

hmpc2 = add_mpc_on_line(ax2, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([h1 h2 h3 hmpc2], 'top');

legend(ax2, [h1 h2 h3], {'\tau_{x,des}', '\tau_{x,thrust}', '\tau_{x,off}'}, ...
    'Location', 'southwest', 'Orientation', 'vertical', 'Box', 'on');

% 3) ATTITUDE
ax3 = nexttile(TL, 5);
hold(ax3, 'on');

hr1 = plot(ax3, t, roll_mrg_s + cfg.offset.roll_mrg, 'LineWidth', 1.8, 'Color', C.mrg);
hr2 = plot(ax3, t, roll_act_s, 'LineWidth', 1.8, 'Color', C.act);
hr3 = plot(ax3, t, roll_des_s + cfg.offset.roll_des, '--', 'LineWidth', 1.5, 'Color', C.des);

grid(ax3, 'on');
xlabel(ax3, 'Time [s]');
ylabel(ax3, 'roll [deg]');
title(ax3, 'Roll tracking');

xlim_auto_or_cfg(ax3, t, cfg.tLim);
add_phase_regions(ax3, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax3, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc3 = add_mpc_on_line(ax3, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([hr1 hr2 hr3 hmpc3], 'top');

legend(ax3, [hr1 hr2 hr3], {'mrg', 'act', 'des'}, 'Location', 'southwest', 'Box', 'on');

% 4) DELTA THETA 2-NORM
ax4 = nexttile(TL, 6);
hold(ax4, 'on');

hd = plot(ax4, t, delta_theta_norm_s, 'LineWidth', 1.8, 'Color', C.delta);

yline(ax4, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');

grid(ax4, 'on');
xlabel(ax4, 'Time [s]');
ylabel(ax4, '||\Delta\theta||_2 [deg]');
title(ax4, 'Attitude reduction');

xlim_auto_or_cfg(ax4, t, cfg.tLim);
add_phase_regions(ax4, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax4, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc4 = add_mpc_on_line(ax4, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([hd hmpc4], 'top');

% 5) RIGHT SIDE: Rotor 2D map, Arm y change, CoT
right_left   = cfg.rightPos(1);
right_bottom = cfg.rightPos(2);
right_width  = cfg.rightPos(3);
right_total_height = cfg.rightPos(4);

right_gap    = 0.035;
right_height = (right_total_height - 2 * right_gap) / 3.0;
right_top    = right_bottom + right_total_height;

pos_cot   = [right_left, right_bottom, right_width, right_height];
pos_yarm  = [right_left, right_bottom + right_height + right_gap, right_width, right_height];
pos_xymap = [right_left, right_bottom + 2 * (right_height + right_gap), right_width, right_height];

% 5-1) Rotor XY map
% horizontal axis = y [mm], vertical axis = x [mm]
ax5 = axes(fig, 'Position', pos_xymap);
hold(ax5, 'on');

plot_rotor_xy_with_snapshots(ax5, t, 1000*r1_y_s, 1000*r1_x_s, C.r1, cfg.xyMap);
plot_rotor_xy_with_snapshots(ax5, t, 1000*r2_y_s, 1000*r2_x_s, C.r2, cfg.xyMap);
plot_rotor_xy_with_snapshots(ax5, t, 1000*r3_y_s, 1000*r3_x_s, C.r3, cfg.xyMap);
plot_rotor_xy_with_snapshots(ax5, t, 1000*r4_y_s, 1000*r4_x_s, C.r4, cfg.xyMap);

scatter(ax5, 1000*r1_y_s(1), 1000*r1_x_s(1), 26, C.r1, 'filled', 'HandleVisibility', 'off');
scatter(ax5, 1000*r2_y_s(1), 1000*r2_x_s(1), 26, C.r2, 'filled', 'HandleVisibility', 'off');
scatter(ax5, 1000*r3_y_s(1), 1000*r3_x_s(1), 26, C.r3, 'filled', 'HandleVisibility', 'off');
scatter(ax5, 1000*r4_y_s(1), 1000*r4_x_s(1), 26, C.r4, 'filled', 'HandleVisibility', 'off');

scatter(ax5, 1000*r1_y_s(end), 1000*r1_x_s(end), 26, C.r1, 'o', 'LineWidth', 1.2, 'HandleVisibility', 'off');
scatter(ax5, 1000*r2_y_s(end), 1000*r2_x_s(end), 26, C.r2, 'o', 'LineWidth', 1.2, 'HandleVisibility', 'off');
scatter(ax5, 1000*r3_y_s(end), 1000*r3_x_s(end), 26, C.r3, 'o', 'LineWidth', 1.2, 'HandleVisibility', 'off');
scatter(ax5, 1000*r4_y_s(end), 1000*r4_x_s(end), 26, C.r4, 'o', 'LineWidth', 1.2, 'HandleVisibility', 'off');

all_y_mm = [1000*r1_y_s; 1000*r2_y_s; 1000*r3_y_s; 1000*r4_y_s];
all_x_mm = [1000*r1_x_s; 1000*r2_x_s; 1000*r3_x_s; 1000*r4_x_s];

xmin = min(all_y_mm, [], 'omitnan') - cfg.xyMap.xpad_mm;
xmax = max(all_y_mm, [], 'omitnan') + cfg.xyMap.xpad_mm;
ymin = min(all_x_mm, [], 'omitnan') - cfg.xyMap.ypad_mm;
ymax = max(all_x_mm, [], 'omitnan') + cfg.xyMap.ypad_mm;

grid(ax5, 'on');
xlabel(ax5, 'y [mm]');
ylabel(ax5, 'x [mm]');
title(ax5, 'Rotor movement map');

xlim(ax5, [xmin xmax]);
ylim(ax5, [ymin ymax]);
pbaspect(ax5, cfg.xyMap.wide_aspect);

legend(ax5, {'Arm1', 'Arm2', 'Arm3', 'Arm4'}, 'Location', 'northeast', 'NumColumns', 2, 'Box', 'on');

% 5-2) Arm y change from +/-0.2379
ax6 = axes(fig, 'Position', pos_yarm);
hold(ax6, 'on');

hy1 = plot(ax6, t, r1_y_mov, 'LineWidth', 1.6, 'Color', C.r1);
hy2 = plot(ax6, t, r2_y_mov, 'LineWidth', 1.6, 'Color', C.r2);
hy3 = plot(ax6, t, r3_y_mov, 'LineWidth', 1.6, 'Color', C.r3);
hy4 = plot(ax6, t, r4_y_mov, 'LineWidth', 1.6, 'Color', C.r4);

yline(ax6, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');

grid(ax6, 'on');
ylabel(ax6, '\Delta y from \pm0.2379 [m]');
title(ax6, 'Arm y change');

xlim_auto_or_cfg(ax6, t, cfg.tLim);
add_phase_regions(ax6, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax6, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc6 = add_mpc_on_line(ax6, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([hy1 hy2 hy3 hy4 hmpc6], 'top');

legend(ax6, [hy1 hy2 hy3 hy4], {'Arm1 y', 'Arm2 y', 'Arm3 y', 'Arm4 y'}, ...
    'Location', 'southwest', 'NumColumns', 2, 'Box', 'on');

% 5-3) CoT position
ax7 = axes(fig, 'Position', pos_cot);
hold(ax7, 'on');

hc1 = plot(ax7, t, cot_x_s, 'LineWidth', 1.8, 'Color', [0.10 0.10 0.10]);
hc2 = plot(ax7, t, cot_y_s, '--', 'LineWidth', 1.8, 'Color', [0.45 0.45 0.45]);

yline(ax7, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');

grid(ax7, 'on');
xlabel(ax7, 'Time [s]');
ylabel(ax7, 'CoT [m]');
title(ax7, 'CoT position');

xlim_auto_or_cfg(ax7, t, cfg.tLim);
add_phase_regions(ax7, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax7, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc7 = add_mpc_on_line(ax7, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([hc1 hc2 hmpc7], 'top');

legend(ax7, [hc1 hc2], {'CoT x', 'CoT y'}, 'Location', 'southwest', 'Box', 'on');

linkaxes([ax1 ax2 ax3 ax4 ax6 ax7], 'x');

set(ax5, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'FontName', 'Times New Roman');
set(ax6, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'FontName', 'Times New Roman');
set(ax7, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'FontName', 'Times New Roman');

apply_paper_style(fig);

%% local functions

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

function add_phase_regions(ax, regions, alpha_val)
    if isempty(regions)
        return;
    end

    hold(ax, 'on');
    xl = xlim(ax);
    yl = ylim(ax);

    for i = 1:size(regions, 1)
        x1 = regions{i, 1};
        x2 = regions{i, 2};
        label = regions{i, 3};
        color_in = regions{i, 4};

        if x2 <= xl(1) || x1 >= xl(2)
            continue;
        end

        xp1 = max(x1, xl(1));
        xp2 = min(x2, xl(2));

        p = patch(ax, [xp1 xp2 xp2 xp1], [yl(1) yl(1) yl(2) yl(2)], color_in, ...
            'FaceAlpha', alpha_val, 'EdgeColor', 'none', 'HandleVisibility', 'off', 'HitTest', 'off');

        uistack(p, 'bottom');

        text(ax, (xp1+xp2)/2, yl(2)-0.06*(yl(2)-yl(1)), label, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'FontWeight', 'bold', 'FontName', 'Times New Roman', ...
            'Color', [0.15 0.25 0.35], 'Clipping', 'on');
    end
end

function add_mpc_on_region(ax, t_on, color_in, alpha_in)
    xl = xlim(ax);
    yl = ylim(ax);

    x1 = max(t_on, xl(1));
    x2 = xl(2);

    if x2 <= x1
        return;
    end

    p = patch(ax, [x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], color_in, ...
        'FaceAlpha', alpha_in, 'EdgeColor', 'none', 'HandleVisibility', 'off', 'HitTest', 'off');

    uistack(p, 'bottom');
end

function h = add_mpc_on_line(ax, t_on, label_txt)
    hold(ax, 'on');

    h = xline(ax, t_on, '--', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.1, 'HandleVisibility', 'on');

    yl = ylim(ax);
    text_y = yl(1) + 0.10 * (yl(2) - yl(1));

    text(ax, t_on + 0.25, text_y, label_txt, ...
        'FontName', 'Times New Roman', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Color', [0.20 0.20 0.20], 'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'middle', 'Clipping', 'on');
end

function xlim_auto_or_cfg(ax, t, tLim)
    if isempty(tLim)
        xlim(ax, [t(1) t(end)]);
    else
        xlim(ax, tLim);
    end
end

function plot_rotor_xy_with_snapshots(ax, t, y_mm, x_mm, color_in, cfg_xy)
    if isempty(t) || isempty(y_mm) || isempty(x_mm)
        return;
    end

    plot(ax, y_mm, x_mm, '-', 'LineWidth', cfg_xy.line_width, 'Color', color_in, 'HandleVisibility', 'off');

    idx_snap = make_time_snap_indices(t, cfg_xy.snapshot_dt);

    if isempty(idx_snap)
        return;
    end

    nS = numel(idx_snap);

    for k = 1:nS
        ii = idx_snap(k);

        if isnan(y_mm(ii)) || isnan(x_mm(ii))
            continue;
        end

        if nS == 1
            s = 1.0;
        else
            s = (k-1) / (nS-1);
        end

        fa = cfg_xy.face_alpha_min + s * (cfg_xy.face_alpha_max - cfg_xy.face_alpha_min);
        ea = cfg_xy.edge_alpha_min + s * (cfg_xy.edge_alpha_max - cfg_xy.edge_alpha_min);

        draw_circle2(ax, y_mm(ii), x_mm(ii), cfg_xy.circle_radius_mm, color_in, fa, ea);
    end
end

function idx_snap = make_time_snap_indices(t, dt)
    if isempty(t) || dt <= 0
        idx_snap = [];
        return;
    end

    t0 = t(1);
    tf = t(end);
    t_snap = t0:dt:tf;

    idx_snap = zeros(size(t_snap));

    for i = 1:numel(t_snap)
        [~, idx_snap(i)] = min(abs(t - t_snap(i)));
    end

    idx_snap = unique(idx_snap(:)');
end

function draw_circle2(ax, xc, yc, r, color_in, face_alpha_in, edge_alpha_in)
    th = linspace(0, 2*pi, 100);
    xx = xc + r*cos(th);
    yy = yc + r*sin(th);

    p = patch(ax, xx, yy, color_in, ...
        'FaceAlpha', face_alpha_in, ...
        'EdgeColor', color_in, ...
        'LineWidth', 0.8, ...
        'HandleVisibility', 'off');

    if isprop(p, 'EdgeAlpha')
        p.EdgeAlpha = edge_alpha_in;
    end
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
        set(txt_all(i), 'Color', 'k');
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
        fprintf('%-12s : [not found]\n', name);
    else
        fprintf('%-12s : %s\n', name, used_name);
    end
end