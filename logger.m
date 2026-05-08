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

cfg.tLim = []; % [] -> use whole range  

% smoothing alpha
% smaller alpha -> stronger smoothing
cfg.smooth.thrust     = 0.001;
cfg.smooth.tau_des    = 0.01;
cfg.smooth.tau_thrust = 0.03;
cfg.smooth.tau_off    = 0.1;
cfg.smooth.pos        = 0.10;
cfg.smooth.roll_act   = 0.08;
cfg.smooth.roll_des   = 0.05;
cfg.smooth.roll_mrg   = 0.05;
cfg.smooth.arm        = 0.10;
cfg.smooth.cot3d      = 0.10;
cfg.smooth.delta_theta  = 0.001;

% plot offsets
cfg.offset.pos_y      = -0.7;
cfg.offset.roll_mrg   = -3.0;
cfg.offset.roll_des   = -3.0;

% virtual margin on thrust plot
cfg.virtual_margin = 21.0;

% each row: {start_time, end_time, label, color}
cfg.phase.regions = {
  %  80.0,    85.0, 'Hovering',     [0.86 0.91 0.96];
    85.0, 87.8,    'Morphing', [0.93 0.88 0.95];
    104.0,    114.1,    'Translation',  [0.90 0.95 0.90];
};

cfg.phase.alpha = 0.7;

% MPC Activate marker / region
cfg.mpcOn.t = 87.2;
cfg.mpcOn.color = [1.00 0.96 0.78];
cfg.mpcOn.alpha = 0.3;
cfg.mpcOn.label = 'MPC Activate';

% figure / layout
cfg.figPos = [40 60 1450 760];
cfg.leftPos = [0.045 0.085 0.50 0.84];
cfg.arm3d.pos = [0.57 0.30 0.41 0.62];
cfg.delta.pos = [0.57 0.07 0.41 0.18];

% right 3D plot view
cfg.arm3d.view = [38 23];

% actual centered arm position view
cfg.arm3d.daspect = [1 1 60];
cfg.arm3d.pbaspect = [1.0 1.0 1.7];

% actual centered arm plot limit
cfg.arm3d.xyFixedLim = 0.45;

% snapshot options
cfg.arm3d.nSnapshots = 3;
cfg.arm3d.circleDiameter_m = 12.5 * 0.0254;
cfg.arm3d.circleRadius_m = 0.5 * cfg.arm3d.circleDiameter_m;
cfg.arm3d.cotLineWidth = 3.8;
cfg.arm3d.armLineWidth = 1.8;
cfg.arm3d.snapFrameWidth = 1.3;
cfg.arm3d.snapCircleWidth = 1.0;
cfg.arm3d.snapMarkerSize = 28;

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
[F, F_name] = pickField(D, {'f_thrst_con', 'f_thrst', 'f_thrust_con', 'f_thrust'});

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

% tau_x
tau_des_x = ensure_len(pick_col(tau_des, 1), numel(t));
tau_thrust_x = ensure_len(pick_col(tau_thrust, 1), numel(t));
tau_off_x = ensure_len(pick_col(tau_off, 1), numel(t));

% pos y
pos_y = ensure_len(pick_col(pos, 2), numel(t));
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

% frame center is always origin in the 3D plot
frame_cx = row_mean_ignore_nan([r1_x_raw, r2_x_raw, r3_x_raw, r4_x_raw]);
frame_cy = row_mean_ignore_nan([r1_y_raw, r2_y_raw, r3_y_raw, r4_y_raw]);

% centered actual coordinates
r1_x = r1_x_raw - frame_cx;
r2_x = r2_x_raw - frame_cx;
r3_x = r3_x_raw - frame_cx;
r4_x = r4_x_raw - frame_cx;

r1_y = r1_y_raw - frame_cy;
r2_y = r2_y_raw - frame_cy;
r3_y = r3_y_raw - frame_cy;
r4_y = r4_y_raw - frame_cy;

cot_x = cot_x_raw - frame_cx;
cot_y = cot_y_raw - frame_cy;

%% smoothing
F1s = lpf1(F1, cfg.smooth.thrust);
F2s = lpf1(F2, cfg.smooth.thrust);
F3s = lpf1(F3, cfg.smooth.thrust);
F4s = lpf1(F4, cfg.smooth.thrust);

tau_des_x_s = lpf1(tau_des_x, cfg.smooth.tau_des);
tau_thrust_x_s = lpf1(tau_thrust_x, cfg.smooth.tau_thrust);
tau_off_x_s = lpf1(tau_off_x, cfg.smooth.tau_off);

pos_y_s = lpf1(pos_y, cfg.smooth.pos);
pos_d_y_s = lpf1(pos_d_y, cfg.smooth.pos);

roll_act_s = rad2deg(lpf1(roll_act, cfg.smooth.roll_act));
roll_des_s = rad2deg(lpf1(roll_des, cfg.smooth.roll_des));
roll_mrg_s = rad2deg(lpf1(roll_mrg, cfg.smooth.roll_mrg));

% delta roll = des - mrg, no visual offset
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

cot_x_s3d = lpf1(cot_x, cfg.smooth.cot3d);
cot_y_s3d = lpf1(cot_y, cfg.smooth.cot3d);

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


% ----------------------------------------------------------------------------------------------------
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

yline(ax1, cfg.virtual_margin, '--', 'Virtual margin', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');

grid(ax1, 'on');
ylabel(ax1, 'Thrust [N]');
title(ax1, 'Motor thrusts');

xlim_auto_or_cfg(ax1, t, cfg.tLim);
add_phase_regions(ax1, cfg.phase.regions, cfg.phase.alpha);

hmpc1 = add_mpc_on_line(ax1, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([p1 p2 p3 p4 hmpc1], 'top');

legend(ax1, [p1 p2 p3 p4 hmpc1], {'F1', 'F2', 'F3', 'F4', 'MPC Activate'}, 'Location', 'northwest', 'Orientation', 'horizontal', 'Box', 'on');

% 2) TAU_X
ax2 = nexttile(TL, 3, [1 2]);
hold(ax2, 'on');

h1 = plot(ax2, t, tau_des_x_s, '--', 'LineWidth', 1.5, 'Color', C.des);
h2 = plot(ax2, t, tau_des_x_s-tau_off_x_s, 'LineWidth', 1.8, 'Color', C.thrust);
h3 = plot(ax2, t, tau_off_x_s, 'LineWidth', 1.8, 'Color', C.off);

grid(ax2, 'on');
ylabel(ax2, '\tau_x [N m]');
title(ax2, 'Roll torque decomposition');

xlim_auto_or_cfg(ax2, t, cfg.tLim);
add_phase_regions(ax2, cfg.phase.regions, cfg.phase.alpha);

hmpc2 = add_mpc_on_line(ax2, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([h1 h2 h3 hmpc2], 'top');

legend(ax2, [h1 h2 h3 hmpc2], {'\tau_{x,des}', '\tau_{x,thrust}', '\tau_{x,off}', 'MPC Activate'}, 'Location', 'northwest', 'Orientation', 'horizontal', 'Box', 'on');

% 3) POS_Y
ax3 = nexttile(TL, 5);
hold(ax3, 'on');

hp1 = plot(ax3, t, pos_d_y_s + cfg.offset.pos_y, '--', 'LineWidth', 1.5, 'Color', C.des);
hp2 = plot(ax3, t, pos_y_s + cfg.offset.pos_y, 'LineWidth', 1.8, 'Color', C.act);

grid(ax3, 'on');
xlabel(ax3, 'Time [s]');
ylabel(ax3, 'pos_y [m]');
title(ax3, 'Position tracking');

xlim_auto_or_cfg(ax3, t, cfg.tLim);
add_phase_regions(ax3, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax3, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc3 = add_mpc_on_line(ax3, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([hp1 hp2 hmpc3], 'top');

legend(ax3, [hp1 hp2 hmpc3], {'des', 'act', 'MPC Activate'}, 'Location', 'northwest', 'Box', 'on');

% 4) ROLL
ax4 = nexttile(TL, 6);
hold(ax4, 'on');

hr1 = plot(ax4, t, roll_mrg_s + cfg.offset.roll_mrg, 'LineWidth', 1.8, 'Color', C.mrg);
hr2 = plot(ax4, t, roll_act_s, 'LineWidth', 1.8, 'Color', C.act);
hr3 = plot(ax4, t, roll_des_s + cfg.offset.roll_des, '--', 'LineWidth', 1.5, 'Color', C.des);

grid(ax4, 'on');
xlabel(ax4, 'Time [s]');
ylabel(ax4, 'roll [deg]');
title(ax4, 'Roll tracking');

xlim_auto_or_cfg(ax4, t, cfg.tLim);
add_phase_regions(ax4, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax4, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc4 = add_mpc_on_line(ax4, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([hr1 hr2 hr3 hmpc4], 'top');

legend(ax4, [hr1 hr2 hr3 hmpc4], {'mrg', 'act', 'des', 'MPC Activate'}, 'Location', 'northwest', 'Box', 'on');

% 5) 3D ACTUAL CENTERED ARM TRAJECTORY + CoT + SNAPSHOTS
ax5 = axes(fig, 'Position', cfg.arm3d.pos);
hold(ax5, 'on');

a1 = plot3(ax5, r1_x_s, r1_y_s, t, 'LineWidth', cfg.arm3d.armLineWidth, 'Color', C.r1);
a2 = plot3(ax5, r2_x_s, r2_y_s, t, 'LineWidth', cfg.arm3d.armLineWidth, 'Color', C.r2);
a3 = plot3(ax5, r3_x_s, r3_y_s, t, 'LineWidth', cfg.arm3d.armLineWidth, 'Color', C.r3);
a4 = plot3(ax5, r4_x_s, r4_y_s, t, 'LineWidth', cfg.arm3d.armLineWidth, 'Color', C.r4);

valid_cot = ~(isnan(cot_x_s3d) | isnan(cot_y_s3d) | isnan(t));

if any(valid_cot)
    acot = plot3(ax5, cot_x_s3d(valid_cot), cot_y_s3d(valid_cot), t(valid_cot), 'k-', 'LineWidth', cfg.arm3d.cotLineWidth);
else
    acot = plot3(ax5, nan, nan, nan, 'k-', 'LineWidth', cfg.arm3d.cotLineWidth);
end

scatter3(ax5, r1_x_s(1), r1_y_s(1), t(1), 35, C.r1, 'filled', 'HandleVisibility', 'off');
scatter3(ax5, r2_x_s(1), r2_y_s(1), t(1), 35, C.r2, 'filled', 'HandleVisibility', 'off');
scatter3(ax5, r3_x_s(1), r3_y_s(1), t(1), 35, C.r3, 'filled', 'HandleVisibility', 'off');
scatter3(ax5, r4_x_s(1), r4_y_s(1), t(1), 35, C.r4, 'filled', 'HandleVisibility', 'off');

scatter3(ax5, r1_x_s(end), r1_y_s(end), t(end), 55, C.r1, 'd', 'filled', 'HandleVisibility', 'off');
scatter3(ax5, r2_x_s(end), r2_y_s(end), t(end), 55, C.r2, 'd', 'filled', 'HandleVisibility', 'off');
scatter3(ax5, r3_x_s(end), r3_y_s(end), t(end), 55, C.r3, 'd', 'filled', 'HandleVisibility', 'off');
scatter3(ax5, r4_x_s(end), r4_y_s(end), t(end), 55, C.r4, 'd', 'filled', 'HandleVisibility', 'off');

Ns = numel(t);
snap_idx = round(linspace(1, Ns, cfg.arm3d.nSnapshots + 2));
snap_idx = snap_idx(2:end-1);
snap_idx = unique(max(1, min(Ns, snap_idx)));

for k = 1:numel(snap_idx)
    ii = snap_idx(k);

    x_snap = [r1_x_s(ii), r2_x_s(ii), r3_x_s(ii), r4_x_s(ii)];
    y_snap = [r1_y_s(ii), r2_y_s(ii), r3_y_s(ii), r4_y_s(ii)];
    z_snap = t(ii);

    cotx_i = cot_x_s3d(ii);
    coty_i = cot_y_s3d(ii);

    draw_strider_snapshot(ax5, x_snap, y_snap, z_snap, cotx_i, coty_i, cfg.arm3d.circleRadius_m, C, cfg);

    if ~(isnan(cotx_i) || isnan(coty_i))
        text(ax5, cotx_i, coty_i, z_snap, sprintf('  t=%.1f', z_snap), 'FontName', 'Times New Roman', 'FontSize', 8, 'Color', [0.15 0.15 0.15], 'Clipping', 'on');
    end
end

grid(ax5, 'on');
box(ax5, 'on');

xlabel(ax5, 'x [m]');
ylabel(ax5, 'y [m]');
zlabel(ax5, 'Time [s]');
title(ax5, 'Actual centered arm trajectory');

legend(ax5, [a1 a2 a3 a4 acot], {'Arm 1', 'Arm 2', 'Arm 3', 'Arm 4', 'CoT'}, 'Location', 'northeast', 'Box', 'on');

view(ax5, cfg.arm3d.view);
daspect(ax5, cfg.arm3d.daspect);
pbaspect(ax5, cfg.arm3d.pbaspect);

xlim(ax5, [-cfg.arm3d.xyFixedLim cfg.arm3d.xyFixedLim]);
ylim(ax5, [-cfg.arm3d.xyFixedLim cfg.arm3d.xyFixedLim]);
zlim(ax5, [t(1) t(end)]);

set(ax5, 'Color', 'w');
set(ax5, 'XColor', 'k');
set(ax5, 'YColor', 'k');
set(ax5, 'ZColor', 'k');
set(ax5, 'FontName', 'Times New Roman');

% 6) RIGHT BOTTOM DELTA THETA 2-NORM
ax6 = axes(fig, 'Position', cfg.delta.pos);
hold(ax6, 'on');

hd = plot(ax6, t, delta_theta_norm_s, 'LineWidth', 1.8, 'Color', C.delta);

yline(ax6, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');

grid(ax6, 'on');
xlabel(ax6, 'Time [s]');
ylabel(ax6, '||\Delta\theta||_2 [deg]');
title(ax6, 'Attitude reduction: ||rpy_{raw} - rpy_{mrg}||_2');

xlim_auto_or_cfg(ax6, t, cfg.tLim);
add_phase_regions(ax6, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax6, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc6 = add_mpc_on_line(ax6, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([hd hmpc6], 'top');

legend(ax6, [hd hmpc6], {'||\Delta\theta||_2', 'MPC Activate'}, 'Location', 'northeast', 'Box', 'on');

set(ax6, 'Color', 'w');
set(ax6, 'XColor', 'k');
set(ax6, 'YColor', 'k');
set(ax6, 'FontName', 'Times New Roman');

%% Plot Final
apply_paper_style(fig);

fprintf('\n===== Field mapping =====\n');
print_map('thrust', F_name);
print_map('tau_des', tau_des_name);
print_map('tau_thrust', tau_thrust_name);
print_map('tau_off', tau_off_name);
print_map('pos', pos_name);
print_map('pos_d', pos_d_name);
print_map('rpy', rpy_name);
print_map('rpy_raw', rpy_raw_name);
print_map('rpy_d', rpy_d_name);
print_map('r_cot', r_cot_name);
print_map('r_rotor1', r1_name);
print_map('r_rotor2', r2_name);
print_map('r_rotor3', r3_name);
print_map('r_rotor4', r4_name);


%% Add chk plot
% Mean thrust margin check
% margin_i = virtual_margin - F_i
% margin_sum = sum_i margin_i

margin1 = cfg.virtual_margin - F1s;
margin2 = cfg.virtual_margin - F2s;
margin3 = cfg.virtual_margin - F3s;
margin4 = cfg.virtual_margin - F4s;

margin_sum = margin1 + margin2 + margin3 + margin4;

idx_mpc_before = t < cfg.mpcOn.t;
idx_mpc_after = t >= cfg.mpcOn.t;

mean_margin_before = mean(margin_sum(idx_mpc_before), 'omitnan');
mean_margin_after = mean(margin_sum(idx_mpc_after), 'omitnan');

fprintf('\n===== Mean thrust margin check =====\n');
fprintf('Before MPC mean margin = %.3f N\n', mean_margin_before);
fprintf('After  MPC mean margin = %.3f N\n', mean_margin_after);
fprintf('After - Before         = %.3f N\n', mean_margin_after - mean_margin_before);

%% Add plot: delta theta roll/pitch/yaw

rpy_raw_deg = rad2deg(rpy_raw);
rpy_d_deg   = rad2deg(rpy_d);

if isempty(rpy_raw_deg) || size(rpy_raw_deg, 2) < 3 || isempty(rpy_d_deg) || size(rpy_d_deg, 2) < 3
    delta_roll_deg  = nan(numel(t), 1);
    delta_pitch_deg = nan(numel(t), 1);
    delta_yaw_deg   = nan(numel(t), 1);
else
    delta_roll_deg  = rpy_raw_deg(:, 1) - rpy_d_deg(:, 1);
    delta_pitch_deg = rpy_raw_deg(:, 2) - rpy_d_deg(:, 2);
    delta_yaw_deg   = rpy_raw_deg(:, 3) - rpy_d_deg(:, 3);
end

delta_roll_deg_s  = lpf1(delta_roll_deg,  cfg.smooth.delta_theta);
delta_pitch_deg_s = lpf1(delta_pitch_deg, cfg.smooth.delta_theta);
delta_yaw_deg_s   = lpf1(delta_yaw_deg,   cfg.smooth.delta_theta);

fig_delta = figure('Color', 'w', 'Position', [80 80 1050 620]);
set(fig_delta, 'InvertHardcopy', 'off');

TL_delta = tiledlayout(fig_delta, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axd1 = nexttile(TL_delta, 1);
hold(axd1, 'on');
pdr = plot(axd1, t, delta_roll_deg_s, 'LineWidth', 1.8, 'Color', C.F1);
yline(axd1, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');
grid(axd1, 'on');
ylabel(axd1, '\Delta roll [deg]');
title(axd1, '\Delta roll = roll_{raw} - roll_{mrg}');
xlim_auto_or_cfg(axd1, t, cfg.tLim);
add_phase_regions(axd1, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(axd1, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);
hmpcd1 = add_mpc_on_line(axd1, cfg.mpcOn.t, cfg.mpcOn.label);
uistack([pdr hmpcd1], 'top');
legend(axd1, [pdr hmpcd1], {'\Delta roll', 'MPC Activate'}, 'Location', 'northeast', 'Box', 'on');

axd2 = nexttile(TL_delta, 2);
hold(axd2, 'on');
pdp = plot(axd2, t, delta_pitch_deg_s, 'LineWidth', 1.8, 'Color', C.F2);
yline(axd2, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');
grid(axd2, 'on');
ylabel(axd2, '\Delta pitch [deg]');
title(axd2, '\Delta pitch = pitch_{raw} - pitch_{mrg}');
xlim_auto_or_cfg(axd2, t, cfg.tLim);
add_phase_regions(axd2, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(axd2, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);
hmpcd2 = add_mpc_on_line(axd2, cfg.mpcOn.t, cfg.mpcOn.label);
uistack([pdp hmpcd2], 'top');
legend(axd2, [pdp hmpcd2], {'\Delta pitch', 'MPC Activate'}, 'Location', 'northeast', 'Box', 'on');

axd3 = nexttile(TL_delta, 3);
hold(axd3, 'on');
pdy = plot(axd3, t, delta_yaw_deg_s, 'LineWidth', 1.8, 'Color', C.F3);
yline(axd3, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');
grid(axd3, 'on');
xlabel(axd3, 'Time [s]');
ylabel(axd3, '\Delta yaw [deg]');
title(axd3, '\Delta yaw = yaw_{raw} - yaw_{mrg}');
xlim_auto_or_cfg(axd3, t, cfg.tLim);
add_phase_regions(axd3, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(axd3, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);
hmpcd3 = add_mpc_on_line(axd3, cfg.mpcOn.t, cfg.mpcOn.label);
uistack([pdy hmpcd3], 'top');
legend(axd3, [pdy hmpcd3], {'\Delta yaw', 'MPC Activate'}, 'Location', 'northeast', 'Box', 'on');

linkaxes([axd1 axd2 axd3], 'x');

apply_paper_style(fig_delta);

%% local functions!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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

function m = row_mean_ignore_nan(X)
    valid = ~isnan(X);
    cnt = sum(valid, 2);

    Xz = X;
    Xz(~valid) = 0;

    m = sum(Xz, 2) ./ max(cnt, 1);
    m(cnt == 0) = nan;
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

        p = patch(ax, [xp1 xp2 xp2 xp1], [yl(1) yl(1) yl(2) yl(2)], color_in, 'FaceAlpha', alpha_val, 'EdgeColor', 'none', 'HandleVisibility', 'off', 'HitTest', 'off');

        uistack(p, 'bottom');

        text(ax, (xp1+xp2)/2, yl(2)-0.06*(yl(2)-yl(1)), label, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', [0.15 0.25 0.35], 'Clipping', 'on');
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

    p = patch(ax, [x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], color_in, 'FaceAlpha', alpha_in, 'EdgeColor', 'none', 'HandleVisibility', 'off', 'HitTest', 'off');

    uistack(p, 'bottom');
end

function h = add_mpc_on_line(ax, t_on, label_txt)
    hold(ax, 'on');

    h = xline(ax, t_on, '--', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.1, 'HandleVisibility', 'on');

    yl = ylim(ax);

    text_y = yl(1) + 0.10 * (yl(2) - yl(1));

    text(ax, t_on + 0.25, text_y, label_txt, 'FontName', 'Times New Roman', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.20 0.20 0.20], 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'Clipping', 'on');
end

function xlim_auto_or_cfg(ax, t, tLim)
    if isempty(tLim)
        xlim(ax, [t(1) t(end)]);
    else
        xlim(ax, tLim);
    end
end

function draw_strider_snapshot(ax, x4, y4, z0, cotx, coty, r_disk, C, cfg)
    if any(isnan(x4)) || any(isnan(y4)) || isnan(z0)
        return;
    end

    clr = {C.r1, C.r2, C.r3, C.r4};

    x_loop = [x4(1) x4(2) x4(3) x4(4) x4(1)];
    y_loop = [y4(1) y4(2) y4(3) y4(4) y4(1)];
    z_loop = z0 * ones(size(x_loop));

    plot3(ax, x_loop, y_loop, z_loop, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', cfg.arm3d.snapFrameWidth, 'HandleVisibility', 'off');

    for i = 1:4
        draw_circle3_xy(ax, x4(i), y4(i), z0, r_disk, clr{i}, cfg.arm3d.snapCircleWidth);
        scatter3(ax, x4(i), y4(i), z0, cfg.arm3d.snapMarkerSize, clr{i}, 'filled', 'HandleVisibility', 'off');
    end

    scatter3(ax, 0, 0, z0, 24, [0.35 0.35 0.35], 'filled', 'HandleVisibility', 'off');

    if ~(isnan(cotx) || isnan(coty))
        scatter3(ax, cotx, coty, z0, 45, 'k', 'filled', 'HandleVisibility', 'off');
    end
end

function draw_circle3_xy(ax, xc, yc, zc, r, color_in, lw)
    th = linspace(0, 2*pi, 120);
    xx = xc + r*cos(th);
    yy = yc + r*sin(th);
    zz = zc * ones(size(th));

    plot3(ax, xx, yy, zz, '--', 'Color', color_in, 'LineWidth', lw, 'HandleVisibility', 'off');
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