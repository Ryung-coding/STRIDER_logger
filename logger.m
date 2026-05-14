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

% true  -> crop 이후 시작 시간을 0초로 맞춤
% false -> 원래 로그 시간 유지
cfg.time.zero_after_crop = true;

% smoothing alpha
% smaller alpha -> stronger smoothing
cfg.smooth.thrust       = 0.06;
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
cfg.smooth.tau_reduce   = 0.05;

% plot offsets
cfg.offset.pos_y      = -0.7;
cfg.offset.roll_mrg   = -5.0;
cfg.offset.roll_des   = -5.0;

% virtual margin on thrust plot
cfg.virtual_margin = 22;

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
cfg.figPos = [40 60 1450 760];
cfg.mainPos = [0.045 0.085 0.930 0.840];

% geometric controller parameters
param = make_geom_param();

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

% velocity
[vel, vel_name] = pickField(D, {'vel'});
[vel_d, vel_d_name] = pickField(D, {'vel_d', 'vel_des'});

% attitude
[rpy, rpy_name] = pickField(D, {'rpy'});
[rpy_raw, rpy_raw_name] = pickField(D, {'rpy_raw', 'rpy_des_raw'});
[rpy_d, rpy_d_name] = pickField(D, {'rpy_d', 'rpy_des'});

% angular velocity
[omega, omega_name] = pickField(D, {'omega'});
[omega_raw, omega_raw_name] = pickField(D, {'omega_raw', 'omega_des_raw'});
[omega_d, omega_d_name] = pickField(D, {'omega_d', 'omega_des'});

% angular acceleration
[alpha_raw, alpha_raw_name] = pickField(D, {'alpha_raw', 'alpha_des_raw'});
[alpha_d, alpha_d_name] = pickField(D, {'alpha_d', 'alpha_des'});

% cot
[r_cot, r_cot_name] = pickField(D, {'r_cot', 'cot'});

% rotor / arm positions
[r1, r1_name] = pickField(D, {'r_rotor1', 'r1'});
[r2, r2_name] = pickField(D, {'r_rotor2', 'r2'});
[r3, r3_name] = pickField(D, {'r_rotor3', 'r3'});
[r4, r4_name] = pickField(D, {'r_rotor4', 'r4'});

fprintf('\n===== Field map =====\n');
print_map('F', F_name);
print_map('tau_des', tau_des_name);
print_map('tau_thrust', tau_thrust_name);
print_map('tau_off', tau_off_name);
print_map('pos', pos_name);
print_map('pos_d', pos_d_name);
print_map('vel', vel_name);
print_map('vel_d', vel_d_name);
print_map('rpy', rpy_name);
print_map('rpy_raw', rpy_raw_name);
print_map('rpy_d', rpy_d_name);
print_map('omega', omega_name);
print_map('omega_raw', omega_raw_name);
print_map('omega_d', omega_d_name);
print_map('alpha_raw', alpha_raw_name);
print_map('alpha_d', alpha_d_name);
print_map('r_cot', r_cot_name);
print_map('r1', r1_name);
print_map('r2', r2_name);
print_map('r3', r3_name);
print_map('r4', r4_name);

%% time crop

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
vel = crop_if(vel, idx);
vel_d = crop_if(vel_d, idx);
rpy = crop_if(rpy, idx);
rpy_raw = crop_if(rpy_raw, idx);
rpy_d = crop_if(rpy_d, idx);
omega = crop_if(omega, idx);
omega_raw = crop_if(omega_raw, idx);
omega_d = crop_if(omega_d, idx);
alpha_raw = crop_if(alpha_raw, idx);
alpha_d = crop_if(alpha_d, idx);
r_cot = crop_if(r_cot, idx);
r1 = crop_if(r1, idx);
r2 = crop_if(r2, idx);
r3 = crop_if(r3, idx);
r4 = crop_if(r4, idx);

t_crop_start = t(1);

if cfg.time.zero_after_crop
    t = t - t_crop_start;
    cfg = shift_time_config_after_crop(cfg, t_crop_start);
end

N = numel(t);

%% thrust F1~F4

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

%% data arrangement

% tau_x
tau_des_x    = ensure_len(pick_col(tau_des, 1), N);
tau_thrust_x = ensure_len(pick_col(tau_thrust, 1), N);
tau_off_x    = ensure_len(pick_col(tau_off, 1), N);

% pos y
pos_y   = ensure_len(pick_col(pos, 2), N);
pos_d_y = ensure_len(pick_col(pos_d, 2), N);

% attitude
rpy = ensure_mat3(rpy, N);
rpy_raw = ensure_mat3(rpy_raw, N);
rpy_d = ensure_mat3(rpy_d, N);

roll_act = rpy(:, 1);
roll_des = rpy_raw(:, 1);
roll_mrg = rpy_d(:, 1);

if all(isnan(roll_des)) && ~all(isnan(roll_mrg))
    roll_des = roll_mrg;
end

if all(isnan(roll_mrg)) && ~all(isnan(roll_des))
    roll_mrg = roll_des;
end

% angular velocity and angular acceleration
omega = ensure_mat3(omega, N);
omega_raw = ensure_mat3(omega_raw, N);
omega_d = ensure_mat3(omega_d, N);
alpha_raw = ensure_mat3(alpha_raw, N);
alpha_d = ensure_mat3(alpha_d, N);

% fallback if angular velocity fields are missing
if all(isnan(omega), 'all')
    omega = numeric_derivative_mat(rpy, t);
end

if all(isnan(omega_raw), 'all')
    omega_raw = numeric_derivative_mat(rpy_raw, t);
end

if all(isnan(omega_d), 'all')
    omega_d = numeric_derivative_mat(rpy_d, t);
end

% fallback if angular acceleration fields are missing
if all(isnan(alpha_raw), 'all')
    alpha_raw = numeric_derivative_mat(omega_raw, t);
end

if all(isnan(alpha_d), 'all')
    alpha_d = numeric_derivative_mat(omega_d, t);
end

% delta theta command from logged R_raw and R_d:
% Rd_logged = R_raw * exp_hat(d_theta_applied)
dtheta_applied = calc_dtheta_from_Rraw_Rd_series(rpy_raw, rpy_d);

delta_roll_deg = rad2deg(dtheta_applied(:, 1));
delta_pitch_deg = rad2deg(dtheta_applied(:, 2));
delta_yaw_deg = rad2deg(dtheta_applied(:, 3));
delta_theta_norm = sqrt(delta_roll_deg.^2 + delta_pitch_deg.^2 + delta_yaw_deg.^2);

% torque reduction by delta theta:
% tau_raw : attitude_control(R_raw, omega_raw, alpha_raw)
% tau_cut : attitude_control(R_raw * Et', Et * omega_raw, Et * alpha_raw)
% Et      : exp_hat(-dtheta_applied)
[R_raw_series, Rd_cut_series, Wd_cut, Wd_dot_cut] = make_delta_theta_desired_series(rpy_raw, omega_raw, alpha_raw, dtheta_applied);

M_raw = calc_geom_moment_series_from_Rd(rpy, omega, R_raw_series, omega_raw, alpha_raw, param);
M_cut = calc_geom_moment_series_from_Rd(rpy, omega, Rd_cut_series, Wd_cut, Wd_dot_cut, param);

tau_raw_roll = M_raw(:, 1);
tau_cut_roll = M_cut(:, 1);
tau_removed_roll = tau_raw_roll - tau_cut_roll;

% raw arm positions
r1_x_raw = ensure_len(pick_col(r1, 1), N);
r2_x_raw = ensure_len(pick_col(r2, 1), N);
r3_x_raw = ensure_len(pick_col(r3, 1), N);
r4_x_raw = ensure_len(pick_col(r4, 1), N);

r1_y_raw = ensure_len(pick_col(r1, 2), N);
r2_y_raw = ensure_len(pick_col(r2, 2), N);
r3_y_raw = ensure_len(pick_col(r3, 2), N);
r4_y_raw = ensure_len(pick_col(r4, 2), N);

% arm position from data directly
r1_x = r1_x_raw;
r2_x = r2_x_raw;
r3_x = r3_x_raw;
r4_x = r4_x_raw;

r1_y = r1_y_raw;
r2_y = r2_y_raw;
r3_y = r3_y_raw;
r4_y = r4_y_raw;

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

delta_roll_s = lpf1(delta_roll_deg, cfg.smooth.delta_theta);
delta_pitch_s = lpf1(delta_pitch_deg, cfg.smooth.delta_theta);
delta_yaw_s = lpf1(delta_yaw_deg, cfg.smooth.delta_theta);
delta_theta_norm_s = lpf1(delta_theta_norm, cfg.smooth.delta_theta);

tau_removed_roll_s = lpf1(tau_removed_roll, cfg.smooth.tau_reduce);

r1_x_s = lpf1(r1_x, cfg.smooth.arm);
r2_x_s = lpf1(r2_x, cfg.smooth.arm);
r3_x_s = lpf1(r3_x, cfg.smooth.arm);
r4_x_s = lpf1(r4_x, cfg.smooth.arm);

r1_y_s = lpf1(r1_y, cfg.smooth.arm);
r2_y_s = lpf1(r2_y, cfg.smooth.arm);
r3_y_s = lpf1(r3_y, cfg.smooth.arm);
r4_y_s = lpf1(r4_y, cfg.smooth.arm);

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

arm_y_avg = mean([r1_y_mov, r2_y_mov, r3_y_mov, r4_y_mov], 2, 'omitnan');

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
C.tau_reduce = [0.40 0.15 0.75];
C.armAvg = [0.20 0.20 0.20];

%% Fig plot

fig = figure('Color', 'w', 'Position', cfg.figPos);
set(fig, 'InvertHardcopy', 'off');

TL = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
TL.Position = cfg.mainPos;

% -------------------------------------------------------------------------
% 1,1) Motor thrusts
ax1 = nexttile(TL, 1);
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

% -------------------------------------------------------------------------
% 1,2) Delta theta command
ax2 = nexttile(TL, 2);
hold(ax2, 'on');

hd1 = plot(ax2, t, delta_roll_s, 'LineWidth', 1.7, 'Color', C.mrg);
hd2 = plot(ax2, t, delta_pitch_s, 'LineWidth', 1.7, 'Color', C.act);
hd3 = plot(ax2, t, delta_yaw_s, '--', 'LineWidth', 1.4, 'Color', C.des);
hd4 = plot(ax2, t, delta_theta_norm_s, 'LineWidth', 1.8, 'Color', C.delta);

yline(ax2, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');

grid(ax2, 'on');
ylabel(ax2, '\Delta\theta [deg]');
title(ax2, 'Delta theta command');

xlim_auto_or_cfg(ax2, t, cfg.tLim);
add_phase_regions(ax2, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax2, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc2 = add_mpc_on_line(ax2, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([hd1 hd2 hd3 hd4 hmpc2], 'top');

legend(ax2, [hd1 hd2 hd3 hd4], {'\Delta roll', '\Delta pitch', '\Delta yaw', '||\Delta\theta||_2'}, ...
    'Location', 'southwest', 'Box', 'on');

% -------------------------------------------------------------------------
% 2,1) Roll torque
ax3 = nexttile(TL, 3);
hold(ax3, 'on');

ht1 = plot(ax3, t, tau_des_x_s, '--', 'LineWidth', 1.5, 'Color', C.des);
ht2 = plot(ax3, t, tau_des_x_s - tau_off_x_s, 'LineWidth', 1.8, 'Color', C.thrust);
ht3 = plot(ax3, t, tau_off_x_s, 'LineWidth', 1.8, 'Color', C.off);

grid(ax3, 'on');
ylabel(ax3, '\tau_x [N m]');
title(ax3, 'Roll torque decomposition');

xlim_auto_or_cfg(ax3, t, cfg.tLim);
add_phase_regions(ax3, cfg.phase.regions, cfg.phase.alpha);

hmpc3 = add_mpc_on_line(ax3, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([ht1 ht2 ht3 hmpc3], 'top');

legend(ax3, [ht1 ht2 ht3], {'\tau_{x,des}', '\tau_{x,thrust}', '\tau_{x,off}'}, ...
    'Location', 'southwest', 'Orientation', 'vertical', 'Box', 'on');

% -------------------------------------------------------------------------
% 2,2) Removed roll torque by delta theta
ax4 = nexttile(TL, 4);
hold(ax4, 'on');

htr = plot(ax4, t, tau_removed_roll_s, 'LineWidth', 1.8, 'Color', C.tau_reduce);

yline(ax4, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');

grid(ax4, 'on');
ylabel(ax4, '\Delta T_d [N m]');
title(ax4, 'Reduced roll torque by \Delta\theta');

xlim_auto_or_cfg(ax4, t, cfg.tLim);
add_phase_regions(ax4, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax4, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc4 = add_mpc_on_line(ax4, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([htr hmpc4], 'top');

legend(ax4, htr, {'T_{d,raw,x}-T_{d,cut,x}'}, 'Location', 'southwest', 'Box', 'on');

% -------------------------------------------------------------------------
% 3,1) Position command
ax5 = nexttile(TL, 5);
hold(ax5, 'on');

hp1 = plot(ax5, t, pos_d_y_s + cfg.offset.pos_y, '--', 'LineWidth', 1.5, 'Color', C.des);
hp2 = plot(ax5, t, pos_y_s + cfg.offset.pos_y, 'LineWidth', 1.8, 'Color', C.act);

grid(ax5, 'on');
xlabel(ax5, 'Time [s]');
ylabel(ax5, 'y [m]');
title(ax5, 'Position command');

xlim_auto_or_cfg(ax5, t, cfg.tLim);
add_phase_regions(ax5, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax5, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc5 = add_mpc_on_line(ax5, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([hp1 hp2 hmpc5], 'top');

legend(ax5, [hp1 hp2], {'y_d', 'y'}, 'Location', 'southwest', 'Box', 'on');

% -------------------------------------------------------------------------
% 3,2) Arm movement
ax6 = nexttile(TL, 6);
hold(ax6, 'on');

ha1 = plot(ax6, t, r1_y_mov, 'LineWidth', 1.6, 'Color', C.F1);
ha2 = plot(ax6, t, r2_y_mov, 'LineWidth', 1.6, 'Color', C.F2);
ha3 = plot(ax6, t, r3_y_mov, 'LineWidth', 1.6, 'Color', C.F3);
ha4 = plot(ax6, t, r4_y_mov, 'LineWidth', 1.6, 'Color', C.F4);

yline(ax6, 0, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');

grid(ax6, 'on');
xlabel(ax6, 'Time [s]');
ylabel(ax6, '\Delta y [m]');
title(ax6, 'Arm movement');

xlim_auto_or_cfg(ax6, t, cfg.tLim);
add_phase_regions(ax6, cfg.phase.regions, cfg.phase.alpha);
add_mpc_on_region(ax6, cfg.mpcOn.t, cfg.mpcOn.color, cfg.mpcOn.alpha);

hmpc6 = add_mpc_on_line(ax6, cfg.mpcOn.t, cfg.mpcOn.label);

uistack([ha1 ha2 ha3 ha4 hmpc6], 'top');

legend(ax6, [ha1 ha2 ha3 ha4], {'Arm 1', 'Arm 2', 'Arm 3', 'Arm 4'}, 'Location', 'southwest', 'Orientation', 'vertical', 'Box', 'on');

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6], 'x');

apply_paper_style(fig);

%% local functions

function cfg = shift_time_config_after_crop(cfg, t_crop_start)
    if ~isempty(cfg.tLim)
        cfg.tLim = cfg.tLim - t_crop_start;
        cfg.tLim(1) = 0.0;
    end

    if isfield(cfg, 'phase') && isfield(cfg.phase, 'regions') && ~isempty(cfg.phase.regions)
        for i = 1:size(cfg.phase.regions, 1)
            cfg.phase.regions{i, 1} = cfg.phase.regions{i, 1} - t_crop_start;
            cfg.phase.regions{i, 2} = cfg.phase.regions{i, 2} - t_crop_start;
        end
    end

    if isfield(cfg, 'mpcOn') && isfield(cfg.mpcOn, 't')
        cfg.mpcOn.t = cfg.mpcOn.t - t_crop_start;
    end
end

function param = make_geom_param()
    param.kR = [45.0; 45.0; 14.0];
    param.kW = [12.0; 12.0; 3.50];

    JX_BAR = 0.099433;
    JZ_BAR = 0.099433;

    param.J = [
        0.3 + JX_BAR, -0.0006,      -0.0006;
        -0.0006,       0.3,          0.0006;
        -0.0006,       0.0006,       0.5318 + JZ_BAR
    ];

    param.M = 5.425 + (0.575 + 0.42) + 1.0;
    param.G = 9.80665;
end

function dtheta = calc_dtheta_from_Rraw_Rd_series(rpy_raw, rpy_d)
    N = size(rpy_raw, 1);
    dtheta = nan(N, 3);

    for i = 1:N
        if any(isnan(rpy_raw(i, :))) || any(isnan(rpy_d(i, :)))
            continue;
        end

        R_raw = rpy_to_R(rpy_raw(i, 1), rpy_raw(i, 2), rpy_raw(i, 3));
        R_d = rpy_to_R(rpy_d(i, 1), rpy_d(i, 2), rpy_d(i, 3));

        R_delta = R_raw' * R_d;
        dtheta_i = log_SO3(R_delta);

        dtheta(i, :) = dtheta_i(:)';
    end
end

function [R_raw_series, Rd_cut_series, Wd_cut, Wd_dot_cut] = make_delta_theta_desired_series(rpy_raw, omega_raw, alpha_raw, dtheta)
    N = size(rpy_raw, 1);

    R_raw_series = nan(3, 3, N);
    Rd_cut_series = nan(3, 3, N);
    Wd_cut = nan(N, 3);
    Wd_dot_cut = nan(N, 3);

    for i = 1:N
        if any(isnan(rpy_raw(i, :))) || any(isnan(omega_raw(i, :))) || any(isnan(alpha_raw(i, :))) || any(isnan(dtheta(i, :)))
            continue;
        end

        R_raw = rpy_to_R(rpy_raw(i, 1), rpy_raw(i, 2), rpy_raw(i, 3));

        dtheta_i = dtheta(i, :)';
        Et = exp_hat(-dtheta_i);

        Rd_cut = R_raw * Et';
        Wd_i = Et * omega_raw(i, :)';
        Wd_dot_i = Et * alpha_raw(i, :)';

        R_raw_series(:, :, i) = R_raw;
        Rd_cut_series(:, :, i) = Rd_cut;
        Wd_cut(i, :) = Wd_i(:)';
        Wd_dot_cut(i, :) = Wd_dot_i(:)';
    end
end

function M = calc_geom_moment_series_from_Rd(rpy, omega, Rd_series, omega_des, alpha_des, param)
    N = size(rpy, 1);
    M = nan(N, 3);

    for i = 1:N
        if any(isnan(rpy(i, :))) || any(isnan(omega(i, :))) || any(isnan(omega_des(i, :))) || any(isnan(alpha_des(i, :)))
            continue;
        end

        Rd = Rd_series(:, :, i);

        if any(isnan(Rd), 'all')
            continue;
        end

        R = rpy_to_R(rpy(i, 1), rpy(i, 2), rpy(i, 3));

        Om = omega(i, :)';
        Omd = omega_des(i, :)';
        Omd_dot = alpha_des(i, :)';

        eR = 0.5 * vee(Rd' * R - R' * Rd);
        eW = Om - R' * Rd * Omd;

        J = param.J;
        kR = param.kR;
        kW = param.kW;

        M_i = -kR .* eR - kW .* eW + cross(Om, J * Om) - J * (hat(Om) * R' * Rd * Omd - R' * Rd * Omd_dot);

        M(i, :) = M_i(:)';
    end
end

function R = rpy_to_R(roll, pitch, yaw)
    cr = cos(roll);
    sr = sin(roll);
    cp = cos(pitch);
    sp = sin(pitch);
    cy = cos(yaw);
    sy = sin(yaw);

    Rx = [
        1,  0,   0;
        0, cr, -sr;
        0, sr,  cr
    ];

    Ry = [
         cp, 0, sp;
          0, 1,  0;
        -sp, 0, cp
    ];

    Rz = [
        cy, -sy, 0;
        sy,  cy, 0;
         0,   0, 1
    ];

    R = Rz * Ry * Rx;
end

function R = exp_hat(v)
    theta = norm(v);

    if theta < 1e-10
        R = eye(3) + hat(v);
        return;
    end

    K = hat(v / theta);
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * K * K;
end

function v = log_SO3(R)
    c = (trace(R) - 1) / 2;
    c = min(1, max(-1, c));

    theta = acos(c);

    if theta < 1e-10
        v = vee(R - R') / 2;
        return;
    end

    if abs(pi - theta) < 1e-5
        v = vee(logm(R));
        v = real(v);
        return;
    end

    v = theta / (2 * sin(theta)) * vee(R - R');
end

function S = hat(v)
    S = [
          0, -v(3),  v(2);
       v(3),     0, -v(1);
      -v(2),  v(1),     0
    ];
end

function v = vee(S)
    v = [
        S(3, 2);
        S(1, 3);
        S(2, 1)
    ];
end

function Xdot = numeric_derivative_mat(X, t)
    X = double(X);
    N = size(X, 1);
    Xdot = nan(size(X));

    if N < 2
        return;
    end

    for j = 1:size(X, 2)
        x = X(:, j);

        if all(isnan(x))
            continue;
        end

        for i = 1:N
            if i == 1
                dt = t(2) - t(1);
                Xdot(i, j) = (x(2) - x(1)) / dt;
            elseif i == N
                dt = t(N) - t(N-1);
                Xdot(i, j) = (x(N) - x(N-1)) / dt;
            else
                dt = t(i+1) - t(i-1);
                Xdot(i, j) = (x(i+1) - x(i-1)) / dt;
            end
        end
    end
end

function X = ensure_mat3(X, n)
    if isempty(X) || ~isnumeric(X)
        X = nan(n, 3);
        return;
    end

    X = double(X);

    if isvector(X)
        tmp = nan(n, 3);
        x = X(:);
        m = min(numel(x), n);
        tmp(1:m, 1) = x(1:m);
        X = tmp;
        return;
    end

    if size(X, 1) > n
        X = X(1:n, :);
    elseif size(X, 1) < n
        tmp = nan(n, size(X, 2));
        tmp(1:size(X, 1), :) = X;
        X = tmp;
    end

    if size(X, 2) < 3
        tmp = nan(n, 3);
        tmp(:, 1:size(X, 2)) = X;
        X = tmp;
    elseif size(X, 2) > 3
        X = X(:, 1:3);
    end
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