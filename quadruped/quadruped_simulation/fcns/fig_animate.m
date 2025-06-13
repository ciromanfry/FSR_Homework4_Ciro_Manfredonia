function [t,EA,EAd] = fig_animate(tout,Xout,Uout,Xdout,Udout,Uext,p)

load('gait.mat');
if gait==0
    name_pos = 'position_plot_0_mod_fin.pdf';
    name_vel = 'velocity_plot_0_mod_fin.pdf';
    name_ang_vel = 'angular_velocity_plot_0_mod_fin.pdf';
    name_cnt = 'control_force_plot_0_mod_fin.pdf';
    name_rob = 'robot_last_frame_0_mod_fin.png';
elseif gait == 1
    name_pos = 'position_plot_1_mod_fin.pdf';
    name_vel = 'velocity_plot_1_mod_fin.pdf';
    name_ang_vel = 'angular_velocity_plot_1_mod_fin.pdf';
    name_cnt = 'control_force_plot_1_mod_fin.pdf';
    name_rob = 'robot_last_frame_1_mod_fin.png';
elseif gait == 2
    name_pos = 'position_plot_2_mod_fin.pdf';
    name_vel = 'velocity_plot_2_mod_fin.pdf';
    name_ang_vel = 'angular_velocity_plot_2_mod_fin.pdf';
    name_cnt = 'control_force_plot_2_mod_fin.pdf';
    name_rob = 'robot_last_frame_2_mod_fin.png';
elseif gait == 3
    name_pos = 'position_plot_3_mod_fin.pdf';
    name_vel = 'velocity_plot_3_mod_fin.pdf';
    name_ang_vel = 'angular_velocity_plot_3_mod_fin.pdf';
    name_cnt = 'control_force_plot_3_mod_fin.pdf';
    name_rob = 'robot_last_frame_3_mod_fin.png';
elseif gait == 4
    name_pos = 'position_plot_4_mod_fin.pdf';
    name_vel = 'velocity_plot_4_mod_fin.pdf';
    name_ang_vel = 'angular_velocity_plot_4_mod_fin.pdf';
    name_cnt = 'control_force_plot_4_mod_fin.pdf';
    name_rob = 'robot_last_frame_4_mod_fin.png';
elseif gait == 5
    name_pos = 'position_plot_5_mod_fin.pdf';
    name_vel = 'velocity_plot_5_mod_fin.pdf';
    name_ang_vel = 'angular_velocity_plot_5_mod_fin.pdf';
    name_cnt = 'control_force_plot_5_mod_fin.pdf';
    name_rob = 'robot_last_frame_5_mod_fin.png';
end

flag_movie = p.flag_movie;

if flag_movie
    try
        name = 'test.mp4';
        vidfile = VideoWriter(name,'MPEG-4');
    catch
        name = 'test';
        vidfile = VideoWriter(name,'Motion JPEG AVI');
    end
    open(vidfile);
end

%% smoothen for animation
t = (tout(1):p.simTimeStep:tout(end));
X = interp1(tout,Xout,t);
U = interp1(tout,Uout,t);
Xd = interp1(tout,Xdout,t);
Ud = interp1(tout,Udout,t);
Ue = interp1(tout,Uext,t);

nt = length(t);
EA = zeros(nt,3);
EAd = zeros(nt,3);
for ii = 1:nt
    EA(ii,:) = fcn_X2EA(X(ii,:));
    EAd(ii,:) = fcn_X2EA(Xd(ii,:));
end

% For ZOH force plot
t2 = repelem(t,2);
t2(1) = [];
t2(end+1) = t2(end);
U2 = repelem(U,2,1);

% Create figure for robot animation
fig_robot = figure('Name', 'Robot Movement', 'NumberTitle', 'off');

% === Robot Animation Loop ===
for ii = 1:p.playSpeed:nt
    figure(fig_robot);
    clf;
    hold on; grid on; axis square; axis equal;
    pcom = X(ii,1:3)';
    xlim([pcom(1)-0.5 pcom(1)+0.5]);
    ylim([pcom(2)-0.5 pcom(2)+0.5]);
    zlim([-0.2 0.6]);
    view([0.2,0.5,0.2]);

    fig_plot_robot(X(ii,:)', U(ii,:)', Ue(ii,:)', p);
    fig_plot_robot_d(Xd(ii,:)', zeros(size(Ud(ii,:)))', p);

    text(pcom(1), pcom(2), 0.4, ['t = ', num2str(t(ii), 2), ' s']);
    text(pcom(1), pcom(2), 0.5, ['vd = ', num2str(Xd(ii,4), 2), ' m/s']);
    text(pcom(1), pcom(2), 0.45, ['v = ', num2str(X(ii,4), 2), ' m/s']);

    hold off;

    if flag_movie
        writeVideo(vidfile, getframe(fig_robot));
    end

    drawnow;
end

font_size = 12;
script_folder = fileparts(mfilename('fullpath'));
save_folder = fullfile(script_folder, 'plots_mod_fin');

exportgraphics(fig_robot, fullfile(save_folder, name_rob), 'ContentType', 'vector');

if flag_movie
    close(vidfile);
end


%% === Plot Static Graphs After Simulation ===

fig_pos = figure('Name', 'Position', 'NumberTitle', 'off'); clf;

subplot(3,1,1);
plot(t, X(:,1), 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1.2); hold on;
plot(t, Xd(:,1), '--', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1.2);
ylabel('$x(t)\ \mathrm{[m]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

subplot(3,1,2);
plot(t, X(:,2), 'b', 'LineWidth', 1.2); hold on;
plot(t, Xd(:,2), 'b--', 'LineWidth', 1.2);
ylabel('$y(t)\ \mathrm{[m]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

subplot(3,1,3);
plot(t, X(:,3), 'g', 'LineWidth', 1.2); hold on;
plot(t, Xd(:,3), 'g--', 'LineWidth', 1.2);
ylabel('$z(t)\ \mathrm{[m]}$', 'Interpreter', 'latex', 'FontSize', font_size);
xlabel('$\mathrm{Time\ [s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

sgtitle('Actual and Desired Positions', 'FontSize', font_size + 2, 'Interpreter', 'latex');
exportgraphics(fig_pos, fullfile(save_folder, name_pos), 'ContentType', 'vector');

fig_vel = figure('Name', 'Velocity', 'NumberTitle', 'off'); clf;

subplot(3,1,1);
plot(t, X(:,4), 'r', 'LineWidth', 1.2); hold on;
plot(t, Xd(:,4), 'r--', 'LineWidth', 1.2);
ylabel('$v_x(t)\ \mathrm{[m/s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

subplot(3,1,2);
plot(t, X(:,5), 'b', 'LineWidth', 1.2); hold on;
plot(t, Xd(:,5), 'b--', 'LineWidth', 1.2);
ylabel('$v_y(t)\ \mathrm{[m/s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

subplot(3,1,3);
plot(t, X(:,6), 'g', 'LineWidth', 1.2); hold on;
plot(t, Xd(:,6), 'g--', 'LineWidth', 1.2);
ylabel('$v_z(t)\ \mathrm{[m/s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
xlabel('$\mathrm{Time\ [s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

sgtitle('Actual and Desired Velocities', 'FontSize', font_size + 2, 'Interpreter', 'latex');
exportgraphics(fig_vel, fullfile(save_folder, name_vel), 'ContentType', 'vector');

fig_ang_vel = figure('Name', 'Angular Velocity', 'NumberTitle', 'off'); clf;

subplot(3,1,1);
plot(t, X(:,16), 'r', 'LineWidth', 1.2); hold on;
plot(t, Xd(:,16), 'r--', 'LineWidth', 1.2);
ylabel('$\omega_x(t)\ \mathrm{[rad/s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

subplot(3,1,2);
plot(t, X(:,17), 'g', 'LineWidth', 1.2); hold on;
plot(t, Xd(:,17), 'g--', 'LineWidth', 1.2);
ylabel('$\omega_y(t)\ \mathrm{[rad/s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

subplot(3,1,3);
plot(t, X(:,18), 'b', 'LineWidth', 1.2); hold on;
plot(t, Xd(:,18), 'b--', 'LineWidth', 1.2);
ylabel('$\omega_z\ \mathrm{[rad/s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
xlabel('$\mathrm{Time\ [s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

sgtitle('Actual and Desired Angular Velocities', 'FontSize', font_size + 2, 'Interpreter', 'latex');
exportgraphics(fig_ang_vel, fullfile(save_folder, name_ang_vel), 'ContentType', 'vector');

fig_ctrl = figure('Name', 'Control Force Fz', 'NumberTitle', 'off'); clf;

subplot(4,1,1);
plot(t2, U2(:,3), 'r', 'LineWidth', 1.2); hold on;
plot(t, Ud(:,3), 'r--', 'LineWidth', 1.2);
ylabel('$F_{z1}\ \mathrm{[N]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

subplot(4,1,2);
plot(t2, U2(:,6), 'g', 'LineWidth', 1.2); hold on;
plot(t, Ud(:,6), 'g--', 'LineWidth', 1.2);
ylabel('$F_{z2}\ \mathrm{[N]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

subplot(4,1,3);
plot(t2, U2(:,9), 'b', 'LineWidth', 1.2); hold on;
plot(t, Ud(:,9), 'b--', 'LineWidth', 1.2);
ylabel('$F_{z3}\ \mathrm{[N]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

subplot(4,1,4);
plot(t2, U2(:,12), 'k', 'LineWidth', 1.2); hold on;
plot(t, Ud(:,12), 'k--', 'LineWidth', 1.2);
ylabel('$F_{z4}\ \mathrm{[N]}$', 'Interpreter', 'latex', 'FontSize', font_size);
xlabel('$\mathrm{Time\ [s]}$', 'Interpreter', 'latex', 'FontSize', font_size);
%legend({'Real','Desired'}, 'Interpreter', 'latex', 'FontSize', font_size, 'Location', 'best');
grid on; set(gca, 'FontSize', font_size);

sgtitle('Control Forces Fz', 'FontSize', font_size + 2, 'Interpreter', 'latex');
exportgraphics(fig_ctrl, fullfile(save_folder, name_cnt), 'ContentType', 'vector');
