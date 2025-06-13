clc; clear all; close all;

% --- CREATE FOLDER IF NOT EXIST ---
if ~exist('plot', 'dir')
    mkdir('plot');
end

% --- PARAMETERS ---
g = 9.81;
l = 1.0;
alpha = pi/8;
gamma = 0.08;
dt = 0.01;
tf = 15;

% --- RANGE OF INITIAL VELOCITIES TO TEST ---
thetadot0_range = linspace(-5, 0, 500);

% Preallocate result array: equilibrium theta or NaN for limit cycle
theta_equilibrium = nan(size(thetadot0_range));

for i = 1:length(thetadot0_range)
    thetadot0 = thetadot0_range(i);

    if thetadot0 >= 0
        theta0 = gamma - alpha;
    else
        theta0 = gamma + alpha;
    end

    y0 = [theta0; thetadot0];
    t0 = 0;
    double_support = 0;

    while t0 < tf
        options = odeset('Events', @(t, y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
        [t, y, te, ye, ~] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);

        if isempty(te)
            break; % no impact -> stop
        end

        [y0, double_support] = impact_map(ye, alpha, g, l);
        t0 = te;

        if double_support
            break; % stopped -> equilibrium
        end
    end

    if double_support
        theta_equilibrium(i) = y0(1); % equilibrium theta value
    else
        theta_equilibrium(i) = 0; % limit cycle, mark as NaN
    end
end

% --- PLOT RESULTS ---


font_size = 12;  % scegli la dimensione del font
script_folder = fileparts(mfilename('fullpath'));
save_folder = fullfile(script_folder, 'plot');

figure(1); clf;

plot(thetadot0_range, theta_equilibrium, 'b-', 'LineWidth', 1.2);
hold on;

% Rimuovi i tick dell'asse y
set(gca, 'YTick', []);

% Imposta etichette asse x e y (y vuota)
xlabel('$\dot{\theta}_0$ (rad/s)', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('\textbf{}', 'Interpreter', 'latex', 'FontSize', font_size);  % ancora vuota ma latex-ready

% Imposta i tick y sulle posizioni desiderate
yticks([-0.31, 0, 0.47]);
% Disabilita le etichette y
set(gca, 'YTickLabel', []);

% Aggiungi etichette manuali con text() su due righe
text(min(thetadot0_range) - 0.05, -0.31, {'(-0.31,0)'}, ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', ...
    'FontSize', font_size);

text(min(thetadot0_range) - 0.05, 0, {'Limit', 'Cycle'}, ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', ...
    'FontSize', font_size);

text(min(thetadot0_range) - 0.05, 0.47, {'(0.47,0)'}, ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', ...
    'FontSize', font_size);
% Abilita LaTeX per le etichette e imposta font
set(gca, 'TickLabelInterpreter', 'none', 'FontSize', font_size);
% Rifinisci il plot
grid on;
xlim([min(thetadot0_range) max(thetadot0_range)]);
title('\textbf{Equilibria}', 'Interpreter', 'latex', 'FontSize', font_size + 2);

set(gca, 'FontSize', font_size);

% Salva la figura in formato vettoriale PDF
exportgraphics(gcf, fullfile(save_folder, 'equilibrium_vs_initial_velocity_curve_neg.pdf'), 'ContentType', 'vector');


%% --- FUNCTIONS ---
function dydt = dynamics(~, y, g, l, ds)
    theta = y(1);
    thetadot = y(2);
    if ~ds
        dtheta = thetadot;
        dthetadot = (g/l)*sin(theta);
    else
        dtheta = 0;
        dthetadot = 0;
    end
    dydt = [dtheta; dthetadot];
end

function [value, isterminal, direction] = impact_event(~, y, alpha, gamma)
    value = [y(1) - alpha - gamma; y(1) - gamma + alpha];
    isterminal = [1; 1];
    direction = [1; -1];
end

function [yplus, ds] = impact_map(y_minus, alpha, g, l)
    if y_minus(2) >= 0
        theta_plus = y_minus(1) - 2*alpha;
    else
        theta_plus = y_minus(1) + 2*alpha;
    end
    thetadot_plus = cos(2*alpha) * y_minus(2);

    if abs(thetadot_plus) < 0.01 * sqrt(g/l)
        thetadot_plus = 0;
        ds = 1; % equilibrium reached
    else
        ds = 0; % still moving (limit cycle)
    end

    yplus = [theta_plus; thetadot_plus];
end