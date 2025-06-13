clc
close all
clear all

if ~exist('plot', 'dir')
    mkdir('plot');
end

% Parameters
g = 9.81;          % gravity (m/s^2)
l = 3.0;           % leg length (m) %%MODIFY HERE%% 
alpha = pi/8;      % half inter-leg angle (rad) %%MODIFY HERE%% 
%gamma = 0.1746;      % slope angle (rad) %%MODIFY HERE%% 10 gradi
%gamma = 0.3491;      % slope angle (rad) %%MODIFY HERE%% 20 gradi
gamma = 0.08;
gamma_grad = gamma*180/pi;
% Initial conditions
thetadot0 = 0.95;%%MODIFY HERE%% 

result = 2*g*(1-cos(gamma-alpha))/(thetadot0)^2;

if (thetadot0 >= 0)
    theta0 = gamma-alpha;
else
    theta0 = gamma+alpha;
end

double_support = 0;

y0 = [theta0; thetadot0];

% Simulation settings
t0 = 0; %initial time
tf = 25; %final time
dt = 0.01; %max step time

% Time/state storage
T = [];
Y = [];

while t0 < tf
    options = odeset('Events', @(t, y) impact_event(t, y, alpha,gamma), 'MaxStep', dt);
    [t, y, te, ye, ie] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);
    
    T = [T; t];
    Y = [Y; y];

    if ~isempty(te)
        [y0,double_support] = impact_map(ye, alpha,g,l); % apply impact map
        t0 = te;
    else
        break;
    end
end

font_size = 12;  % scegli la dimensione del font
script_folder = fileparts(mfilename('fullpath'));
save_folder = fullfile(script_folder, 'plot');

% --- Figure 1 ---
figure(1); clf;

plot(T, Y(:,1), 'b', 'LineWidth', 1.2, 'DisplayName', '$\theta$ (rad)');
hold on;
plot(T, Y(:,2), 'r', 'LineWidth', 1.2, 'DisplayName', '$\dot{\theta}$ (rad/s)');

xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('State', 'Interpreter', 'latex', 'FontSize', font_size);

title('Rimless Wheel Dynamics', 'Interpreter', 'latex', 'FontSize', font_size + 2);

legend('Interpreter','latex', 'FontSize', font_size, 'Location', 'best');
grid on;
set(gca, 'FontSize', font_size);
exportgraphics(figure(1), fullfile(save_folder, "ex4_l_21.pdf"), 'ContentType', 'vector');

% --- Figure 2 ---
figure(2); clf;

plot(Y(:,1), Y(:,2), 'b', 'LineWidth', 1.2, 'DisplayName', '$\theta$ (rad)');
hold on;
plot(Y(1,1), Y(1,2), 'r*', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Initial point');

xlabel('$\theta$ (rad)', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$\dot{\theta}$ (rad/s)', 'Interpreter', 'latex', 'FontSize', font_size);

title('Rimless Wheel Limit Cycle', 'Interpreter', 'latex', 'FontSize', font_size + 2);

%legend('Interpreter','latex', 'FontSize', font_size, 'Location', 'southeast');
grid on;
set(gca, 'FontSize', font_size);
exportgraphics(figure(2), fullfile(save_folder, "ex4_l_22.pdf"), 'ContentType', 'vector');


function dydt = dynamics(~, y, g, l, ds)
    theta = y(1);
    thetadot = y(2);
    if (~ds)
        dtheta = thetadot;
        dthetadot = (g/l) * sin(theta);
    else
        dtheta = 0;
        dthetadot = 0;
    end
    dydt = [dtheta; dthetadot];
end

function [value, isterminal, direction] = impact_event(~, y, alpha,gamma)
    
    value = [y(1)-alpha-gamma; y(1)-gamma+alpha];% Trigger when theta = gamma+alpha
                                     %Trigger when theta = gamma-alpha
    isterminal = [1;1];         % Stop the integration
    direction = [1;-1];          % Detect only when increasing
end

function [yplus,ds] = impact_map(y_minus, alpha,g,l)%minus: before impact time; plus: after impact time
    if (y_minus(2)>=0)
        theta_plus = y_minus(1)-2*alpha;
    else
        theta_plus = y_minus(1)+2*alpha;
    end
    thetadot_plus = cos(2*alpha) * y_minus(2);
    if (thetadot_plus < 0.01*sqrt(g/l) && thetadot_plus >-0.01*sqrt(g/l)) 
        thetadot_plus = 0;
        ds = 1;
    else
        ds = 0;
    end
    yplus = [theta_plus; thetadot_plus];
end