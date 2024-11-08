%% Inverted pendulum

clear all;
clc;

%% State modeling of the system

m = 0.3; % Mass 
L = 1; % Length of the pendulum
g = 9.81; % Gravity Acceleration

% Equilibrium points are x1 = 0 and x1 = pi

A1 = [ 0, 1;  % x1 = 0
      g/L, 0];

A2 = [ 0, 1; % x1 = pi
      -g/L, 0];

disp('The eigen values of A1 are');
disp(eig(A1));
disp('The eigen values of A2 are');
disp(eig(A2));

% The eigen values of A1 are 3.1321 and -3.1321
% The eigen values of A2 are 0.0000 + 3.1321i and 0.0000 - 3.1321i

% So we can see that the system is unstable for x1 = 0 as it has a pole on
% the RHS.

%% State model of the System

m = 0.3; % Mass 
L = 1; % Length of the pendulum
g = 9.81; % Gravity Acceleration

A = [ 0, 1;  
      g/L, 0];

B = [0;
     1/m*L^2];

C = [1, 0];

D = 0;

%% Controllability 

Controllability = ctrb(A,B);
Rank_Controllability = rank(Controllability);
disp('Controllability')
disp(Controllability);
disp(Rank_Controllability);

Observibility = obsv(A,C);
Rank_Observibility = rank(Observibility);
disp('Observibility')
disp(Observibility);
disp(Rank_Observibility);

% We can see that the system is both controllable and observable
% As the rank of the Controllability and Observability  matrix is 2
% Which is same as the dimension of the Controllablity and Observability Matrix
% The system is both Controllable and Observable


%% State feedback controller

sd1 = -5; % Desired pole positions
sd2 = -9;

syms s;
SI = s.*[1, 0;
        0, 1];

M = SI - A;

det_M = det(M);

coeff = sym2poly(det_M);

disp('Coefficents of the matrix');
disp(coeff);

a1 = coeff(2);
a2 = coeff(3);

E = Controllability;

W = [a1, 1;    % W matrix
      1, 0];

T = E*W; % Transformation matrix T

T_inv = inv(T);

p2 = (s - sd1)*(s - sd2);
coeff3 = sym2poly(p2);

alpha_1 = coeff3(2);
alpha_2 = coeff3(3);

disp('K values of the matrix');
K = [alpha_2 - a2 alpha_1 - a1]*T_inv;
disp(K);

A_CL = A - B*K;

s = tf("s");

I = [1,0;
    0,1];

sys = C*inv(s.*I -A_CL)*B;

dc_gain = dcgain(sys);
Kr = 1/dc_gain;

disp('DC gain of the system is');
disp(Kr);
%%

iniCon = [pi/8,0];
t = [0,5]; % Time Step

[t, x] = ode45(@(t, x) odesolver2(x, K(1), K(2), 0), t, iniCon);

hFigure = figure();
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'none', ...
                 'BorderWidth', 1, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
hAxes = axes(hPanel, 'Color', 'none');
plot(t,x(:,1),'Color','g','LineStyle','-','Displayname',sprintf("x1"));
hold on;
plot(t,x(:,2),'Color','r','LineStyle','--','Displayname',sprintf("x2"));

grid on;
set(findall(gcf,'type','line'),'linewidth',2)
legend();
title("Plots of x_1 and x_2 from pole placement method")
hold off;

%% Tracking Error Specifications

% Define parameters
t = 0:100;
iniCon = [0; 0];
w_n = 5 * pi / 100;         % frequency
a = 0.3;                    % amplitude

% Manually tuning the gain matrix K = [K1, K2]
K1 = [10, 50, 100];
K2 = [10, 50, 100];

% Finding output using ODE solver
[t, y1] = ode45(@(t, x) odesolver3(t, x, K1(1), K2(1), w_n, a), t, iniCon);
[t, y2] = ode45(@(t, x) odesolver3(t, x, K1(2), K2(2), w_n, a), t, iniCon);
[t, y3] = ode45(@(t, x) odesolver3(t, x, K1(3), K2(3), w_n, a), t, iniCon);

% Figure 1: Plotting x1 for different K values with reference trajectory
figure();
hPanel1 = uipanel(gcf, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                  'BorderType', 'none', 'BackgroundColor', 'w');
hAxes1 = axes(hPanel1, 'Color', 'none');
plot(t, y1(:,1), 'LineStyle', '-', 'Color', 'g', 'LineWidth', 2, 'DisplayName', 'K = 10');
hold on;
plot(t, y2(:,1), 'LineStyle', '-', 'Color', 'r', 'LineWidth', 2, 'DisplayName', 'K = 50');
plot(t, y3(:,1), 'LineStyle', '-', 'Color', 'b', 'LineWidth', 2, 'DisplayName', 'K = 100');
plot(t, 0.3 * sin(0.05 * pi * t), 'LineStyle', ':', 'Color', 'k', 'LineWidth', 2, 'DisplayName', 'desired');
grid on;
legend('show');
title("Output x_1 for increasing values of K_1 and K_2");
hold off;

% Tracking error calculation
tracking_error = 100 * abs(0.3 - max(y3(:,1)));

% Figure 2: Plotting x2 for different K values with reference trajectory
figure();
hPanel2 = uipanel(gcf, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                  'BorderType', 'none', 'BackgroundColor', 'w');
hAxes2 = axes(hPanel2, 'Color', 'none');
plot(t, y1(:,2), 'LineStyle', '-', 'Color', 'g', 'LineWidth', 2, 'DisplayName', 'K = 10');
hold on;
plot(t, y2(:,2), 'LineStyle', '-', 'Color', 'r', 'LineWidth', 2, 'DisplayName', 'K = 50');
plot(t, y3(:,2), 'LineStyle', '-', 'Color', 'b', 'LineWidth', 2, 'DisplayName', 'K = 100');
plot(t, 0.3 * 0.05 * pi * cos(0.05 * pi * t), 'LineStyle', ':', 'Color', 'k', 'LineWidth', 2, 'DisplayName', 'desired');
grid on;
legend('show');
title("Output x_2 for increasing values of K_1 and K_2");
hold off;

% Figure 3: Final system response with chosen K values
figure();
hPanel3 = uipanel(gcf, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                  'BorderType', 'none', 'BackgroundColor', 'w');
hAxes3 = axes(hPanel3, 'Color', 'none');
plot(t, y3(:,1), 'LineStyle', '-', 'Color', 'g', 'LineWidth', 2, 'DisplayName', 'x_1');
hold on;
plot(t, 0.3 * sin(0.05 * pi * t), 'LineStyle', '--', 'Color', 'b', 'LineWidth', 2, 'DisplayName', 'desired x_1');
plot(t, y3(:,2), 'LineStyle', '-', 'Color', 'r', 'LineWidth', 2, 'DisplayName', 'x_2');
plot(t, 0.3 * 0.05 * pi * cos(0.05 * pi * t), 'LineStyle', '--', 'Color', 'k', 'LineWidth', 1, 'DisplayName', 'desired x_2');
grid on;
legend('show');
title("The final system Response for chosen value of K_1 = K_2 = 100");
hold off;

% Set overall line width for all figures
set(findall(gcf, 'type', 'line'), 'linewidth', 2);

%% Problem 5

m = 0.3;
g = 9.81;
L = 1;

% Manually tuning the Q and R matrices
Q = [1, 0; 0, 1];
R = [10, 1, 0.1, 0.01, 0.001];
A = [0, 1; g/L, 0];
B = [0; 1/(m*L^2)];

% Define a set of colors for plotting
colors = lines(length(R));  % Using MATLAB's predefined colormap "lines" which generates a set of distinguishable colors

for i = 1:length(R)
    K_m(i, :) = lqr(A, B, Q, R(i));
    K1 = K_m(i, 1);
    K2 = K_m(i, 2);
    t = 0:0.1:8;
    iniCon = [pi/8; 0];
    [t, y_lqr] = ode45(@(t, x) odesolver4(t, x, K1, K2), t, iniCon);
    
    % Plot for x_1 with a unique color for each R
    figure(1)
    plot(t, y_lqr(:, 1), 'LineStyle', '-', 'LineWidth', 2, 'Color', colors(i, :), 'DisplayName', sprintf("x_1 R = %.3f", R(i)));
    hold on;
    grid on;
    set(gca, 'LineWidth', 2);
    legend();
    title("Effect of R on x_1");
    
    % Plot for x_2 with a unique color for each R
    figure(2);
    plot(t, y_lqr(:, 2), 'LineStyle', '--', 'LineWidth', 2, 'Color', colors(i, :), 'DisplayName', sprintf("x_2 R = %.3f", R(i)));
    hold on;
    grid on;
    set(gca, 'LineWidth', 2);
    legend();
    title("Effect of R on x_2");
end


%% Changing Q while keeping R fixed as 0.1
R = 0.1;
a = [1, 5, 10];

% Define a set of colors for plotting
colors = lines(length(a));  % Using MATLAB's predefined colormap "lines" which generates a set of distinguishable colors

for i = 1:length(a)
    Q = [a(i), 0; 0, a(i)];
    K_m(i, :) = lqr(A, B, Q, R);
    K1 = K_m(i, 1);
    K2 = K_m(i, 2);
    t = 0:0.1:8;
    iniCon = [pi/8; 0];
    [t, y_lqr] = ode45(@(t, x) odesolver4(t, x, K1, K2), t, iniCon);
    
    % Plot for x_1 with a unique color for each a(i)
    figure(1)
    plot(t, y_lqr(:, 1), 'LineStyle', '-', 'LineWidth', 2, 'Color', colors(i, :), 'DisplayName', sprintf("x_1 q_{ii} = %d", a(i)));
    hold on;
    grid on;
    set(gca, 'LineWidth', 2);
    legend();
    title("Effect of Q on x_1");
    
    % Plot for x_2 with a unique color for each a(i)
    figure(2);
    plot(t, y_lqr(:, 2), 'LineStyle', '--', 'LineWidth', 2, 'Color', colors(i, :), 'DisplayName', sprintf("x_2 q_{ii} = %d", a(i)));
    hold on;
    grid on;
    set(gca, 'LineWidth', 2);
    legend();
    title("Effect of Q on x_2");
end

%% LQR METHOD
m = 0.3;
g = 9.81;
L = 1;

% Manually tuning the Q and R matrices
Q = [10, 0; 0, 10];
R = 0.1;

A = [0, 1; g/L, 0];
B = [0; 1/(m*L^2)];

% Solving for the Gain matrix using MATLAB's LQR function
K_m = lqr(A, B, Q, R);
K1 = K_m(1);
K2 = K_m(2);

% Time vector and initial conditions
t = 0:0.1:20;
iniCon = [pi/8; 0];

% Solve the ODE system using ode45
[t, y_lqr] = ode45(@(t, x) odesolver4(t, x, K1, K2), t, iniCon);

% Plotting the results
figure;
plot(t, y_lqr(:, 1), 'LineStyle', '-', 'LineWidth', 2, 'Color', 'b', 'DisplayName', 'x_1', 'MarkerSize', 5);
hold on;
plot(t, y_lqr(:, 2), 'LineStyle', '--', 'LineWidth', 2, 'Color', 'r', 'DisplayName', 'x_2', 'MarkerSize', 5);
hold off;

% Enhance the plot for aesthetics
grid on;
set(gca, 'LineWidth', 1.5, 'FontSize', 12, 'Box', 'on', 'FontName', 'Arial');
legend('show', 'Location', 'best', 'FontSize', 12);
title('Output of the Linearized System with State Feedback Controller using LQR Method', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('States', 'FontSize', 12);

% Closed loop system
A_cl3 = A - B*K_m;

% Calculate closed loop poles
syms l;
SI = l * [1, 0; 0, 1];
M = SI - A_cl3;
det_M = l^2 + 34.6443*l + 34.7469;
poles = roots([1, 34.6443, 34.7469]);

% Display closed loop poles
disp('Closed loop poles:');
disp(poles);


%% Problem 5.2: Initial conditions and requirements
t = 0:100;
iniCon = [0; 0];
w_n = 5*pi/100;
a = 0.3;

Q_t = [20, 0; 0, 20];
R_t = 0.001;
K_t = lqr(A, B, Q_t, R_t);
K1 = K_t(1);
K2 = K_t(2);

[t, y_lqr4] = ode45(@(t, x) odesolver3(t, x, K1, K2, w_n, a), t, iniCon);

theta_desired = 0.3*sin(0.05*pi*t);
theta_dot_desired = 0.3*0.05*pi*cos(0.05*pi*t);
x_desired = [theta_desired, theta_dot_desired];

u = -K_t * (y_lqr4' - x_desired');

figure(4)
plot(t, y_lqr4(:, 1), 'LineStyle', '-', 'LineWidth', 2, 'Color', 'g');
hold on;
plot(t, theta_desired, 'LineStyle', ':', 'LineWidth', 2, 'Color', 'k');
plot(t, y_lqr4(:, 2), 'LineStyle', '--', 'LineWidth', 2, 'Color', 'r');
plot(t, theta_dot_desired, 'LineStyle', ':', 'LineWidth', 2, 'Color', 'k');
legend({'$\theta$', "desired track of $\theta$", '$\dot{\theta}$', "desired track of $\dot{\theta}$"}, 'Interpreter', 'latex');
title("Tracking Trajectory using LQR");

figure(3);
plot(t, u, 'LineWidth', 2);
title("Control Input");

% Define a set of colors for the lines
colors = ['g', 'b', 'r', 'c', 'm']; % You can add more colors if needed

R = 0.1;
b = [1, 5, 10, 15, 20];
for i = 1:length(b)
    Q = [b(i), 0; 0, b(i)];
    K_t(i, :) = lqr(A, B, Q, R);
    K1 = K_t(i, 1);
    K2 = K_t(i, 2);
    
    [t, y_lqr4] = ode45(@(t, x) odesolver3(t, x, K1, K2, w_n, a), t, iniCon);
    
    % Plot for x1
    figure(1)
    plot(t, y_lqr4(:, 1), 'LineStyle', '-', 'LineWidth', 2, 'Color', colors(i), 'DisplayName', sprintf("x_1 q_{ii} = %d", b(i)));
    hold on;
    grid on;
    set(gca, 'LineWidth', 2);
    legend();
    title("Effect of Q on x_1");
    
    % Plot for x2
    figure(2)
    plot(t, y_lqr4(:, 2), 'LineStyle', '--', 'LineWidth', 2, 'Color', colors(i), 'DisplayName', sprintf("x_2 q_{ii} = %d", b(i)));
    hold on;
    grid on;
    set(gca, 'LineWidth', 2);
    legend();
    title("Effect of Q on x_2");
end

% Plot the desired track for x1
figure(1)
plot(t, 0.3*sin(w_n*t), 'LineStyle', ':', 'LineWidth', 2, 'Color', 'k', 'DisplayName', "desired track");
legend();

% Plot the desired track for x2
figure(2)
plot(t, 0.3*w_n*cos(w_n*t), 'LineStyle', ':', 'LineWidth', 2, 'Color', 'k', 'DisplayName', "desired track");
legend();


%% Changing R while keeping Q fixed as [20,0;0,20]
% Define a set of colors for the lines
colors = ['g', 'b', 'r']; % Adjust according to the number of R values

R = [0.1, 0.01, 0.001];
for i = 1:length(R)
    t = 0:0.1:100;
    iniCon = [0; 0];
    K_t(i, :) = lqr(A, B, Q, R(i));
    K1 = K_t(i, 1);
    K2 = K_t(i, 2);
    
    [t, y_lqr4] = ode45(@(t, x) odesolver3(t, x, K1, K2, w_n, a), t, iniCon);
    
    % Plot for x1
    figure(1)
    plot(t, y_lqr4(:, 1), 'LineStyle', '-', 'LineWidth', 2, 'Color', colors(i), 'DisplayName', sprintf("x_1 R = %.3f", R(i)));
    hold on;
    grid on;
    set(gca, 'LineWidth', 2);
    legend();
    title("Effect of R on x_1");
    
    % Plot for x2
    figure(2)
    plot(t, y_lqr4(:, 2), 'LineStyle', '--', 'LineWidth', 2, 'Color', colors(i), 'DisplayName', sprintf("x_2 R = %.3f", R(i)));
    hold on;
    grid on;
    set(gca, 'LineWidth', 2);
    legend();
    title("Effect of R on x_2");
end

% Plot the desired track for x1
figure(1)
plot(t, 0.3*sin(w_n*t), 'LineStyle', ':', 'LineWidth', 2, 'Color', 'k', 'DisplayName', "desired track");
legend();

% Plot the desired track for x2
figure(2)
plot(t, 0.3*w_n*cos(w_n*t), 'LineStyle', ':', 'LineWidth', 2, 'Color', 'k', 'DisplayName', "desired track");
legend();


%% Functions

function dxdt = odesolver2(x, K1, K2, r)
    m = 0.3; % Mass 
    L = 1; % Length of the pendulum
    g = 9.81; % Gravity Acceleration

    K = [K1, K2];
    v = -K * (x + r); % State Controller
    
    dxdt = zeros(2, 1); % Initialize dxdt as a column vector
    dxdt(1) = x(2);
    dxdt(2) = (g / L) * x(1) + (1 / (m * L^2)) * v;
end

function dxdt = odesolver3(t,x,K1,K2,w_n,a)
    m = 0.3; % Mass 
    L = 1; % Length of the pendulum
    g = 9.81; % Gravity Acceleration

    K = [K1, K2];
    x_desired = [0.3*sin(0.05*pi*t); 0.3*0.05*pi*cos(0.05*pi*t)];
    u=-K*(x-x_desired);

    dxdt(1) = x(2);
    dxdt(2) = (g/L)*(x(1)) + (1/m*L^2)*u;
    dxdt = [dxdt(1); dxdt(2)];
end

function dxdt = odesolver4(t,x,K1,K2)
    m = 0.3;
    g=9.81;
    L = 1;
    K = [K1,K2];
    u=-K*(x);
    
    dxdt(1) = x(2);
    dxdt(2) = (g/L)*(x(1)) + (1/m*L^2)*u;
    dxdt = [dxdt(1); dxdt(2)];
end

