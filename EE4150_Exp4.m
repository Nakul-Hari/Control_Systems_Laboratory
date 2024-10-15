clear all;
clc;
s = tf('s');

%% Finding out the value of Kp

K = 0.01;
J = 0.01;
b = 0.1;
R = 1;
L = 0.5;

T_sys = K/((J*s + b)*(L*s + R) + K^2);

K_p = 50:5:300;

for i = 1:length(K_p)
    T_pid = pid(K_p(i), 0, 0);
    T = feedback(T_pid * T_sys, 1);
    [y, t] = step(T, 1.2);
    
    % Store performance metrics
    Info = stepinfo(T, 'SettlingTimeThreshold', 0.05);
    overshoot = Info.Overshoot;
    Ess = abs(y(end) - 1) * 100;

    Kp_array(i, 1) = K_p(i);
    Kp_array(i, 2) = Info.RiseTime;
    Kp_array(i, 3) = Info.SettlingTime;
    Kp_array(i, 4) = overshoot;
    Kp_array(i, 5) = Ess;

    % Plot the step response
    if K_p(i) == 240  % Make Kp = 240 more prominent
        plot(t, y, 'LineWidth', 2, 'Color', 'r'); % Red color for prominence
    else
        plot(t, y, 'LineWidth', 0.5); % Normal plot for others
    end
    
    grid on;
    hold on;
end

% Enhancing figure aesthetics
title('Step Response of PID Controller', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Output Response', 'FontSize', 12);
xlim([0, 1.2]); % Adjust time limits as needed
ylim([0, 1.5]); % Adjust output limits as needed
set(gca, 'FontSize', 12); % Set axis font size
hold off; % Release the hold

disp(Kp_array);

% from the above obtained results ig we can choose an intermediate value
% to have a trade off between overshoot and steady-state error we'll choose
% the value of Kp to be 240.0000. 
% Hence overshoot will be 41.6841 % and Steady State Error will be 4.0710

%% Determininh the value of Ki

K = 0.01;
J = 0.01;
b = 0.1;
R = 1;
L = 0.5;

s = tf('s'); % Define 's' as a transfer function variable
T_sys = K/((J*s + b)*(L*s + R) + K^2);

K_i = 60:5:300; % Define K_i values
Ki_array = zeros(length(K_i), 5); % Initialize Ki_array to store results

for i = 1:length(K_i) % Loop through all K_i values

    T_pid = pid(240, K_i(i), 0); % Create PID controller
    T = feedback(T_pid*T_sys, 1); % Get the closed-loop transfer function
    [y, t] = step(T, 1.2); % Compute step response

    % Plot the step response
    if K_i(i) == 290  % Make Ki = 290 more prominent
        plot(t, y, 'LineWidth', 2, 'Color', 'g'); % Green color for prominence
    else
        plot(t, y, 'LineWidth', 0.5); % Normal plot for others
    end

    grid on;
    hold on;

    Info = stepinfo(T, 'SettlingTimeThreshold', 0.05); % Get step response info

    overshoot = Info.Overshoot; % Calculate overshoot
    Ess = abs(y(end) - 1) * 100; % Calculate steady-state error

    % Store metrics only if steady-state error is within requirements
    if Ess <= 1
        Ki_array(i, 1) = K_i(i); % Use K_i instead of K_p
        Ki_array(i, 2) = Info.RiseTime;
        Ki_array(i, 3) = Info.SettlingTime;
        Ki_array(i, 4) = overshoot;
        Ki_array(i, 5) = Ess;
    end
end

% Enhancing figure aesthetics
title('Step Response of PID Controller', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Output Response', 'FontSize', 12);
xlim([0, 1.2]); % Adjust time limits as needed
ylim([0, 1.5]); % Adjust output limits as needed
set(gca, 'FontSize', 12); % Set axis font size
hold off; % Release the hold

disp(Ki_array); % Display the results array

% We got a range of values from 200 to 295 as the value of Ki we'll choose
% one among them to be final we'll choose Ki = 290, Hence getting the
% Steady State Error as 0.5083%. Hence the Steady State Error Requirement
% is met

%% Obtaining Kd 

K = 0.01;
J = 0.01;
b = 0.1;
R = 1;
L = 0.5;

s = tf('s');  % Define the Laplace variable
T_sys = K / ((J * s + b) * (L * s + R) + K^2);

K_d = 0:5:100;  % Gain values for Kd
Kd_array = zeros(length(K_d), 5);  % Initialize array to store results

for i = 1:length(K_d)

    T_pid = pid(240, 290, K_d(i));  % Create PID controller with Kp = 240, Ki = 290, Kd = K_d(i)
    T = feedback(T_pid * T_sys, 1);  % Closed-loop transfer function
    [y, t] = step(T, 1.2);  % Step response
    
    % Plot the step response
    if K_d(i) == 50  % Make Kd = 50 more prominent
        plot(t, y, 'LineWidth', 2.5, 'Color', 'b');  % Blue color for prominence
    else
        plot(t, y, 'LineWidth', 0.5);  % Normal plot for others
    end

    grid on;
    hold on;

    Info = stepinfo(T, 'SettlingTimeThreshold', 0.05);
    
    overshoot = Info.Overshoot;
    Ess = abs(y(end) - 1) * 100;  % Calculate steady-state error as a percentage

    if overshoot <= 2
        Kd_array(i, 1) = 240;  % Store K_d value
        Kd_array(i, 2) = 290;  % Store rise time
        Kd_array(i, 3) = K_d(i);  % Store K_d value
        Kd_array(i, 4) = Info.RiseTime;  % Store rise time
        Kd_array(i, 5) = Info.SettlingTime;  % Store settling time
        Kd_array(i, 6) = overshoot;  % Store overshoot
        Kd_array(i, 7) = Ess;  % Store steady-state error
    end
end

% Enhancing figure aesthetics
title('Step Response of PID Controller', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Output Response', 'FontSize', 12);
xlim([0, 1.2]);  % Adjust time limits as needed
ylim([0, 1.5]);  % Adjust output limits as needed
set(gca, 'FontSize', 12);  % Set axis font size
hold off;  % Release the hold

disp(Kd_array);  % Display the results


% We got a range of values from 0 to 250 as the value of Ki we'll choose
% one among them to be final we'll choose Kd = 50, Hence getting the
% Maximum Overshoot as 0. Hence the Overshoot Requirement
% is met

%% Final Values for PID controller yaay

K = 0.01;
J = 0.01;
b = 0.1;
R = 1;
L = 0.5;

T_sys = K/((J*s + b)*(L*s + R) + K^2);
T_pid = pid(240, 290, 50);
T = feedback(T_pid*T_sys,1);

[y, t] = step(T);
info = stepinfo(T, 'SettlingTimeThreshold', 0.05);
disp(info);
% Extract settling time and overshoot
settling_time = info.SettlingTime;
overshoot = info.Overshoot; % Maximum overshoot in percentage

% Find the final value for settling time calculation
final_value = y(end);

% Plot the step response of the closed-loop system
figure;

% Plot the response before the settling time
before_settling = t <= settling_time;
plot(t(before_settling), y(before_settling), 'b', 'LineWidth', 2); % Blue line before settling

hold on;

% Plot the response after the settling time
after_settling = t > settling_time;
plot(t(after_settling), y(after_settling), 'r', 'LineWidth', 2); % Red line after settling

% Customize the plot
title('Step Response of the Compensated System', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Response', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% Adjust axis limits
set(gca, 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(t)]); % Adjust x-axis limits based on the time vector
ylim([0 1.2]); % Adjust y-axis limits based on the response


settling_index = find(t >= settling_time, 1, 'first');
if ~isempty(settling_index)
    plot(t(settling_index), y(settling_index), 'ro', 'MarkerSize', 8, 'LineWidth', 2); % Red circle for settling time
    text(t(settling_index), y(settling_index), sprintf(' Settling Time (%.2f s)', t(settling_index)), ...
         'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold', 'VerticalAlignment', 'top');
end

% Add legend
legend('Before Settling Time', 'After Settling Time', 'Max Overshoot', 'Settling Time', 'Location', 'Best', 'FontSize', 12);

hold off;

%% Pole Placement

K = 0.01;
J = 0.01;
b = 0.1;
R = 1;
L = 0.5;

A = [ -b/J, K/J;
      -K/L, -R/L];

B = [ 0;
      1/L];

C = [1 0];

D = 0;

sys = ss(A,B,C,D);

step(sys);
grid on;

%% Problem 1

t = [0,10]; % Time Step

iniCon = [0; % Initial conditions
          1];

[t, x] = ode45(@(t, x) odesolver(x), t, iniCon);

hFigure = figure();
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'none', ... % Removed the border
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
hAxes = axes(hPanel, 'Color', 'none');
plot(t, x(:,1), 'LineWidth', 1, 'DisplayName', '\omega(x_1) rad/s');
hold on;
plot(t, x(:,2), 'LineWidth', 1, 'DisplayName', 'current (x_2) A');
grid on;
xlabel("time (s)");
title(sprintf("ODE Simulation for the state space model of DC Motor without feedback control.\nInitial condition (x_1, x_2) = (%d, %d)", iniCon(1), iniCon(2)));
legend();


%% Controllability and Observability

Controllability = ctrb(A,B);
Rank_Controllability = rank(Controllability);

disp(Controllability);
disp(Rank_Controllability);

Observibility = obsv(A,C);
Rank_Observibility = rank(Observibility);

disp(Observibility);
disp(Rank_Observibility);

%% EigenValues

poles = eig(A);
disp(poles);

% Eigen values corresponds to the poles of the control system. Since both
% of them are in the LHP they'll be stable.


%% Designing State Feedback Controller 

iniCon1=[1,1];

K_1a = [0,1];
K_1b = [0,10];
K_1c = [0,20];

[t, y3_a] = ode45(@(t, x) odesolver2(x, K_1a(1), K_1a(2), 0), t, iniCon1);
[t, y3_b] = ode45(@(t, x) odesolver2(x, K_1b(1), K_1b(2), 0), t, iniCon1);
[t, y3_c] = ode45(@(t, x) odesolver2(x, K_1c(1), K_1c(2), 0), t, iniCon1);

hFigure = figure();
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'none', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
hAxes = axes(hPanel, 'Color', 'none');
plot(t,y3_a(:,1),'Color','r','LineStyle','-','Displayname',sprintf("x1, K1 = %d, K2 = %d",K_1a(1),K_1a(2)));
hold on;
plot(t,y3_b(:,1),'Color','b','LineStyle','-','Displayname',sprintf("x1, K1 = %d, K2 = %d",K_1b(1),K_1b(2)));
hold on;
plot(t,y3_c(:,1),'Color','g','LineStyle','-','Displayname',sprintf("x1, K1 = %d, K2 = %d",K_1c(1),K_1c(2)));
hold on;

plot(t,y3_a(:,2),'Color','r','LineStyle','--','Displayname',sprintf("x2, K1 = %d, K2 = %d",K_1a(1),K_1a(2)));
hold on;
plot(t,y3_b(:,2),'Color','b','LineStyle','--','Displayname',sprintf("x2, K1 = %d, K2 = %d",K_1b(1),K_1b(2)));
hold on;
plot(t,y3_c(:,2),'Color','g','LineStyle','--','DisplayName',sprintf("x2, K1 = %d, K2 = %d",K_1c(1),K_1c(2)));
hold on;

grid on;
set(findall(gcf,'type','line'),'linewidth',2)
legend();
xlim([0, 2])
title("Plots of x_1 and x_2 for Differnt values of K_2 while K_1 = 0")
hold off;

%% Designing State Feedback Controller 

iniCon1=[1,1];

K_1a = [1,1];

[t, y3_a] = ode45(@(t, x) odesolver2(x, K_1a(1), K_1a(2), 0), t, iniCon1);

hFigure = figure();
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'none', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
hAxes = axes(hPanel, 'Color', 'none');
plot(t,y3_a(:,1),'Color','g','LineStyle','-','Displayname',sprintf("x1, K1 = %d, K2 = %d",K_1a(1),K_1a(2)));
hold on;
plot(t,y3_a(:,2),'Color','r','LineStyle','--','Displayname',sprintf("x2, K1 = %d, K2 = %d",K_1a(1),K_1a(2)));
hold on;

grid on;
set(findall(gcf,'type','line'),'linewidth',2)
legend();
xlim([0, 2])
title("Plots of x_1 and x_2 for K_1 = 1 and K_2 = 1")
hold off;
%% Pole Placement

Controllability = ctrb(A,B);
Rank_Controllability = rank(Controllability);

disp(Controllability);
disp(Rank_Controllability);

% As the rank of the Controllability matrix is 2 which is same as the
% dimension of the controllablity matrix the system is controllable

j = sqrt(-1);
Ts = 2;
Mp = 0.05;
zeta = sqrt(log(Mp)^2/(pi^2 + log(Mp)^2));
w_n = 2/(zeta);

sd1 = -(zeta*w_n) + j*w_n*sqrt(1-zeta^2);
sd2 = -(zeta*w_n) - j*w_n*sqrt(1-zeta^2);

disp(sd1); % sd1 = -2 + 2.0974*j;
disp(sd2); % sd2 = -2 - 2.0974*j;

%sd1 and sd2 are our desired pole positions mu1 and mu2
%%
syms s;
SI = s.*[1, 0;
        0, 1];

M = SI - A;

det_M = det(M);
coeff = sym2poly(det_M);

disp(coeff);
%%
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
K = [alpha_2 - a2 alpha_1 - a1]*T_inv;
disp(K);
A_CL = A - B*K;
%% DC gain

s = tf("s");
I = [1,0;0,1];
sys = C*inv(s.*I -A_CL)*B;
dc_gain = dcgain(sys);
Kr = 1/dc_gain;


iniCon = [0,0];
[t, y2] = ode45(@(t, x) odesolver2(x, K(1), K(2), Kr), t, iniCon);
ts = t(y2(:,1)<=1.02 & y2(:,1)>=0.98);


% Define color palette for aesthetic plots
color1 = "#D95319"; % Orange
color2 = "#0072BD"; % Blue

% Figure 1 - Plot x1 with and without feedback control
hFigure = figure('Color', 'none'); % Set background color to white
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'none');  
hAxes = axes(hPanel, 'Color', 'none', 'Box', 'on'); % Add box around plot

% Plot with improved aesthetic settings
plot(t, y2(:,1), 'Color', color1, 'LineStyle', '-', 'Displayname', 'x_1 (with feedback control)', 'LineWidth', 2);
hold on;
plot(t, x(:,1), 'Color', color2, 'LineStyle', '--', 'Displayname', 'x_1 (without feedback control)', 'LineWidth', 2);

% % Add horizontal lines with transparency for better visibility
% yline(0.98, '--', 'Color', [0.5 0.5 0.5 0.7], 'LineWidth', 1);
% yline(1.02, '--', 'Color', [0.5 0.5 0.5 0.7], 'LineWidth', 1);

% Grid settings for better readability
grid on;
set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.3); % Light dotted grid
set(gca, 'FontSize', 12, 'FontName', 'Helvetica'); % Font settings

% Customize the legend
legend('Location', 'best', 'FontSize', 12, 'Box', 'on'); % Place legend outside

% Labeling and title with better spacing
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Amplitude', 'FontSize', 14, 'FontWeight', 'bold');
title(sprintf("Output x_1 of system with and without feedback control for unit step input.\nInitial condition (x_1,x_2) = (%d,%d)", iniCon(1), iniCon(2)), ...
      'FontSize', 14, 'FontWeight', 'bold');

% Second Figure - Plot x2 with and without feedback control
hFigure2 = figure('Color', 'w'); % Set background color to white
hPanel2 = uipanel(hFigure2, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'none');  
hAxes2 = axes(hPanel2, 'Color', 'none', 'Box', 'on'); % Add box around plot

% Plot with improved aesthetic settings
plot(t, y2(:,2), 'Color', color1, 'LineStyle', '-', 'Displayname', 'x_2 (with feedback control)', 'LineWidth', 2);
hold on;
plot(t, x(:,2), 'Color', color2, 'LineStyle', '--', 'Displayname', 'x_2 (without feedback control)', 'LineWidth', 2);

% Grid settings for better readability
grid on;
set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.3); % Light dotted grid
set(gca, 'FontSize', 12, 'FontName', 'Helvetica'); % Font settings

% Customize the legend
legend('Location', 'best', 'FontSize', 12, 'Box', 'on'); % Place legend outside

% Labeling and title with better spacing
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Amplitude', 'FontSize', 14, 'FontWeight', 'bold');
title(sprintf("Output x_2 plots of system with and without feedback control for unit step input.\nInitial condition (x_1,x_2) = (%d,%d)", iniCon(1), iniCon(2)), ...
      'FontSize', 14, 'FontWeight', 'bold');

set(findall(gcf,'type','line'),'LineWidth',1.5); % Increase line width



%% 
ss_error2 = abs(1 - y2(end,1))*100; %steady state error.
Mp_final = (max(y2(:,1))-1)*100; %Maximum peak overshoot
%% Functions used in the code

% Problem 1
function dxdt = odesolver(x)

K = 0.01;
J = 0.01;
b = 0.1;
R = 1;
L = 0.5;

v = 0;

dxdt(1) = (-b/J)*x(1) + (K/J)*x(2);
dxdt(2) = (-K/L)*x(1) + (-R/L)*x(2) + (1/L)*v;

dxdt = [dxdt(1); dxdt(2)];
y = x(1);

end

% Problem 4
function dxdt = odesolver2(x,K1,K2,Kr)

K = 0.01;
J = 0.01;
b = 0.1;
R = 1;
L = 0.5;

K_ = [K1,K2];

r = 1;
  
v = -K_*x + Kr*r;
dxdt(1) = (-b/J)*x(1) + (K/J)*x(2);
dxdt(2) = (-K/L)*x(1) + (-R/L)*x(2) + (1/L)*v;
dxdt = [dxdt(1); dxdt(2)];

end
