
clear all;
clc;

%% Root Locus of the Uncompensated System

% Define the numerator and denominator of the transfer function
num = 1;   % K is kept as 1 for the root locus plot
den = conv([1 0], conv([1 2], [1 8]));  % s*(s + 2)*(s + 8)

% Create the transfer function
sys = tf(num, den);

% Plot the root locus
figure;
rlocus(sys);
title('Root Locus of the UnCompansated System');
xlabel('Real Axis (seconds^{-1})');
ylabel('Imaginary Axis (seconds^{-1})');
xlim([-10,5]);
ylim([-5,5]);

% Customization for marking points
hold on;


% Mark the given Dominant poles P1 and P2
P1 = -0.8 + (96/90)*1i;
P2 = -0.8 - (96/90)*1i;
plot(real(P1), imag(P1), 'x', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'LineWidth', 2);
plot(real(P2), imag(P2), 'x', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 2);

set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2);

% Add legend
legend('Root Locus of the Uncompensated system', ...
       'Dominant Pole P1 at -0.8 + (1+(1/15))i', ...
       'Dominant Pole P2 at -0.8 - (1+(1/15))i', 'Location', 'Best');
grid on;

%% Magnitude Calculation

G = tf(1,conv([1 0], conv([1 2], [1 8]))); % Uncompensated open loop

stepinfo(sys)
j = sqrt(-1);

zeta = 0.6;
wn = 4/3;

sd1 = -zeta*wn + j*wn*sqrt(1-zeta^2);
sd2 = -zeta*wn - j*wn*sqrt(1-zeta^2);

%Finding gain using magnitude criterion;
mag_0 = abs(polyval([1,0],sd1));
mag_2 = abs(polyval([1,2],sd1));
mag_8 = abs(polyval([1,8],sd1));

disp('Magnitude Criterion');
disp([mag_0, mag_2, mag_8]);
K = mag_0*mag_2*mag_8; % Gain of the Uncompensated System from Magnitude criterion
disp(K);
%% Transient Response

% Define the numerator and denominator of the transfer function
num = 15.5815;   % K is kept as 1 for the root locus plot
den = conv([1 0], conv([1 2], [1 8]));  % s*(s + 2)*(s + 8)

% Create the transfer function
sys_open_loop = tf(num, den);

% Create the closed-loop transfer function with unit negative feedback
sys_closed_loop = feedback(sys_open_loop, 1);

% Compute the step response and step information
[y, t] = step(sys_closed_loop);
info = stepinfo(sys_closed_loop);

% Extract settling time and overshoot
settling_time = info.SettlingTime;
overshoot = info.Overshoot; % Maximum overshoot in percentage

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
title('Step Response of the Closed-Loop Uncompensated System', 'FontSize', 14, 'FontWeight', 'bold');
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
         'FontSize', 12, 'Color', 'black', 'FontWeight', 'bold', 'VerticalAlignment', 'top');
end

% Add legend
legend('Before Settling Time', 'After Settling Time', 'Settling Time', 'Location', 'Best', 'FontSize', 12);

hold off;



%% Root Locus of the Compensated System

% Define the numerator and denominator of the transfer function
num = [15.5815 15.5815 * 0.015];  % 9.69*(s + 0.15)
den = conv([1 0], conv([1 2], conv([1 8], [1 0.0009626])));  % s*(s + 2)*(s + 8)*(s + 0.011355)

% Create the transfer function
sys = tf(num, den);

% Plot the root locus
figure;
rlocus(sys);
title('Root Locus of the Compensated System');
xlabel('Real Axis (seconds^{-1})');
ylabel('Imaginary Axis (seconds^{-1})');
xlim([-10,5]);
ylim([-2,2]);

% Customization for marking points (Update these values if you have specific dominant poles)
hold on;

% Example Dominant poles (Modify these if you have specific poles to mark)
P1 = -0.8 + 0.6i;
P2 = -0.8 - 0.6i;
plot(real(P1), imag(P1), 'x', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'LineWidth', 2);
plot(real(P2), imag(P2), 'x', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 2);

set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2);

% Add legend
legend('Root Locus of the System', ...
       'Dominant Pole P1', ...
       'Dominant Pole P2', 'Location', 'Best');
grid on;

%% Step Response of the Compensated System

% Define the numerator and denominator of the transfer function
num = [15.5815 15.5815 * 0.015];  % 9.69*(s + 0.15)
den = conv([1 0], conv([1 2], conv([1 8], [1 0.0009626])));  % s*(s + 2)*(s + 8)*(s + 0.011355)

% Create the transfer function for the open-loop system
sys_open_loop = tf(num, den);

% Create the closed-loop transfer function with unit negative feedback
sys_closed_loop = feedback(sys_open_loop, 1);

% Compute the step response and step information
[y, t] = step(sys_closed_loop);
info = stepinfo(sys_closed_loop);
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
title('Step Response of the Closed-Loop Compensated System', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Response', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% Adjust axis limits
set(gca, 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(t)]); % Adjust x-axis limits based on the time vector
ylim([0 1.2]); % Adjust y-axis limits based on the response

% Mark the maximum overshoot
[peak_value, peak_index] = max(y);
plot(t(peak_index), peak_value, 'go', 'MarkerSize', 8, 'LineWidth', 2); % Green circle for peak
text(t(peak_index), peak_value, sprintf(' Max Overshoot (%.1f%%)', overshoot), ...
     'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');


settling_index = find(t >= settling_time, 1, 'first');
if ~isempty(settling_index)
    plot(t(settling_index), y(settling_index), 'ro', 'MarkerSize', 8, 'LineWidth', 2); % Red circle for settling time
    text(t(settling_index), y(settling_index), sprintf(' Settling Time (%.2f s)', t(settling_index)), ...
         'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold', 'VerticalAlignment', 'top');
end

% Add legend
legend('Before Settling Time', 'After Settling Time', 'Max Overshoot', 'Settling Time', 'Location', 'Best', 'FontSize', 12);

hold off;

%% Question 2 Bode Plot of the Uncompensated system

% Define the numerator and denominator of the Uncompensated system transfer function
num = 5;  % got from the velocity constant
den = conv([1 0], conv([1 1], [0.5 1]));  % s*(s + 1)*(0.5s + 1)

% Create the transfer function for the open-loop system
uncompensated_sys = tf(num, den);

%Bode plot K1.G(s)
hFigure = figure;
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
hAxes = axes(hPanel, 'Color', 'none'); 
title("Bode Digram of Open loop transfer function with the gain K1");
margin(uncompensated_sys)
set(findall(gcf,'type','line'),'linewidth',2);
grid on
% Effect of gain
%% Finding Wgc for required phase margin

% Define the numerator and denominator of the Uncompensated system transfer function
num = 5;  % got from the velocity constant
den = conv([1 0], conv([1 1], [0.5 1]));  % s*(s + 1)*(0.5s + 1)

% Create the transfer function for the open-loop system
uncompensated_sys = tf(num, den);

[Magnitude, Phase, Wout] = bode(uncompensated_sys);

Magnitude = squeeze(Magnitude);
Phase = squeeze(Phase);

% This is the value of the new gain cross-over frequency by interpolation
Wgc = interp1(Phase, Wout, -135); 

% Gain at the new gain cross-over frequency
Gain = 20*log10(interp1(Wout, Magnitude, Wgc));

%% Lag Compensator Bode Plot

% Define the numerator and denominator of the Lag Compensator transfer function
num = 0.6649*[1 0.05621];
den = [1 0.007475];

compensator_sys = tf(num,den);

    hFigure = figure;
    hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
    hAxes = axes(hPanel, 'Color', 'none'); 
    bode(compensator_sys);
    title('Lag Compensator Bode Plot');
    set(findall(gcf,'type','line'),'linewidth',2, 'Color', 'r');
    grid on
%% Final Bode plot for Compensated System

% Define the numerator and denominator of the Compensated system transfer function
num = 0.6649*[1 0.05621];  % 0.6649*(s + 0.05621)
den = conv([1 0.007475], conv([1 0], conv([1 1], [0.5 1])));  % s*(s + 1)*(0.5s + 1)*(s + 0.007475)

% Create the transfer function for the open-loop system
compensated_sys = tf(num, den);
disp(compensated_sys)
%Bode plot K1.G(s)
hFigure = figure;
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
hAxes = axes(hPanel, 'Color', 'none'); 
[mag, phase,wout]=bode(compensated_sys);
title("Bode Digram of Open loop transfer function with the gain K1");
margin(compensated_sys)
set(findall(gcf,'type','line'),'linewidth',2);
grid on

%% Checking the velocity error constant

% Define the numerator and denominator of the Uncompensated system transfer function
num = 0.6649*[1 0.05621];  % 0.6649*(s + 0.05621)
den = conv([1 0.007475], conv([1 0], conv([1 1], [0.5 1])));  % s*(s + 1)*(0.5s + 1)*(s + 0.007475)

% Create the transfer function for the open-loop system
compensated_sys = tf(num, den);

% Create the closed-loop system using unity feedback
sys = feedback(compensated_sys, 1);

% Define the time vector
t = 0:0.01:50;

% Set up the figure and panel for visualization
hFigure = figure;
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  % Make a white panel with black line borders
hAxes = axes(hPanel, 'Color', 'none');  % Set the axes background color to none

% Simulate the ramp response (input = t)
Ramp_out = lsim(sys, t, t);

% Plot the response
plot(t, Ramp_out,'r-', 'LineWidth', 2);
hold on;
plot(t, t,'g--', 'LineWidth', 2);
grid on;
title("Unit Ramp Response of the Compensated System");

% Calculate and display the steady-state error for the ramp response
Ess_Ramp = t(end) - Ramp_out(end);
disp('Steady-state error for unit ramp input:');
disp(Ess_Ramp);

disp('Velocity error constant:');
disp(1/Ess_Ramp);

%% Plotting everyhitng on a single plot

% Define the numerator and denominator of the Uncompensated system transfer function
num = 5;  % got from the velocity constant
den = conv([1 0], conv([1 1], [0.5 1]));  % s*(s + 1)*(0.5s + 1)

% Create the transfer function for the open-loop system
uncompensated_sys = tf(num, den);

% Define the numerator and denominator of the Compensated system transfer function
num = 0.6649*[1 0.05621];  % 0.6649*(s + 0.05621)
den = conv([1 0.007475], conv([1 0], conv([1 1], [0.5 1])));  % s*(s + 1)*(0.5s + 1)*(s + 0.007475)

% Create the transfer function for the open-loop system
compensated_sys = tf(num, den);

% Define the numerator and denominator of the Lag Compensator transfer function
num = 0.6649*[1 0.05621];
den = [1 0.007475];

% Create the transfer function for the compensator system
compensator_sys = tf(num,den);

%Bode plot K1.G(s)
hFigure = figure;
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
hAxes = axes(hPanel, 'Color', 'none'); 
[mag, phase,wout]=bode(compensated_sys);
title("Bode Digram of all the systems");
bode(compensator_sys);
hold on;
bode(uncompensated_sys);
margin(compensated_sys);
set(findall(gcf,'type','line'),'linewidth',2);
grid on;
grid on;

% Adding legends and modifying line styles
legend('Lag Compensator', 'Uncompensated System', 'Compensated System');