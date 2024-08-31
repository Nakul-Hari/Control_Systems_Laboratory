clear all;
clc;

%% Root Locus of the Uncompensated System

% Define the numerator and denominator of the transfer function
num = [1];   % K is kept as 1 for the root locus plot
den = [1 10 24 0];  % Coefficients of s^3, s^2, s, and s^0 in the denominator

% Create the transfer function
sys = tf(num, den);

% Plot the root locus
figure;
rlocus(sys);
sgrid(.3579,5.589);
title('Root Locus of the UnCompansated System');
xlabel('Real Axis (seconds^{-1})');
ylabel('Imaginary Axis (seconds^{-1})');

% Customization for marking points
hold on;


% Mark the given Dominant poles P1 and P2
P1 = -2 + 5.216i;
P2 = -2 - 5.216i;
plot(real(P1), imag(P1), 'x', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'LineWidth', 2);
plot(real(P2), imag(P2), 'x', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 2);

set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2);

% Add legend
legend('Root Locus of the Uncompensated system', ...
       'Dominant Pole P1 at -2 + 5.216i', ...
       'Dominant Pole P2 at -2 - 5.216i', 'Location', 'Best');
grid on;

%% Angle and Magnitude Calculation


G = tf(1,[1, 10, 24, 0]); %uncompensated open loop
sys = feedback(G,1); %uncompensated closed loop
stepinfo(sys)
j = sqrt(-1);
Mp = 0.3;
ts = 2;
zeta = sqrt(log(Mp)^2/(pi^2 + log(Mp)^2));
wn = 4/(zeta*ts);                                                                                                                                                                                
ts2 = 4/(zeta*wn);
sd1 = -zeta*wn + j*wn*sqrt(1-zeta^2);
sd2 = -zeta*wn - j*wn*sqrt(1-zeta^2);
cos_theta = (zeta*wn)/sqrt((zeta*wn)^2 + (wn*sqrt(1-zeta^2))^2);
%placing zero directly below the lead compensator
zero = real(sd1);
%finding p usign angle criterion
angle_0 = angle(polyval([1,0],sd1))*180/pi;
angle_4 = angle(polyval([1,4],sd1))*180/pi;
angle_6 = angle(polyval([1,6],sd1))*180/pi;
disp('Angle Criterion');
disp([angle_0, angle_4, angle_6]);
theta_zero = 90;
theta_p = 180 - angle_6 - angle_4 - angle_0 + theta_zero;
pole = zero - imag(sd1)/tan(theta_p*pi/180);
%Finding gain using magnitude criterion;
mag_0 = abs(polyval([1,0],sd1));
mag_4 = abs(polyval([1,4],sd1));
mag_6 = abs(polyval([1,6],sd1));
mag_zero = abs(polyval([1,-zero],sd1)); 
mag_pole = abs(polyval([1,-pole],sd1));
disp('Magnitude Criterion');
disp([mag_0, mag_4, mag_6, mag_zero, ]);
K = mag_pole*mag_0*mag_4*mag_6/mag_zero; %gain of compensator
lead_compensator = zpk(zero,pole,1);
comp_sys = series(G,lead_compensator);
comp_sys = K*comp_sys;


%% Root Locus of the Compansated System

% Define the numerator and denominator of the transfer function
num = [337.6101 337.6101 * 2];  % 337.6101*(s + 2)
den = conv([1 0], conv([1 4], conv([1 6], [1 8.809])));  % s*(s + 4)*(s + 6)*(s + 8.809)

% Create the transfer function
sys = tf(num, den);

% Plot the root locus
figure;
rlocus(sys);
title('Root Locus of the Compansated System');
xlabel('Real Axis (seconds^{-1})');
ylabel('Imaginary Axis (seconds^{-1})');

% Customization for marking points (Update these values if you have specific dominant poles)
hold on;

% Example Dominant poles (Modify these if you have specific poles to mark)
P1 = -2 + 5.216i;  % Dominant Pole 1
P2 = -2 - 5.216i;  % Dominant Pole 2
plot(real(P1), imag(P1), 'x', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'LineWidth', 2);
plot(real(P2), imag(P2), 'x', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 2);

set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2);

% Add legend
legend('Root Locus of the System', ...
       'Dominant Pole P1', ...
       'Dominant Pole P2', 'Location', 'Best');
grid on;

%% Step Response of the Compensated System

% Define the numerator and denominator of the compensated system transfer function
num = [337.61 337.61 * 2];  % 337.61*(s + 2)
den = conv([1 0], conv([1 4], conv([1 6], [1 8.809])));  % s*(s + 4)*(s + 6)*(s + 8.809)

% Create the transfer function for the open-loop system
sys_open_loop = tf(num, den);

% Create the closed-loop transfer function with unit negative feedback
sys_closed_loop = feedback(sys_open_loop, 1);

% Compute the step response and step information
[y, t] = step(sys_closed_loop);
info = stepinfo(sys_closed_loop);

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

%% Bode Plot

% Define the numerator and denominator of the compensated system transfer function
num = [337.61 337.61 * 2];  % 337.61*(s + 2)
den = conv([1 0], conv([1 4], conv([1 6], [1 8.809])));  % s*(s + 4)*(s + 6)*(s + 8.809)

% Create the transfer function for the open-loop system
sys_open_loop = tf(num, den);

% Create the closed-loop transfer function with unit negative feedback
sys_closed_loop = feedback(sys_open_loop, 1);

% Plot the Bode plot of the closed-loop system
figure;

% Plot Bode magnitude and phase separately
[mag, phase, freq] = bode(sys_closed_loop);

% Convert magnitude to dB and squeeze dimensions for plotting
mag_dB = squeeze(20*log10(mag));
phase_deg = squeeze(phase);

% Magnitude Plot
subplot(2, 1, 1);
semilogx(freq, mag_dB, 'b-', 'LineWidth', 2); % Plot magnitude in dB
title('Bode Plot of the Closed-Loop Compensated System', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Magnitude (dB)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 12, 'FontWeight', 'bold');
xlim([min(freq(:)) max(freq(:))]); % Set x-axis limits based on frequency range

% Phase Plot
subplot(2, 1, 2);
semilogx(freq, phase_deg, 'r-', 'LineWidth', 2); % Plot phase in degrees
xlabel('Frequency (rad/s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Phase (degrees)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 12, 'FontWeight', 'bold');
xlim([min(freq(:)) max(freq(:))]); % Set x-axis limits based on frequency range

%% Bode Plot of the Uncompensated System

%From the requirement of steady state error, we found that
K1 = 15;

%open loop transfer function
G = tf(1,[1,1,0]);
uncompensated_sys = K1*G;

%Bode plot K1G(s)
hFigure = figure(1);
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
hAxes = axes(hPanel, 'Color', 'none'); 
[mag, phase,wout]=bode(uncompensated_sys);
title("Bode Digram of Open loop transfer function with the gain K1");
margin(uncompensated_sys)
set(findall(gcf,'type','line'),'linewidth',2);
grid on

%% Automated Process for finding the Compensated System

Pm_req = 45;
[GM_u,PM_u, w_pc_u, w_gc_u] = margin(uncompensated_sys);
PM_extra = (Pm_req - PM_u );
PM = PM_u;
w_gc = w_gc_u;

%Automating the procees of increasing Phi_m to get PM >=45 deg
i = 0;
while( PM<=45 || w_gc>=7.5)    
    i = i+1;
    PM_add = PM_extra*(1+(0.1*i)); %adding extra 10% to PM, later increase to 20% to get the required gain
    sine = sind(PM_add);
    %Finding a=z/p
    a = (1+sine)/(1-sine);
    %Placing w_m at new 0dB frequency.
    w_m = a^(1/4)*w_gc_u;
    %finding compensator pole and zero;
    pole = w_m*sqrt(a);
    zero = pole/a;
    K = K1*a;
    %designing the compensator ;
    G_c = K*tf([1,zero],[1,pole]);
    hFigure = figure(i);
    hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
    hAxes = axes(hPanel, 'Color', 'none'); 
    compensated_sys = G_c*G;
    bode(compensated_sys)
    margin(compensated_sys)
    set(findall(gcf,'type','line'),'linewidth',2)
    grid on
    [GM,PM,w_pc,w_gc]=margin(compensated_sys);
end

%% Final Compensated System

%From the requirement of steady state error, we found that
K1 = 1;

%open loop transfer function
G = tf(1,[1,1,0]);
uncompensated_sys = K1*G;


PM_Final = PM;
w_gc_final = w_gc;
    hFigure = figure(4);
    hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
    hAxes = axes(hPanel, 'Color', 'none'); 
    compensated_sys = G_c*G;
    bode(compensated_sys)
    margin(uncompensated_sys)
    hold on;
    margin(compensated_sys)
    legend('Uncompensated System', 'Compensated System'); % Add a legend to differentiate the curves
    set(findall(gcf,'type','line'),'linewidth',2)
    grid on

%% Compensator Bode Plot

num = 51.5620*[1 2.7973];
den = [1 9.6157];
compensator_sys = tf(num,den);

    hFigure = figure(4);
    hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  
    hAxes = axes(hPanel, 'Color', 'none'); 
    bode(compensator_sys)
    set(findall(gcf,'type','line'),'linewidth',2, 'Color', 'r');
    grid on
%% Ramp Input
% Define the transfer function Gc(s)G(s)
num = [58.6522 158.9]; % Numerator coefficients of Gc(s)G(s)
den = [1 11.59 10.59 0]; % Denominator coefficients of Gc(s)G(s)

% Create the transfer function for the open-loop system
sys_open_loop = tf(num, den);

% Create the closed-loop transfer function with unit negative feedback
GcGs = feedback(sys_open_loop, 1);

G1 = tf(1,[1,1,0]); %Uncompensated Ramp Response
G = feedback(G1, 1);

% Define the time vector for the simulation
t = 0:0.01:10; % Time from 0 to 100 seconds with a step of 0.01s

% Define the ramp input
ramp_input = t; % Ramp input is a linear increase with time

% Simulate the response of the system to the ramp input
ramp_response = lsim(GcGs, ramp_input, t);
ramp_response_Raw = lsim(G, ramp_input, t);

% Plot the response and the ramp input
figure;
plot(t, ramp_input, 'r--', 'LineWidth', 2); % Plot ramp input in red dashed line
hold on; % Hold the plot to overlay the system response
plot(t, ramp_response, 'g-', 'LineWidth', 2); % Plot system response in green solid line
hold on;
plot(t, ramp_response_Raw, 'b-', 'LineWidth', 2); % Plot Uncompansated system response in blue solid line
title('Response of the Compensated System to a Ramp Input', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Response', 'FontSize', 12, 'FontWeight', 'bold');
legend('Ramp Input', 'Compensated System Response', 'UnCompensated System Response'); % Add a legend to differentiate the curves
grid on;
hold off; % Release the hold on the plot

%% Checking the steady state error
sys = feedback(compensated_sys,1);
t = 0:0.001:10;
s = tf('s');
hFigure = figure(5);
hPanel = uipanel(hFigure, 'Units', 'normalized', ...
                 'Position', [0 0 1 1], ...
                 'BorderType', 'line', ...
                 'BorderWidth', 2, ...
                 'BackgroundColor', 'w', ...
                 'HighlightColor', 'k');  % Make a white panel with red line borders
hAxes = axes(hPanel, 'Color', 'none');  % Set the axes background color to none
step(sys/s)
title("Unit Ramp response of the compensated system")
set(findall(gcf,'type','line'),'linewidth',2)
ramp_inp = t;
ramp_resp =lsim(sys,ramp_inp,t);
ss_error = abs(ramp_inp(end)-ramp_resp(end))*100;
%% Velocity constant
Kv = (1/ss_error)*100;
