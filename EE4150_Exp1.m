%Problem 1
m = 1;
b = 4;
% We can put b=0 for Undamped Case
% We can vary b from 0 to 2 for Under-Damped Case
% We can put b = 2 for Critically-Damped Case
% We can vary b from 2 to infinity for Over-Damped Case
k = 1;
G = tf([1],[m b k]);
figure;
pzmap(G);
xlim([-1.5 0]);
%%
%Problem 2
m = 1;
b = 4;
k = 1;
figure;
step(G); %Step Response
title('Step Response');
grid on;
%%
figure;
impulse(G); %Impulse Response
title('Impulse Response');
grid on;
%%
t = 0 : 0.01 : 20;
figure;
lsim(G,t,t); %Ramp Response
title('Ramp Response')
grid on;
%%
Step_Out = step(G);
Ess_step = 1 - Step_Out(115);
disp(['Ess_Step = ', num2str(Ess_step)]); %Outputs Step Steady State Error

Impluse_out = impulse(G);
Ess_impulse = Impluse_out(115);
disp(['Ess_Impulse = ', num2str(Ess_impulse)]); %Outputs Impulse Steady State Error

Ramp_out = lsim(G, t, t);
Ess_ramp = t(2001) - Ramp_out(2001);
disp(['Ess_Ramp = ', num2str(Ess_ramp)]); %Outputs Ramp Steady State Error

%%
% for parallel damping elements b is doubled and for series element b is halved for the same value of b.
% Hence the steady state errors will also follow the same.
%%
m = 1;
b = 0;
k = 1;

G = tf([1],[m b k]);
figure;
step(G);
xlim([0,20]);
title('Undamped System');
grid on;
%%
%Problem 3
m = 1;
b = 1;
k = 1;
G = tf([1],[m b k]);
[p,t] = step(G);
stepinfo(G)
%%
%Delay time is the time to reach 50% of the steady state value
for i = 1 : 1 : 101
    if p(i)<0.5
        Delay_time = t(i);
    end
end

disp(['Delay Time = ' , num2str(Delay_time)]);
%%
%Under Damped
m = 1;
b = 1;
k = 1;
G = tf([1],[m b k]);
figure;
sgtitle('Under-Damped');
subplot(1,2,1);
margin(G);
grid on;
subplot(1,2,2);
nyquist(G);
grid on;
%%
%Critically Damped
m = 1;
b = 2;
k = 1;
G = tf([1],[m b k]);
figure;
sgtitle('Critically-Damped');
subplot(1,2,1);
margin(G);
grid on;
subplot(1,2,2);
nyquist(G);
grid on;
%%
%Over Damped
m = 1;
b = 5;
k = 1;
G = tf([1],[m b k]);
figure;
sgtitle('Over-Damped');
subplot(1,2,1);
margin(G);
grid on;
subplot(1,2,2);
nyquist(G);
grid on;

%The turn-on and turn-off time delay is dead time. Controller gain have to
%be increased to make the system stable when de3ad time dealay is there
