% MAE 263A Project
% Simulation
% NOTE FOR CURRENT TRAJECTORY THERE IS DISCONTINUITY AT T = 4.7 SECONDS

clc;
% clf;
clear all;

% Main
% Parameter
d1 = 0.1;      % [m] Height of 2nd motor rotational axis from ground
L2 = 0.135;     % [m] Direct distance between 2nd and 3rd motor rotational axes
L3 = 0.012;    % [m] Distance between 3rd and 4th motor rotational axes (12mm)
dE = 0.02;       % [m] FIX THIS NUMBER: Distance along z4 from z3 (or x3 or x4) to spoon center
c = [d1, L2, L3, dE];

% Trajectory Cartesian Space (FIX THIS)
N = 101;
t = linspace(0,2*pi,N);

z = (d1+dE)*ones(1,N) - 0.03; %linspace(d1,L3+L2,N);
radius = sqrt((L3+L2)^2 - z.^2);
x = (L2+L3).*cos(t) - 0.08;
y = (L2+L3).*sin(t) - 0.08;

theta1 = zeros(1,N);
theta2 = zeros(1,N);
theta3 = zeros(1,N);
theta4 = zeros(1,N);

% Joint Space
t1_prev = pi/2; % Supposed to be initial t1,  used to help with singularity
for i = 1:N
    R = [cos(t(i)) -sin(t(i)) 0; sin(t(i)) cos(t(i)) 0; 0 0 1]*[sqrt(2)/2 0 -sqrt(2)/2; 0 1 0; sqrt(2)/2 0 sqrt(2)/2];

    p = [x(i) y(i) z(i)]';
    T0e = [R p;0 0 0 1];
    [theta1(i),theta2(i),theta3(i),theta4(i)] = IK(T0e,c,t1_prev);
    t1_prev = theta1(i);
end

t1 = unwrap(theta1);
t2 = unwrap(theta2);
t3 = unwrap(theta3);
t4 = unwrap(theta4);

param = [t1;t2;t3;t4];
path = [x;y;z];

movie = 0; % create movie if 1
speed = 1; % 1 to N

figure(1)
for i = 1:1
    animation(c,param,path,movie,speed)
end