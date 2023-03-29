% MAE 263A Project
% Workspace Visualizer
% warning: very janky

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

% Find every feasible combination of t1, t2, t3 with some resolution 2*pi/N
N = 120;
i1max = N/4; % limit this to see cross section
i2max = N/2;
i3max = 3*N/4;

theta = linspace(0,2*pi*(N-1)/N,N);
t1 = zeros(1,i1max*i2max*i3max);
t2 = zeros(1,i1max*i2max*i3max);
t3 = zeros(1,i1max*i2max*i3max);

for i1 = 1:i1max
    for i2 = 1:i2max
        for i3 = 1:i3max
            t1((i1-1)*N^2 + (i2-1)*N + (i3)) = theta(i1);
            t2((i1-1)*N^2 + (i2-1)*N + (i3)) = theta(i2);
            t3((i1-1)*N^2 + (i2-1)*N + (i3)) = theta(i3);
        end
    end
end

a = (L2*cos(t2) + L3*cos(t2+t3) - dE*sin(t2+t3));
x = a.*cos(t1);
y = a.*sin(t1);
z = d1 + L2*sin(t2) + L3*sin(t2+t3) + dE*cos(t2+t3);

figure(1)
plot3(x,y,z,'.')
axis equal
grid on