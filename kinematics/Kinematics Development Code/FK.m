function [fx,fy,fz,T] = FK(c,param)

d1 = c(1); % Height of 2nd motor rotational axis from ground
L2 = c(2); % Direct distance between 2nd and 3rd motor rotational axes
L3 = c(3); % Distance between 3rd and 4th motor rotational axes (12mm)
dE = c(4); % Distance along z4 from z3 (or x3 or x4) to spoon center

t1 = param(1);
t2 = param(2);
t3 = param(3);
t4 = param(4);


% DH parameters
DH = [0       0   d1  t1;
      pi/2    0   0   t2;
      0       L2  0   t3;
      -pi/2   L3  0   t4;
      0       0   dE  0  ];
alpha = DH(:,1); a = DH(:,2); d = DH(:,3); theta = DH(:,4);

% initial
To = eye(4);
fx = 0; fy = 0; fz = 0;
T{1} = To;

for j = 1:5
    
    Ti = [cos(theta(j)) -sin(theta(j)) 0 a(j);
        sin(theta(j))*cos(alpha(j)) cos(theta(j))*cos(alpha(j)) ...
        -sin(alpha(j)) -sin(alpha(j))*d(j);
        sin(theta(j))*sin(alpha(j)) cos(theta(j))*sin(alpha(j)) ...
        cos(alpha(j)) cos(alpha(j))*d(j);0 0 0 1];
    To = To*Ti;
    fx = [fx To(1,4)]; % frame x coordiante
    fy = [fy To(2,4)]; % frame y coordiante
    fz = [fz To(3,4)]; % frame z coordiante
    T{j+1} = To;
    
end

end