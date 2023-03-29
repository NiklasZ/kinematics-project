function [t1,t2,t3,t4] = IK(T0e,c,t1_current)
% Inverse kinematics
    
    %% Extract inputs
    R = T0e(1:3,1:3);
    p = T0e(1:3,4);
    d1 = c(1);
    L2 = c(2);
    L3 = c(3);
    dE = c(4);
    
    %% Initial infeasibility tests
    e = 10^-15; % machine zero fudge factor
    if any(abs([R(1,3), R(2,3), R(3,1), R(3,2), R(3,3)]) > 1+e)
        error('Infeasible desired orientation (1)!')
    elseif sqrt(R(1,3)^2+R(2,3)^2) > 1+e || sqrt(R(3,1)^2+R(3,2)^2) > 1+e
        error('Infeasible desired orientation (2)!')
    end

    %% t23
    t23 = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));

    %% t2
    s2 = (p(3) - dE*cos(t23) - L3*sin(t23) - d1)/L2;
    c2 = (-sqrt(p(1)^2 + p(2)^2) + dE*sin(t23) - L3*cos(t23))/L2;
    if abs(s2) <= 1+e && abs(c2) <= 1+e
        t2 = atan2(s2, c2);
    else
        error('infeasible t2')
    end

    %% t3
    t3 = t23 - t2;

    %% t1, t4
    if (R(1,3)~=0) || (R(2,3)~=0)   % Can calculate t1,t4 directly
        s1 = -R(2,3)/sin(t23); c1 = -R(1,3)/sin(t23);
        s4 = -R(3,2)/sin(t23); c4 =  R(3,1)/sin(t23);
        if abs(s1) <= 1+e && abs(c1) <= 1+e ...
        && abs(s4) <= 1+e && abs(c4) <= 1+e
            t1 = atan2(s1, c1);
            t4 = atan2(s4, c4);
        else
            error('infeasible, nonsingular t1,t4')
        end
    else                            % Must get t4 from t1
        if (p(1)~=0) || (p(2)~=0)
            r = L2*cos(t2) + L3*R(3,3);
            s1 = p(2)/r;
            c1 = p(1)/r;
                if abs(s1) <= 1+e && abs(c1) <= 1+e
                    t1 = atan2(s1, c1);
                else
                    error('infeasible')
                end
        else                        % Singularity case
            t1 = t1_current;
        end
        
        % Calculate t4 from t1
        if R(3,3) == 1 % c23 = 1
            t4 = atan2(R(2,1),R(1,1)) - t1;
        elseif R(3,3) == -1 % c23 = -1
            t4 = atan2(R(2,1),-R(1,1)) + t1;
        else
            error('Check singularity calcs')
        end
    end

end