% % Should be 1024
% t1 = angleToMotorPosition(90)
% % Should be 3072
% t2 = angleToMotorPosition(-90)
% % Should be 3072
% t3 = angleToMotorPosition(-450)
% % Should be 0
% t4 = angleToMotorPosition(360)

function pos = angleToMotorPosition(theta)
    s = sign(theta);
    reduced = mod(theta, 360*s); % Reduce to -360 <= theta <= 360 range
    if reduced < 0 
       reduced = reduced + 360; % Reduce to 0 <= theta <= 360 range
    end

    pos = typecast(int32(reduced/360*4095),'uint32'); % Map to 0 <= pos <= 4095 
        
end