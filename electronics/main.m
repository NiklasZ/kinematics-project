% MAE C163A/C263A Project
% Team 9
% NOTE to Niklas: on Linux, need to give access open USB port with "sudo chmod 666 /dev/ttyUSB0" and set ttyUSB0 in initialize.m

% Set fixed vars, accessible anywhere. Mostly so we don't have to pass them
% into every function call
global ADDR_PRESENT_POSITION ADDR_PRESENT_VELOCITY ADDR_GOAL_POSITION ADDR_DRIVE_MODE ADDR_HOMING_OFFSET ADDR_OPERATING_MODE
global ANGLE_OFFSETS % motor offsets in terms of degrees, indexed by motor ID. Depends on the resting state of the motor
global ALLOWED_ANGLES % min-max range allowed angles allowed for each joint in radians, indexed by motor ID.
global PROTOCOL_VERSION
global MX28_ID % Dynamixel ID: 1 2 3 4 5 6
global MOVING_STATUS_THRESHOLD % at what distance we consider a joint's position to be close enough to the target position.
global port_num % set by initialisation
global COMM_SUCCESS % indicates if last motor communication was succcessful
global TORQUE_DISABLE
global TORQUE_ENABLE
global MX28_TORQUE_ENABLE
global VELOCITY_CONTROL_MODE % velocity-based operating mode
global ADDR_GOAL_VELOCITY % the address of the velocity to set

ADDR_PRESENT_POSITION = 132; 
ADDR_PRESENT_VELOCITY = 128;
ADDR_OPERATING_MODE = 11;
ADDR_DRIVE_MODE = 10;
ADDR_GOAL_VELOCITY = 104;
ADDR_GOAL_POSITION = 116;
ADDR_HOMING_OFFSET = 20;


VELOCITY_CONTROL_MODE = 1;
ANGLE_OFFSETS = [0 0 0 0 0 0]; 
ALLOWED_ANGLES = [[-1 -1],[0 180],[-1 -1],[-1 -1],[-1 -1]]; 
PROTOCOL_VERSION = 2.0;
MX28_ID = [1 2 3 4 5 6];
MOVING_STATUS_THRESHOLD = 40;
COMM_SUCCESS = 0; 
TORQUE_DISABLE = 0;
TORQUE_ENABLE = 1;
MX28_TORQUE_ENABLE = 64;


% Initialize
initialize();
input('Press any key to continue!');
num_motors = 6;

% Main
setVelocityMode(MX28_ID)

try
    initial_angs = getAngles(MX28_ID)
    initial_pos = getPositions(MX28_ID);
    
    % -- FLOUR TRAJECTORY -- %
    % Angles adapted from the simulation's, but needed to be fine-tuned by
    % hand to work effectively in the real world.
    % starting setup
    start_angles = [5, 176, 203, 215];
    thetas(:) = start_angles(:);
    moveJoints([2], [thetas(2)], [0])
    moveJoints([1],[thetas(1)],[0])
    moveJoints([3], [thetas(3)], [0])
    moveJoints([4], [thetas(4)], [0])

    % Phase 1 - make arm upright
    thetas(2) = thetas(2) - 90;
    moveJoints([2], [thetas(2)], [0])
    thetas(3) = thetas(3) - 170;
    moveJoints([3], [thetas(3)], [-1])
    thetas(4) = thetas(4) + 120;
    moveJoints([4], [thetas(4)], [0])

    % Phase 2 - move scooper in
    thetas(2) = thetas(2) + 40;    

    moveJoints([2], [thetas(2)], [0])

    % Phase 3 - scoop
    thetas(1) = thetas(1) + 7;
    thetas(4) = thetas(4) - 120;
    thetas(2) = thetas(2) + 6;
    moveJoints([1,2,4], [thetas(1),thetas(2),thetas(4)], [0,0,0], [20,20,50])

    % Phase 3.5 - shuffle the flour
    thetas(4) = thetas(4) - 45;
    moveJoints([4], [thetas(4)], [0])
    thetas(4) = thetas(4) + 45;
    moveJoints([4], [thetas(4)], [0])

    % Phase 4 - lift scoop back out
    thetas(2) = thetas(2) - 46;
    moveJoints([2], [thetas(2)], [-1], [30])

    % Phase 5 - move to deposit
    thetas(1) = thetas(1) + 83;
    moveJoints([1], [thetas(1)], [0])
    pause(0.1)

    % Phase 6 - deposit
    thetas(2) = thetas(2) + 25;
    thetas(3) = thetas(3) + 20;
    moveJoints([2,3], [thetas(2),thetas(3)], [0,0], [30,50])
    thetas(4) = thetas(4) - 180;
    moveJoints([4], [thetas(4)], [-1])

    % Phase 7 - shake
    thetas(4) = thetas(4) - 90;
    moveJoints([4], [thetas(4)], [-1], [100])
    thetas(4) = thetas(4) + 180;
    moveJoints([4], [thetas(4)], [1], [200])
    
    % Phase 7 - return to start
    moveJoints([1,3,4], [start_angles(1),start_angles(3),start_angles(4)], [0,0,0])
    moveJoints([2], [start_angles(2)], [0])
   
    % final setup 
    final_angs = getAngles(MX28_ID)


    % -- ICE CREAM TRAJECTORY -- %

%     % starting setup
%     start_angles = [5, 176, 203, 215];
%     thetas(:) = start_angles(:);
%     moveJoints([2], [thetas(2)], [0])
%     moveJoints([1],[thetas(1)],[0])
%     moveJoints([3], [thetas(3)], [0])
%     moveJoints([4], [thetas(4)], [0])
% 
%     % Phase 1 - make arm upright
%     thetas(2) = thetas(2) - 90;
%     moveJoints([2], [thetas(2)], [0])
%     thetas(3) = thetas(3) - 150;
%     moveJoints([3], [thetas(3)], [-1])
%     thetas(4) = thetas(4) + 120;
%     moveJoints([4], [thetas(4)], [0])
% 
%     % Phase 2 - move scooper in
%     thetas(2) = thetas(2) + 30;    
%     moveJoints([2], [thetas(2)], [0])
%     thetas(3) = thetas(3) - 30;
%     moveJoints([3], [thetas(3)], [0])
% 
%     % Phase 3 - scoop
%     thetas(1) = thetas(1) + 7;
%     thetas(4) = thetas(4) - 120;
%     thetas(2) = thetas(2) + 6;
%     moveJoints([1,2,4], [thetas(1),thetas(2),thetas(4)], [0,0,0], [20,20,50])
% 
%     % Phase 4 - lift scoop back out
%     thetas(2) = thetas(2) - 46;
%     moveJoints([2], [thetas(2)], [0], [50])
%     pause(0.2)
% 
%     % Phase 5 - move to deposit
%     thetas(1) = thetas(1) + 83;
%     moveJoints([1], [thetas(1)], [0], [30])
% 
%     % Phase 6 - deposit
%     thetas(2) = thetas(2) + 25;
%     thetas(3) = thetas(3) + 20;
%     moveJoints([2,3], [thetas(2),thetas(3)], [0,0], [30,50])
%     thetas(4) = thetas(4) - 180;
%     moveJoints([4], [thetas(4)], [-1])
% 
%     % Phase 7 - shake
%     thetas(4) = thetas(4) - 90;
%     moveJoints([4], [thetas(4)], [-1], [100])
%     thetas(4) = thetas(4) + 180;
%     moveJoints([4], [thetas(4)], [1], [150])
%     
%     % Phase 7 - return to start
%     moveJoints([1,3,4], [start_angles(1),start_angles(3),start_angles(4)], [0,0,0])
%     moveJoints([2], [start_angles(2)], [0])

% -- Stretched Poses -- %
%     % starting setup
%     pause(10)
%     start_angles = [5, 176, 113, 215];
%     thetas(:) = start_angles(:);
%     moveJoints([2], [thetas(2)], [0])
%     moveJoints([1],[thetas(1)],[0])
%     moveJoints([3], [thetas(3)], [0])
%     moveJoints([4], [thetas(4)], [0])
%     pause(2)
% 
%     thetas(2) = thetas(2) - 80;
%     moveJoints([2], [thetas(2)], [0])
%     
%     thetas(3) = thetas(3) - 100;
%     moveJoints([3], [thetas(3)], [-1])
%     thetas(3) = thetas(3) + 180;
%     moveJoints([3], [thetas(3)], [1])
% 
%     pause(2)
% 
%     thetas(3) = thetas(3) - 90;
%     moveJoints([3], [thetas(3)], [0])
% 
%     thetas(2) = thetas(2) - 96;
%     moveJoints([2], [thetas(2)], [0], [40])


   
    % final setup 
    final_angs = getAngles(MX28_ID)
    
    input('Press any key to terminate!');
catch e
    fprintf('ERROR:\n')
    fprintf(getReport(e))
end

% Stop all running motors at end
for i=1:numel(MX28_ID)
    setVelocity(MX28_ID(i),0)
end
terminate();


% execute movements in parallel
% directions: -1 is clockwise, 1 is counter-clockwise and 0 is whichever is the
% shortest path
function x = moveJoints(joint_ids, target_angles, directions, speeds)
    % If no custom speeds are specified, use default
    if ~exist('speeds','var')
        speeds = ones(length(joint_ids)) * 50;
    end
    delay = 0.1;
    goal_positions = zeros(length(joint_ids),1);

    % Check that there are no repeated joint ids
    if numel(joint_ids)~=numel(unique(joint_ids))
        error('cannot move the same joint id multiple times in a call.')
    end
    
    % Get goal positions
    for i = 1:numel(joint_ids)
        goal_positions(i) = angleToMotorPosition(target_angles(i));
    end
    
    % Move joints until the positions have been reached
    while 1
        for i = 1:numel(joint_ids)
            monitorVelocity(joint_ids(i), goal_positions(i), speeds(i), directions(i))
        end
        pause(delay)
        velocities = getVelocities(joint_ids);
        if max(abs(velocities)) == 0
            break;
        end
    end
    pause(0.2);
end

function out = getAngles(joint_ids)
    out = zeros(length(joint_ids),1);
    positions = getPositions(joint_ids);
    for i = 1:numel(joint_ids)
        out(i) = motorPosToAngle(positions(i));
    end    
end

% Get multiple joint positions (0-4095)
function out = getPositions(joint_ids)
    out = zeros(length(joint_ids),1);
    for i = 1:numel(joint_ids)
        out(i) = getPosition(joint_ids(i));
    end
end

% Get single joint position (0-4095)
function out = getPosition(joint_id)
    global port_num PROTOCOL_VERSION MX28_ID ADDR_PRESENT_POSITION ANGLE_OFFSETS
    pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(joint_id), ADDR_PRESENT_POSITION);
    out = round(mod(pos - ANGLE_OFFSETS(joint_id)*4095/360, 4095)); % in velocity mode the position can be infinitely large. We only care about the position with respect to the angle.
end


% Get multiple joint velocities (-inf,+inf)
function out = getVelocities(joint_ids)
    out = zeros(length(joint_ids),1);
    for i = 1:numel(joint_ids)
        out(i) = getVelocity(joint_ids(i));
    end
end

% Get single velocity (-inf,+inf)
function out = getVelocity(joint_id)
    global port_num PROTOCOL_VERSION MX28_ID ADDR_PRESENT_VELOCITY
    out = read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(joint_id), ADDR_PRESENT_VELOCITY);
end

function out = setVelocity(joint_id, velocity)
    global port_num PROTOCOL_VERSION ADDR_GOAL_VELOCITY
    v = typecast(int32(velocity),'uint32');
    write4ByteTxRx(port_num, PROTOCOL_VERSION, joint_id, ADDR_GOAL_VELOCITY, v);
%     debugLastTxRxResult()
end


% Moves all joints to home position 0 angle
function out = toHome()
    global port_num PROTOCOL_VERSION MX28_ID ADDR_GOAL_POSITION IDLE_STATE
    for i = 1:6
        write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), ADDR_GOAL_POSITION, angleToMotorPosition(IDLE_STATE(i)));
    end
    pause(1);
end

function out = debugLastTxRxResult()
    global port_num PROTOCOL_VERSION COMM_SUCCESS
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
end

function out = motorPosToAngle(motorPositions)
    angles = zeros(length(motorPositions),1);
    for i = 1:numel(motorPositions)
        angles(i) = motorPositions(i) / 4095 * 360;
    end
    out = angles;
end

function out = setVelocityMode(ids)
    global port_num PROTOCOL_VERSION MX28_TORQUE_ENABLE TORQUE_ENABLE TORQUE_DISABLE ADDR_OPERATING_MODE VELOCITY_CONTROL_MODE 

    for i = 1:numel(ids)
        write1ByteTxRx(port_num, PROTOCOL_VERSION, ids(i), MX28_TORQUE_ENABLE, TORQUE_DISABLE);
        write1ByteTxRx(port_num, PROTOCOL_VERSION, ids(i), ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE);
        write1ByteTxRx(port_num, PROTOCOL_VERSION, ids(i), MX28_TORQUE_ENABLE, TORQUE_ENABLE);
    end
end

function out = monitorVelocity(id, goal_position, speed, direction)
    global MOVING_STATUS_THRESHOLD
    distance = getDistanceToGoal(id, goal_position);
    [closest_distance, min_idx] = min(distance);
    
    % Set clockwise velocity
    if direction == -1
        distance = distance(1);
        velocity = speed * direction;
    % Set counter-clockwise velocity
    elseif direction == 1
        distance = distance(2);
        velocity = speed * direction;
    % Pick whichever has the shortest path.
    elseif direction == 0
        distance = closest_distance;
        if min_idx == 1
           velocity = speed * -1;
        else
            velocity = speed;
        end
    else
        error('unknown direction')
    

    end

    min_v = 20*sign(velocity);
    % Stop once we are at the target
    if closest_distance <= MOVING_STATUS_THRESHOLD
        setVelocity(id, 0)
        %pause(0.2)
    % Min speed once we are near the target
    elseif distance < abs(velocity*6)
        setVelocity(id, min_v)
    else
        setVelocity(id, velocity)
    end
end

function out = getDistanceToGoal(id, goal_position)
    current_position = getPosition(id);
    % distance assuming positive velocity (counter-clockwise)
    if goal_position >= current_position
        ccw_out = goal_position - current_position;
    else
        ccw_out = goal_position + 4095 - current_position;
    end

    % distance assuming negative velocity (clockwise)
    if goal_position >= current_position
        cc_out = goal_position + 4095 - current_position;
    else
        cc_out = current_position - goal_position;
    end

    out = [cc_out ccw_out];
end