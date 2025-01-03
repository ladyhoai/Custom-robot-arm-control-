clc;
clear all;
close all;

L1 = Link('d', 0.1965, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-180,180]), 'offset', -pi);       % Rotates about z-axis (yaw)
L2 = Link('d', 0, 'a', 0.2, 'alpha', 0, 'qlim', deg2rad([-180 180]), 'offset', pi/2);    % Rotates about x-axis (pitch)
L3 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-180 180]), 'offset', pi/2);       % Rotates about x-axis (pitch)
L4 = Link('d', 0.3, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-180 180]), 'offset', 0);    % Rotates about x-axis (pitch)
L5 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180,180]), 'offset',0);
L6 = Link('d',0.08,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

global currentPose;
global alphabet;
% Arc parameter order: [startPointX startPointY, radius, arcDegree, placeHolderToIdentifyArcFromLine]
% during line drawing, -1 mean not raising pen
alphabet = {{'A', [0 0 5 10 -1], [10 0 1], [2.5 5 7.5 5 1]}, ...
            {'B', [0 0 0 10 -1], [3 10 -1], [2.5 -pi -1 -1], [2.5 -pi -1 -1], [0 0 1], [3 5 0 5 1]}, ...
            {'C', [5 10 5 pi -1 1]}, ...
            {'D', [0 0 0 10 -1], [5 -pi -1 1]}, ...
            {'E', [5 10 0 10 -1], [0 0 -1], [5 0 1], [0 5 5 5 1]}, ...
            {'F', [0 0 0 10 -1], [5 10 1], [0 5 5 5 1]}, ...
            {'G', [5 10 5 pi -1 -1], [5 5 -1], [3 5 1]}, ...
            {'H', [0 0 0 10 1], [10 0 10 10 1], [0 5 10 5 1]}, ...
            {'I', [5 0 5 10 1], [2.5 0 7.5 0 1],[2.5 10 7.5 10 1]}, ...
            {'J', []}, ...
            {'K', [0 0 0 10 1], [0 5 5 10 1], [0 5 5 0 1]}, ...
            {'L', [0 10 0 0 -1], [0 0 10 0 1]}, ...
            {'M', [0 0 0 10 -1], [5 5 -1], [10 10 -1], [10 0 1]}, ...
            {'N', [0 0 0 10 -1], [10 0 -1], [10 10 1]}, ...
            {'O', [5 10 5 2*pi -1 1]}, ...
            {'P', [0 0 0 10 -1], [3 10 -1], [2.5 -pi -1 1]}, ...
            {'Q', []}, ...
            {'R', [0 0 0 10 -1], [3 10 -1], [2.5 -pi -1 -1], [7 0 1]},  ...
            {'S', [5 10 2.5 5*pi/6 -1 -1], [2.5 -6*pi/5 -1 1]}, ...
            {'T', [5 0 5 10 1], [0 10 10 10 1]}, ...
            {'U', [2.5 10 2.5 0 -1], [7.5 0 -1], [7.5 10 1]}, ...
            {'V', [0 10 5 0 -1], [10 10 1]}, ...
            {'W', [0 10 2.5 0 -1], [5 7 -1], [7.5 0 -1], [10 10 1]}, ...
            {'Y', [0 0 10 10 1], [0 10 5 5 1]}, ...
            {'X', [0 0 10 10 1], [0 10 10 0 1]}, ...
            {'Z', [0 10 10 10 -1], [0 0 -1], [10 0 1]}};

global robot;
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'myRobot');

workspace = [-4 4 -4 4 -0.05 2]; 

scale = 0.5;

robot.teach;
pause(1);
global q;
q = [0, 0, pi/2, 0, -pi/2, 0];
% q = zeros(1, 6);
robot.plot(q);
hold on;
global newQ;
newQ = [];

% Setup the serial port
arduinoPort = serialport("/dev/cu.usbmodemF412FA6335602", 115200); % Replace "COM3" with your Arduino port
configureTerminator(arduinoPort, "CR/LF"); % Arduino commonly uses Carriage Return + Line Feed
flush(arduinoPort);
configureCallback(arduinoPort, "terminator", @(src, event) backgroundSerialReader(src));
disp('Press enter to continue');

pause;

while true

    selProgram = input('R for RRMC nav, N for joint control, C draw circle ', 's');

    if strcmpi(selProgram, 'n') 
    
        % Prompt for a new target position
        x = input('Enter x position (meters): ');
        y = input('Enter y position (meters): ');
        z = input('Enter z position (meters): ');
    
        newQ = robot.ikcon(transl(x, y, z) * trotx(pi),q);

        disp(rad2deg(newQ));
        disp(newQ);
        % Now plot this new joint state
        robot.plot(newQ)
        
        % Example joint angles in radians (replace with actual values)
        joint_angles = newQ; % Replace with your joint angles in radians
        
        % Convert angles from radians to degrees
        joint_angles_deg = rad2deg(joint_angles);
        
        % Initialize the result string
        result = '';
        
        % Parse Joint 1
        if joint_angles_deg(1) < 0
            result = [result, sprintf('q%03d', abs(round(joint_angles_deg(1))))];
        else
            result = [result, sprintf('a%03d', abs(round(joint_angles_deg(1))))];
        end
        
        % Parse Joint 2
        if joint_angles_deg(2) < 0
            result = [result, sprintf('w%03d', abs(round(joint_angles_deg(2))))];
        else
            result = [result, sprintf('s%03d', abs(round(joint_angles_deg(2))))];
        end
        
        % Parse Joint 3
        if joint_angles_deg(3) < 0
            result = [result, sprintf('e%03d', abs(round(joint_angles_deg(3))))];
        else
            result = [result, sprintf('d%03d', abs(round(joint_angles_deg(3))))];
        end

        if joint_angles_deg(5) < 0
            result = [result, sprintf('f%03d', abs(round(joint_angles_deg(5))))];
        else
            result = [result, sprintf('r%03d', abs(round(joint_angles_deg(5))))];
        end
        
        % Display the result string
        disp(['Resulting String: ', result]);
    
        send_choice = input('Do you want to send joint angle to the Arduino? (y/n): ', 's');
        if strcmpi(send_choice, 'y')
            % Send the string to the Arduino via serial
            writeline(arduinoPort, result);
            disp('String sent to Arduino.');
        end
    
         exit_choice = input('Do you want to exit? (y/n): ', 's');
        if strcmpi(exit_choice, 'y')
            q = newQ;
            break;
        end
    
        q = newQ;

    elseif strcmpi(selProgram, 'r') 
        x = input('Enter x offset (meters): ');
        y = input('Enter y offset (meters): ');
        z = input('Enter z offset (meters): ');

        goal = [x y z]';

        jointVel = drawLineXY(goal, 0);
        rowToRM = any(jointVel > 3 | jointVel < -3, 2);
        jointVel = jointVel(~rowToRM, :);

        disp(jointVel);
        jointVel = [jointVel; [0 0 0 0 0 0]];
        cfTraj = input('Send line trajectory to Arm? (y/n): ', 's');
        
        if strcmpi(cfTraj, 'y')
            writeline(arduinoPort, "t1");
            pause(0.5);

            for i = 1:size(jointVel, 1)
                first = (jointVel(i, 1) * 200 * 8 * 6) / (2 *pi); %rotate base
                sec = (jointVel(i, 2) * 200 * 4 * 5 * 6) / (2 *pi); % shoulder 1
                third = (jointVel(i, 3) * 200 * 5 * 6) / (2 *pi); % shoulder 2
                forth = (jointVel(i, 5) * 200 * 32 * 6) / (2 *pi); % wrist 2
                jointToSend = int16([first sec third -forth]);
                dataBytes = typecast(jointToSend, 'uint8');
                pause(0.05);
                write(arduinoPort, dataBytes, "uint8");
            end
        end

    elseif strcmpi(selProgram, 'c') 
        x = input('Enter circle center X offset (meters): ');
        y = input('Enter circle center Y offset (meters): ');
        z = input('Enter Z offset (meters): ');
        radius = input('Enter circle radius (meters): ');
        jointVel = drawCircleXY([x y z], radius, 2*pi);

        rowToRM = any(jointVel > 3 | jointVel < -3, 2);
        jointVel = jointVel(~rowToRM, :);

        disp(jointVel);

        jointVel = [jointVel; [0 0 0 0 0 0]];
        cfTraj = input('Send circular trajectory to Arm? (y/n): ', 's');

        if strcmpi(cfTraj, 'y')
            writeline(arduinoPort, "t1");
            pause(0.5);

            for i = 1:200
                first = (jointVel(i, 1) * 200 * 8 * 6) / (2 *pi); %rotate base
                sec = (jointVel(i, 2) * 200 * 4 * 5 * 6) / (2 *pi); % shoulder 1
                third = (jointVel(i, 3) * 200 * 5 * 6) / (2 *pi); % shoulder 2
                forth = (jointVel(i, 5) * 200 * 32 * 6) / (2 *pi); % wrist 2
                jointToSend = int16([first sec third -forth]);
                dataBytes = typecast(jointToSend, 'uint8');
                pause(0.05);
                write(arduinoPort, dataBytes, "uint8");
            end
        end
    
    elseif strcmpi(selProgram, 'write')
        % sentence = input('Enter character to draw: ');
        % writeSen(['S'], 0);
        jointTraj = [];
        jointTraj = [jointTraj; writeSen(['M', 'E', 'R', 'R', 'Y'], 0)];
        % jointTraj = [jointTraj; writeSen(['C', 'H', 'R', 'I', 'S', 'T', 'M', 'A', 'S'], -0.07)];
        % disp(jointTraj);
        rowToRM = any(jointTraj > 3 | jointTraj < -3, 2);
        jointTraj = jointTraj(~rowToRM, :);
        writematrix(jointTraj, "qMatrix.txt");
        jointTraj = [jointTraj; [0 0 0 0 0 0]];

        cfTraj = input('Write physical letter? (y/n): ', 's');

        if strcmpi(cfTraj, 'y')
            writeline(arduinoPort, "t1");
            pause(0.5);

            for i = 1:size(jointTraj, 1)
                first = (jointTraj(i, 1) * 200 * 8 * 6) / (2 *pi); %rotate base
                sec = (jointTraj(i, 2) * 200 * 4 * 5 * 6) / (2 *pi); % shoulder 1
                third = (jointTraj(i, 3) * 200 * 5 * 6) / (2 *pi); % shoulder 2
                forth = (jointTraj(i, 5) * 200 * 32 * 6) / (2 *pi); % wrist 2
                jointToSend = int16([first sec third -forth]);
                dataBytes = typecast(jointToSend, 'uint8');
                pause(0.05);
                write(arduinoPort, dataBytes, "uint8");
            end
        end

    end
end

function backgroundSerialReader(arduinoPort)
    data = readline(arduinoPort); % Read the line of data
    disp(['Received: ', data]); % Print the data
end

function res = writeSen(sentence, yPos)
    location = 0.3;
    jointTraj = [];
    for i = 1:length(sentence)
        jointTraj = [jointTraj; drawCharacter(sentence(i), [location, yPos, 0.15]', 400)];
        location = location + 0.035;
    end
    res = jointTraj;
end

function res = drawCharacter(character, origin, scale)
    global alphabet;
    global currentPose;
    velArray = [];

    for i = 1:length(alphabet)
        if alphabet{i}{1} == character
            for y = 2:length(alphabet{i})
                if length(alphabet{i}{y}) == 5 % this is a line
                    velArray = [velArray; drawLineXY([origin(1) + alphabet{i}{y}(1)/scale, origin(2) + alphabet{i}{y}(2)/scale, 0.15]', 0)];
                    velArray = [velArray; drawLineXY([origin(1) + alphabet{i}{y}(1)/scale, origin(2) + alphabet{i}{y}(2)/scale, 0.05]', 0)];
                    velArray = [velArray; drawLineXY([origin(1) + alphabet{i}{y}(3)/scale, origin(2) + alphabet{i}{y}(4)/scale, 0.05]', 1)];

                    if alphabet{i}{y}(5) ~= -1
                        velArray = [velArray; drawLineXY([origin(1) + alphabet{i}{y}(3)/scale, origin(2) + alphabet{i}{y}(4)/scale, 0.15]', 0)];
                    end

                elseif length(alphabet{i}{y}) == 3
                    velArray = [velArray; drawLineXY([origin(1) + alphabet{i}{y}(1)/scale, origin(2) + alphabet{i}{y}(2)/scale, 0.05]', 1)];

                    if alphabet{i}{y}(3) ~= -1
                        velArray = [velArray; drawLineXY([origin(1) + alphabet{i}{y}(1)/scale, origin(2) + alphabet{i}{y}(2)/scale, 0.15]', 0)];
                    end

                elseif length(alphabet{i}{y}) == 6
                    velArray = [velArray; drawLineXY([origin(1) + alphabet{i}{y}(1)/scale, origin(2) + alphabet{i}{y}(2)/scale, 0.15]', 0)];
                    velArray = [velArray; drawLineXY([origin(1) + alphabet{i}{y}(1)/scale, origin(2) + alphabet{i}{y}(2)/scale, 0.05]', 0)];
                    velArray = [velArray; drawCircleXY([0 0 0], alphabet{i}{y}(3)/scale, alphabet{i}{y}(4))];
                    
                    if alphabet{i}{y}(6) ~= -1
                        velArray =[velArray; drawLineXY([origin(1) + alphabet{i}{y}(1)/scale, origin(2) + alphabet{i}{y}(2)/scale, 0.15]', 0)];
                    end
                elseif length(alphabet{i}{y}) == 4
                    velArray = [velArray; drawCircleXY([0 0 0], alphabet{i}{y}(1)/scale, alphabet{i}{y}(2))];
                    if alphabet{i}{y}(4) ~= -1
                        velArray = [velArray; drawLineXY([origin(1) + currentPose(1, 4)/scale, origin(2) + currentPose(2, 4)/scale, 0.15]', 0)];
                    end
                end

            end
            break;
        end
    end
    disp('Done');
    res = velArray;
end

function res = drawLineXY(goal, visualize)
    global robot;
    global q;
    global newQ;
    global currentPose;

    steps = 200;
    deltaT = 0.05; % Discrete time step
        
    x = zeros(3,steps); % for translation
    rotTraj = zeros(3, steps); % 3 rows for roll, pitch, yaw
    currentPose = robot.fkine(newQ).T;
    currentTrans = currentPose(1:3, 4);
    currentRot = currentPose(1:3, 1:3);
    
    RGoal = rpy2r(deg2rad(180), deg2rad(0), deg2rad(0));
    s = lspb(0,1,steps); % Create interpolation scalar

    for i = 1:steps
        x(:,i) = currentTrans*(1-s(i)) + s(i)*goal; % Create trajectory in x-y
        Ri = currentRot * (1 - s(i)) + RGoal * s(i); % Interpolate between rotation matrices
        rotTraj(:,i) = tr2rpy(Ri); % Convert back to roll, pitch, yaw
    end

    qMatrix = nan(steps,6);
    jointVel = [];
    qMatrix(1, :) = q;

    for i = 1:steps-1
        % robot.animate(qMatrix(i, :));
        q = qMatrix(i, :);
        newQ = qMatrix(i, :);
        xdot = (x(:,i+1) - x(:,i))/deltaT; % Calculate
        xdot_rot = (rotTraj(:,i+1) - rotTraj(:,i)) / deltaT; % Rotational velocity
        xdot = [xdot; xdot_rot];
        J = robot.jacob0(qMatrix(i,:)); % Get the Jacobian at the current state
        qdot = inv(J)*xdot; % Solve velocitities via RMRC
        jointVel = [jointVel; qdot'];
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot'; % Update next

        % currentPose = robot.fkine(qMatrix(i, :)).T; % Forward kinematics
        % currentEndEffectorPosition = currentPose(1:3, 4); % Extract position
        % 
        % if visualize == 1
        %     % Plot the current end-effector position
        %     plot3(currentEndEffectorPosition(1), currentEndEffectorPosition(2), currentEndEffectorPosition(3), 'ro','MarkerSize', 1);
        %     drawnow;
        % end
    end
   res = jointVel;
end

function res = drawCircleXY(center, radius, arcDeg)
        % Get inputs
    global robot;
    global q;
    global newQ;
    global currentPose;

    steps = 200;
    deltaT = 0.05; % Discrete time step
    
    % Preallocate for trajectory and rotation
    x = zeros(3, steps); % Translation trajectory
    rotTraj = zeros(3, steps); % Roll, pitch, yaw trajectory
    
    % Current pose
    currentPose = robot.fkine(newQ).T;
    currentTrans = currentPose(1:3, 4);
    currentRot = currentPose(1:3, 1:3);
    
    % Target rotation
    RGoal = rpy2r(deg2rad(180), deg2rad(0), deg2rad(0));

    if arcDeg < 0
        s = linspace(-arcDeg/2, arcDeg/2, steps); % Parametric angle for the circle
    else
        s = linspace(arcDeg/2, 3*arcDeg/2, steps); % Parametric angle for the circle
    end
    
    % Create circular trajectory
    for i = 1:steps
        x(:, i) = [center(1) + radius * cos(s(i));
                   center(2) + radius * sin(s(i));
                   center(3)]; % Circle in XY plane at height z
        Ri = currentRot * (1 - s(i)/(2*pi)) + RGoal * (s(i)/(2*pi)); % Interpolate rotation
        rotTraj(:, i) = tr2rpy(Ri); % Convert rotation matrix to RPY
    end

    trajX = x(1, :);
    trajY = x(2, :);
    trajZ = x(3, :);
    plot3(trajX, trajY, trajZ, '-o', 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', 'r');

    % Initialize joint configuration matrix
    qMatrix = nan(steps, 6);
    jointVel = [];
    qMatrix(1, :) = q;
    
    % Generate trajectory using RMRC
    for i = 1:steps-1
        % robot.animate(qMatrix(i, :)); % Animate robot
        q = qMatrix(i, :);
        newQ = qMatrix(i, :);
        xdot = (x(:, i+1) - x(:, i)) / deltaT; 
        xdot_rot = (rotTraj(:, i+1) - rotTraj(:, i)) / deltaT; 
        xdot = [xdot; xdot_rot]; % Combine translational and rotational velocities
        J = robot.jacob0(qMatrix(i, :)); % Get Jacobian
        qdot = inv(J) * xdot; % Solve for joint velocities
        % disp(qdot);
        jointVel = [jointVel; qdot'];
        qMatrix(i+1, :) = qMatrix(i, :) + deltaT * qdot'; % Update joint positions

        % currentPose = robot.fkine(qMatrix(i, :)).T; % Forward kinematics
        % currentEndEffectorPosition = currentPose(1:3, 4); % Extract position
        % 
        % % Plot the current end-effector position
        % plot3(currentEndEffectorPosition(1), currentEndEffectorPosition(2), currentEndEffectorPosition(3), 'ro','MarkerSize', 1);
        % drawnow;
    end
    % disp(qMatrix);
    res = jointVel;
end
