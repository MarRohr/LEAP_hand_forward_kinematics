% Plot frames of LEAP-Hand
robot = importrobot('robot.urdf');

config = homeConfiguration(robot);
jointPositions = [config.JointPosition];

% Add random number to zero vector of joint positions
randomOffsets = -0.5 + (0.5 - (-0.5)) * rand(size(jointPositions));
newJointPositions = jointPositions + randomOffsets;

for i = 1:length(config)
    config(i).JointPosition = newJointPositions(i);
end

show(robot, config,Frames="off")
hold on;

% forward kinematics
EE = forward_kinematics_LEAP_DH(newJointPositions);

EE1 = EE{1};
EE2 = EE{2};
EE3 = EE{3};
EEt = EE{4};

EE1_pos = EE1(1:3,4);
EE2_pos = EE2(1:3,4);
EE3_pos = EE3(1:3,4);
EEt_pos = EEt(1:3,4);

% plot contact points
plot3(EE1_pos(1),EE1_pos(2),EE1_pos(3),'b*')
hold on;
plot3(EE2_pos(1),EE2_pos(2),EE2_pos(3),'b*')
hold on;
plot3(EE3_pos(1),EE3_pos(2),EE3_pos(3),'b*')
hold on;
plot3(EEt_pos(1),EEt_pos(2),EEt_pos(3),'b*')

% plot contact frames
hold on;
plotCoordinateSystem(EE1_pos, EE1(1:3,1:3), 0.01);
hold on;
plotCoordinateSystem(EE2_pos, EE2(1:3,1:3), 0.01);
hold on;
plotCoordinateSystem(EE3_pos, EE3(1:3,1:3), 0.01);
hold on;
plotCoordinateSystem(EEt_pos, EEt(1:3,1:3), 0.01);

function plotCoordinateSystem(origin, rotationMatrix, axisLength)
    % plotting a coordinate system based on a rotation matrix

    % extract the axes from the rotation matrix
    xAxis = rotationMatrix(:, 1) * axisLength;
    yAxis = rotationMatrix(:, 2) * axisLength;
    zAxis = rotationMatrix(:, 3) * axisLength;
    
    % draw x-axis (red)
    quiver3(origin(1), origin(2), origin(3), ...
            xAxis(1), xAxis(2), xAxis(3), ...
            'r', 'LineWidth', 2, 'MaxHeadSize', 1.5);
    hold on;
    
    % draw y-axis (green)
    quiver3(origin(1), origin(2), origin(3), ...
            yAxis(1), yAxis(2), yAxis(3), ...
            'g', 'LineWidth', 2, 'MaxHeadSize', 1.5);
    
    % draw z-axis (blue)
    quiver3(origin(1), origin(2), origin(3), ...
            zAxis(1), zAxis(2), zAxis(3), ...
            'b', 'LineWidth', 2, 'MaxHeadSize', 1.5);
    
    % labels
    text(origin(1) + xAxis(1), origin(2) + xAxis(2), origin(3) + xAxis(3), 'X', 'Color', 'r');
    text(origin(1) + yAxis(1), origin(2) + yAxis(2), origin(3) + yAxis(3), 'Y', 'Color', 'g');
    text(origin(1) + zAxis(1), origin(2) + zAxis(2), origin(3) + zAxis(3), 'Z', 'Color', 'b');
    
end
