%% Real Robot Control
function RealRobotControl
    % Initialising a connection to the ROS computer
   % rosinit('192.168.27.1');
    jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
    
    % Getting the current joint state of the real robot
    pause(2); % Pausing to allow for rosmessage to appear
    currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)';
    currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
    
    % Creating variable with joint names
    % Allows joint commands to be associated to particular joints
    jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
    


    % Creating array of joint states to follow
    jointStates = CutPathFucntion();            % PATH FUNCTION
    
    
    
    
    
    % For loop used to send sequential commands to the real UR3 robot
    for i = 1:size(jointStates,1)
        
        % Setting UR3 goal states
        [client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
        goal.Trajectory.JointNames = jointNames;
        goal.Trajectory.Header.Seq = i;
        goal.Trajectory.Header.Stamp = rostime('Now','system');
        goal.GoalTimeTolerance = rosduration(0.05);
        bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
        
        durationSeconds = 5; % This is how many seconds the movement will take
        
        startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        startJointSend.Positions = currentJointState_123456;
        startJointSend.TimeFromStart = rosduration(0);
        endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        
        nextJointState_123456 = jointStates(i,:);
        endJointSend.Positions = nextJointState_123456;
        endJointSend.TimeFromStart = rosduration(durationSeconds);
        
        goal.Trajectory.Points = [startJointSend; endJointSend];
        goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
        
        
        % Sending the command to the UR3
        sendGoal(client,goal);
        pause; % Pausing robot (waits until user input to recieve next command)
        
        % Updating the current joint state to new current position
        currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)';
        currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
    
    end

end