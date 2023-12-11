%% Connection to the ROS_MASTER_URI of the robot

ROS_MASTER_URI = "http://192.168.1.151:11311"; %The ROS_MASTER_URI is different for each robot
rosinit(ROS_MASTER_URI); % We initialize the connection with the ROS of the robot 
                         % ROSCORE need to be launched in the robot before
                         % running rosinit
                

%% Disconnection from the ROS_MASTER_URI of the robot

rosshutdown % Shutdown the connection with the ROS of the robot
            % This command needs to be executed everytime the ROSCORE of
            % the robot is interrupted OR everytime we want to connect to
            % another robot

