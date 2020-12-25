%%  Path Planning in a 2d Environment

%% Defining the Robot Model

R = 0.05;                % Wheel radius [m]
L = 0.15;                % Wheelbase [m]

dd = DifferentialDrive(R,L);    %Declaring the Robot Configuration

%% Setting the Simulation parameters

sampleTime = 0.05;              % Sample time [s]
tVec = 0:sampleTime:22;         % Time array

initPose = [2;2;0];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

%% Path planning

inflate(map,R); 

% Creating a Probabilistic Road Map Planner

planner = mobileRobotPRM(map);
planner.NumNodes = 250;
planner.ConnectionDistance = 5;

% Declaring the Start and Goal Position

startPoint = initPose(1:2)';
goalPoint  = [18, 27];

% Finding a path from the start point to a specified goal point

waypoints = findpath(planner,startPoint,goalPoint);
show(planner)

%% Implementing the Pure Pursuit Controller

controller = controllerPurePursuit;     %Initialising the controller
controller.Waypoints = waypoints;       %Specifying the waypoints to follow
controller.LookaheadDistance = 0.35;    
controller.DesiredLinearVelocity = 1.5;
controller.MaxAngularVelocity = 1.5;


%% Creating a Visualizer

viz = Visualizer2D;  
viz.hasWaypoints = true;
viz.mapName = 'map';

%% Simulation loop

r = rateControl(1/sampleTime);  %Setting the simulaiton to run at a fixed frequency.

for idx = 2:numel(tVec) 
    
    % Run the Pure Pursuit controller and convert output to wheel speeds
    
    %Obtain the reference robot velocities from the controller.
    [vRef,wRef] = controller(pose(:,idx-1));  
    
    %Obtain the wheel velocities from inverse kinematics.
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the actual velocities 
    [v,w] = forwardKinematics(dd,wL,wR);
    
    % Convert from robot frame to world frame
    bodyVel = [v;0;w]; % Robot velocities [vx;vy;w]
    vel = bodyToWorld(bodyVel,pose(:,idx-1));  
    
    % Performing forward discrete integration step (euler method)
    % Updating the values of robot pose for each time step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    
    %Robot Data
    wheelVel = [wL, wR];
    save('WheelVelocities.csv', 'wheelVel', '-ASCII','-append');
    writematrix(pose,'RobotPose.csv') 
    save('LinearVelocity.csv', 'v', '-ASCII','-append');
    save('AngularVelocity.csv', 'w', '-ASCII','-append');
    waitfor(r);
end