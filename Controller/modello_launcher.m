% add Image to the path
addpath(genpath('Images'));

%%
%    load("waypoints_real.mat");
%Ts = 41; % simulation time

 load("waypoints_gen.mat");
 Ts = 59; % simulation time
% sample time = 105

% load("waypoints_sim.mat");
% Ts = 50; % simulation time


%%

bicycle= bicycleKinematics();

controller1 = controllerPurePursuit(DesiredLinearVelocity=3, Waypoints=waypoints);

goalPoints = waypoints(end,:)';
goalRadius = 1;

sampleTime = 0.05;               % Sample time [s]
tVec = 0:sampleTime:105;          % Time array
initPose = [waypoints(1,:)'; 0]; % Initial pose (x y theta)

[tBicycle,bicyclePose] = ode45(@(t,y)derivative(bicycle,y,helper(controller1,y,goalPoints,goalRadius)),tVec,initPose);

bicycleTranslations = [bicyclePose(:,1:2) zeros(length(bicyclePose),1)];
bicycleRot = axang2quat([repmat([0 0 1],length(bicyclePose),1) bicyclePose(:,3)]);


% load the scene data file generated from Driving Scenario Designer

% define reference points
xRef = bicycleTranslations(:,1);
yRef = bicycleTranslations(:,2);

%xRef(end+1) = 17.2402000000000;
%yRef(end +1)= 3.19899000000000;

bicycleTranslations([1],:) = [];

xWay = bicycleTranslations(:,1);
yWay = bicycleTranslations(:,2);


% define reference time for plotting 
s = size(xRef);
tRef = (linspace(0,Ts,s(1)))'; % this time variable is used in the "2D Visualization" block for plotting the reference points. 

% define parameters used in the models
L = 3; % bicycle length
ld = 5; % lookahead distance
X_o = bicycleTranslations(1,1); % initial vehicle position
Y_o = bicycleTranslations(1,2); % initial vehicle position 
psi_o = 0; % initial yaw angle

