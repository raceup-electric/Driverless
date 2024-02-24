close all;

% Note per Fede, quando provi a buttare questo codice dentro ROS, ricordati di togliere le plot 

yellow = readcell('gt_YellowCones.txt')
yellow = cell2mat(yellow)

blue = readcell('gt_BlueCones.txt')
blue = cell2mat(blue)

innerConePosition = yellow;
outerConePosition = blue;

%% Preprocessing the data

[m,nc] = size(innerConePosition); % size of the inner/outer cone positions data
P = zeros(2*m,nc); % initiate a P matrix consisting of inner and outer coordinates
P(1:2:2*m,:) = innerConePosition;
P(2:2:2*m,:) = outerConePosition; % merge the inner and outer coordinates with alternate values
xp = []; % create an empty numeric xp vector to store the planned x coordinates after each iteration
yp = []; % create an empty numeric yp vector to store the planned y coordinates after each iteration

% Forming Triangles:

interv = 3; % interval        %FUNNZIONA SOLAMENTE CON interv=3
for i = interv:interv:2*m
 DT = delaunayTriangulation(P(((abs((i-1)-interv)):i),:)); % create Delaunay triangulation for abs((i-1)-interv)):i points
 Pl = DT.Points; % coordinates of abs((i-1)-interv)):i vertices
 Cl = DT.ConnectivityList; % triangulation connectivity matrix
 [mc,nc] = size(Pl); % size
 if rem(interv,2) == 0
 cIn = [2 1;(1:2:mc-3)' (3:2:(mc))'; (mc-1) mc];
 cOut = [(2:2:(mc-2))' (4:2:mc)'];
 else
 % inner and outer constraints when the interval is odd
 cIn = [2 1;(1:2:mc-2)' (3:2:(mc))'; (mc-1) mc];
 cOut = [(2:2:(mc-2))' (4:2:mc)'];
 end

C = [cIn;cOut];

TR = delaunayTriangulation(Pl,C); % Delaunay triangulation with constraints

TRC = TR.ConnectivityList; % triangulation connectivity matrix
TL = isInterior(TR); % logical values that indicate whether the triangles are inside the bounded region
TC = TR.ConnectivityList(TL,:); % triangulation connectivity matrix


 [~,pt] = sort(sum(TC,2)); % optional step. The rows of connectivity matrix are arranged in ascending sum of rows...
 % This ensures that the triangles are connected in progressive order.
 TS = TC(pt,:); % connectivity matrix based on ascending sum of rows
 TO = triangulation(TS,Pl);


 xPo = TO.Points(:,1);
 yPo = TO.Points(:,2);
 E = edges(TO); % triangulation edges
 iseven = rem(E, 2) == 0; % neglect boundary edges
 Eeven = E(any(iseven,2),:);
 isodd = rem(Eeven,2) ~=0;
 Eodd = Eeven(any(isodd,2),:);
 xmp = ((xPo((Eodd(:,1))) + xPo((Eodd(:,2))))/2); % x coordinate midpoints
 ymp = ((yPo((Eodd(:,1))) + yPo((Eodd(:,2))))/2); % y coordinate midpoints
 Pmp = [xmp ymp]; % midpoint coordinates final vector

 distancematrix = squareform(pdist(Pmp));
 distancesteps = zeros(length(Pmp)-1,1);
 for j = 2:length(Pmp)
 distancesteps(j-1,1) = distancematrix(j,j-1);
 end
 totalDistance = sum(distancesteps); % total distance travelled
 distbp = cumsum([0; distancesteps]); % distance for each waypoint
 gradbp = linspace(0,totalDistance,100); % linspace(X1, X2, N) generates N points between X1 and X2
 xq = interp1(distbp,xmp,gradbp,'spline'); % interpolate x coordinates
 yq = interp1(distbp,ymp,gradbp,'spline'); % interpolate y coordinates
 xp = [xp xq]; % store obtained x midpoints after each iteration
 yp = [yp yq]; % store obtained y midpoints after each iteration
 waypoints = [xp.', yp.']; % Transposed because xp and yp are of dimensions 1x200 

 %% Delaunay plot
 figure(3)
 % subplot
 pos1 = [0.1 0.15 0.5 0.7];
 subplot('Position',pos1)
 pathPlanPlot(yellow,blue,P,DT,TO,xmp,ymp,cIn,cOut,xq,yq)
 title(['Path planning based on constrained Delaunay' newline ' triangulation'])
 % subplot
 subplot('Position',pos2)
 pathPlanPlot(yellow,blue,P,DT,TO,xmp,ymp,cIn,cOut,xq,yq)
 xlim([min(min(xPo(1:2:(mc-1)),xPo(2:2:mc))) max(max(xPo(1:2:(mc-1)),xPo(2:2:mc)))])
 ylim([min(min(yPo(1:2:(mc-1)),yPo(2:2:mc))) max(max(yPo(1:2:(mc-1)),yPo(2:2:mc)))])

end

plot(waypoints(:,1),waypoints(:,2),"kx-",MarkerSize=5);

%% KINEMATICS
bicycle= bicycleKinematics();

%Sim Params

% Define the total time and the sample rate
sampleTime = 0.05;               % Sample time [s]
tVec = 0:sampleTime:90;          % Time array
initPose = [waypoints(3,:)'; 0]; % Initial pose (x y theta)

%% Controller

controller1 = controllerPurePursuit(DesiredLinearVelocity=3, Waypoints=waypoints);

goalPoints = waypoints(end,:)';
goalRadius = 1;

[tBicycle,bicyclePose] = ode45(@(t,y)derivative(bicycle,y,helper(controller1,y,goalPoints,goalRadius)),tVec,initPose);

%% Plots

bicycleTranslations = [bicyclePose(:,1:2) zeros(length(bicyclePose),1)];
bicycleRot = axang2quat([repmat([0 0 1],length(bicyclePose),1) bicyclePose(:,3)]);

%%

figure
plot(waypoints(:,1),waypoints(:,2),"kx-",MarkerSize=5);
hold all
plotTransforms(bicycleTranslations(1:15:end,:),bicycleRot(1:15:end,:),MeshFilePath="groundvehicle.stl",MeshColor="b");
axis equal
view(0,90)


