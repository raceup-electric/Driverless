function robotRefState = helper(purePursuitObj, robotPose, robotGoal, goalRadius)
% Get controller output
controller = purePursuitObj;
[vRef, wRef] = controller(robotPose);

% Stop controller when goal is reached
distanceToGoal = norm(robotPose(1:2) - robotGoal(:));
if distanceToGoal < goalRadius
    vRef = 0;
    wRef = 0;
end

robotRefState = [vRef; wRef];
end