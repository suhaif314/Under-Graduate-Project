

% Define the forward kinematics function
function [x, y] = forward_kinematics_cspace(theta1, theta2,l1,l2)
    x1=l1*cos(theta1);
    y1=l1*sin(theta2);
    x2 = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
    y2 = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
    x = [0;x1;x2];
    y = [0;y1;y2];
    
end
