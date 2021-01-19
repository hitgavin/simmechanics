%PLOT_ROBOT plot 2 links planner robot
%
%        plot_robot
%
% function is used to plot a 2 link planner robot
% PLOT_ROBOT(LINK, Q) plot a 2 links robot, LINK is a 2d vector of link 
% length, Q is a 2d vector of joint angles.
%

% Copyright (C) 2020-2021, by gavin
function [x_e, J] = plot_robot(link, q)
l1 = link(1);
l2 = link(2);
theta1 = q(1);
theta2 = q(2);

x0 = 0;
y0 = 0;

x1 = l1*cosd(theta1);
y1 = l1*sind(theta1);

x2 = x1 + l2 * cosd(theta2);
y2 = y1 + l2 * sind(theta2);

plot([x0 x1], [y0 y1], [x1 x2], [y1 y2], 'linewidth', 3);
text(0.5*(x0+x1),0.5*(y0+y1),' Link 1');
text(0.5*(x1+x2),0.5*(y1+y2),' Link 2');
% jacobian matrix
J = [-y2, y1 - y2; 
     x2 , x2 - x1];
J = inv(J*J' + eye(2)*0.0001);
x_e = [x2, y2];
end
