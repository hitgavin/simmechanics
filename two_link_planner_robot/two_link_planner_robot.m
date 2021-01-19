% TWO_LINK_PLANNER_ROBOT Simulate operability of a two links planner robot.
%
%        two_link_planner_robot
%
% script is used to simulate operability of a two links planner robot.
%
% Notes::
% - Based on my CSDN blog
%   If you are interested in robotics, welcome to my blog
%   https://blog.csdn.net/hitgavin.

% Copyright (C) 2020-2021, by gavin
l = [0.5,0.5];
q1 = linspace(0,90,20);
q2 = linspace(0,90,20);

syms x y
ct = 1;
for i = 1:length(q1)
    theta1 = q1(i);
    for j = 1:length(q2)
        theta2 = q2(j);
        [x_e, J] = plot_robot(l, [theta1,theta2]);
        hold on;
        grid on;
        j1 = J(1,1);
        j2 = J(1,2);
        j3 = J(2,1);
        j4 = J(2,2);
        x2 = x_e(1);
        y2 = x_e(2);
        % plot operability ellipse v_e^T * (J*J^T)^{-1} * v_e = 1
        ezplot(j1 * (x - x2)^2 + j2 * (x - x2) * (y - y2) + j3 * (x - x2) * (y - y2) + j4 * (y - y2)^2 - 1);
        axis([-1 1.5 -1 1.5]);
        title('two link planner robot');
        hold off;
        pause(0.3);
        
        F = getframe(gcf);
        I = frame2im(F);
        [I, map] = rgb2ind(I, 256);
        if ct == 1
            imwrite(I,map,'./doc/two_link_planner_robot.gif','gif','Loopcount',inf,'DelayTime',0.2);
        else
            imwrite(I,map,'./doc/two_link_planner_robot.gif','gif','WriteMode','append','DelayTime',0.2);
        end
        ct = ct + 1;
    end
end