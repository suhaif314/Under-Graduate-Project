
% Define the Rectangle Obstacle.
X1 = [5,18,18,5,5];         
Y1 = [-5,-5,-16,-16,-5];
fill(X1,Y1,'red')
hold on
%Define the triangle obstacle.
X2 = [-15 -5 -15,-15];
Y2 = [18,5,5,18];
fill(X2,Y2,'blue')

%Define the circle obstacle.

R3 = 5;
theta_ = linspace(0,2*pi,1000);
X3 = (5+R3*sin(theta_))';
Y3 = (5+R3*cos(theta_))'; 
fill(X3,Y3,'yellow')
axis([-20 20 -20 20])


%2=Rlink

grid on

l1 = 10;
l2 = 8;
 
% Define the range of joint angles
theta1_range = linspace(0, 2*pi, 1000);
theta2_range = linspace(0, 2*pi, 1000);


P1t1 = [];
P1t2 = [];
P2t1 = [];
P2t2 = [];
P3t1 = [];
P3t2 = [];
for i = 1:length(theta1_range)
    for j = 1:length(theta2_range)
        tiledlayout(1,2)
        [x, y] = forward_kinematics_cspace(theta1_range(i), theta2_range(j),l1,l2);
        plot(x,y,'black')
        pause(1e-9)
        axis([-20 20 -20 20])
        
        nexttile
        if inpolygon(x, y, X1, Y1)
            P1t1 = [P1x, theta1_range(i)];
            P1t2 = [P1y, theta2_range(j)];
            
        elseif inpolygon(x,y,X2,Y2)
            P2t1 = [P2x, theta1_range(i)];
            P2t2 = [P2y, theta2_range(j)];

        elseif inpolygon(x,y,X3,Y3)
            P3t1 = [P3x, theta1_range(i)];
            P3t2 = [P3y, theta2_range(j)];
        end

        plot(P1t2,P1t1, '.', 'Color', 'red')
        hold on
        plot(P2t2,P2t1,'.','Color','blue')
        plot(P3t2,P3t1,'.','Color','yellow')
        hold off

        xlabel('Theta2');
        ylabel('Theta1');
        axis([0 2*pi 0 2*pi])
        grid on 
        pause(1e-9)
        
    end
end