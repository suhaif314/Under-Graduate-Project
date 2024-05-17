%RECTANGLE ONSTACLE:
x = [10,18,18,10,10];
y = [-10,-10,-16,-16,-10];
plot(x,y)
hold on 

%tRIANGLE OBSTACLE: 

x2 = [-15 -6 -15,-15];
y2 = [18,10,10,18];
plot(x2,y2,'b')


%CIRCLE OBSTACLE:

theta = linspace(0,2*pi,1000);
x3 = 15+2.5*sin(theta);
y3 = 10+2.5*cos(theta);
plot(x3,y3,'LineWidth',1,"Color",'r');
hold on

n = 1000;
R = 2.5;

                       % Now create the set of points.
t31 = 2*pi*rand(n,1);
r31 = R*sqrt(rand(n,1));
x31 = 15 + r31.*cos(t);
y31 = 10 + r.*sin(t);

%Plot the random points.
plot(x31,y31, 's', 'MarkerSize', 1);
axis square;
grid on;

axis([-20 20 -20 20])
axis equal

% X3 = [x3;x31];
% Y3 = [y3,y31];


l1 = 10;
l2 = 10;

theta1_init = 0;
theta2_init = 0;

% v = VideoWriter("cspace.avi");
% open(v);
% for theta1_f = linspace(0,360,10)
%     for theta2_f = linspace(0,360,10)
%         th1 = linspace(theta1_init,theta1_f,100);
%         th2 = linspace(theta2_init,theta2_f,100);
% 
%        
% 
%         for i=1:100
%             a = [0 l1 l2];
% 
%             c1 = cosd(th1(i)); s1 =sind(th1(i)); c2 =cosd(th2(i)); s2 =sind(th2(i));
% 
%             T1 = [c1 -s1 0 a(1);
%                 s1 c1 0 0;
%                 0 0 1 0;
%                 0 0 0 1];
% 
%             T2 = [c2 -s2 0 a(2);
%                 s2 c2 0 0;
%                 0 0 1 0;
%                 0 0 0 1];
%     
%             T3 = [1 0 0 a(3);
%                 0 1 0 0;
%                 0 0 1 0;
%                 0 0 0 1];
% 
%             Joint1_pos = T1*T2*[0;0;0;1];
%             Joint2_pos = T1*T2*T3*[0;0;0;1];
%             X = [0 Joint1_pos(1) Joint2_pos(1)];
%             Y = [0 Joint1_pos(2) Joint2_pos(2)];
% 
%             hg = hggroup;
%             plot(X,Y,'-bo','LineWidth',3,'MarkerEdgeColor','k','Parent',hg);
%             pause(0.0005);
%             axis([-20 20 -20 20]);
%             axis manual;
%             grid on;
%             
% 
%             frame = getframe(gcf);
%             writeVideo(v,frame);
% 
%             if i<100
%                 delete(hg);
% 
%             end
% 
%         end
%     end
% end
% 
% close(v);
hold off
