%Inverse_Kinematics
clear;
clc;

l1 = 5;
l2 = 5;

x = -5;
y = 5;

theta1_init = 0;
theta2_init = 0;

cos2 = (x^2+y^2 - l1^2 - l2^2)/(2*l1*l2) ;
theta2 = acos(cos2);
beta = atan2(y,x);
cos_psi = (x^2+y^2+l1^2-l2^2)/(2*l1*sqrt(x^2+y^2));
psi = acos(cos_psi);
if theta2<0
    theta1 = beta+psi;
else
    theta1 = beta-psi;
end

Sol_degree = [theta1;theta2]*180/pi ;

theta1_f = Sol_degree(1);
theta2_f = Sol_degree(2);
    
th1 = linspace(theta1_init,theta1_f,100);
th2 = linspace(theta2_init,theta2_f,100);

v = VideoWriter("Inverse_K.avi");
open(v);

for i=1:100
    a = [0 l1 l2];

    c1 = cosd(th1(i)); s1 =sind(th1(i)); c2 =cosd(th2(i)); s2 =sind(th2(i));

    T1 = [c1 -s1 0 a(1);s1 c1 0 0;0 0 1 0;0 0 0 1];

    T2 = [c2 -s2 0 a(2);s2 c2 0 0;0 0 1 0;0 0 0 1];
    
    T3 = [1 0 0 a(3);0 1 0 0;0 0 1 0;0 0 0 1];

    Joint1_pos = T1*T2*[0;0;0;1];
    Joint2_pos = T1*T2*T3*[0;0;0;1];
    X = [0 Joint1_pos(1) Joint2_pos(1)];
    Y = [0 Joint1_pos(2) Joint2_pos(2)];

    hg = hggroup;
    plot(X,Y,'-bo','LineWidth',3,'MarkerEdgeColor','k','Parent',hg);
    pause(0.05);
    axis([-20 20 -20 20]);
    axis manual;
    grid on;
    hold on;

    frame = getframe(gcf);
    writeVideo(v,frame);

    if i<100
        delete(hg);

    end
   
end

close(v);




