%Forward kinematics of 2 link Manipulator 
% L1 = 10;
% L2 = 10; %link lengths
% 
% %initial conditions for the joint angles 
% theta1_init =0;
% theta2_init =0;
% 
% %final required angle to be given 
% theta1_final = 90;
% theta2_final = 0;
% 
% th1 = linspace(theta1_init,theta1_final,100);
% th2 = linspace(theta2_init,theta2_final,100);
% 
% v = VideoWriter('Forward_kinematics.m.avi'); %writes the video images .
% open(v);
% 
% for i=1:1000
% 
%     %DH parameters
%     a = [0 L1 L2];            %link lengths 
%     alpha = [0 0 0];          %link Twists 
%     d = [0 0 0];              %link offsets
%     th = [th1(i),th2(i),0];   %joint angle 
%     %now calculating the transformation matrices 
%     c1 = cosd(th1(i)); s1 = sind(th1(i));c2 = cosd(th2(i));s2 = sind(th2(i));
%     T1 = [c1 -s1 0 a(1);s1 c1 0 0;0 0 1 0;0 0 0 1];
% 
%     T2 = [c2 -s2 0 a(2);s2 c2 0 0;0 0 1 0;0 0 0 1];
%     
%     T3 = [1 0 0 a(3);0 1 0 0;0 0 1 0;0 0 0 1];
% 
%     Joint1_pos =T1*T2*[0;0;0;1];          %by formula we get the position of the joint 1
%     Joint2_pos = T1*T2*T3*[0;0;0;1];      %by formula we get the position of the joint 2       
%     X = [0,Joint1_pos(1),Joint2_pos(1)];
%     Y = [0,Joint1_pos(2),Joint2_pos(2)];
% 
%     hg = hggroup;
%     plot(X,Y,'-bo','LineWidth',3,'MarkerEdgeColor','k','Parent',hg);
%     xlabel('x-coordinate(units)');
%     ylabel('y-coordinate(units)');
%     title('Forward Kinematics of 2R linkage');
%     pause(0.05);
%     axis([-20 20 -20 20]);
%     axis manual;
%     grid on;
%     hold on;
% 
%     frame = getframe(gcf); % adding frame to video gcf-- current figure angle.
% 
%     if i<100   %by preventing the deletion of the last ploting
%     delete(hg);
%     end
%  
% 
% end
% close(v);


clear;
clc;
l1 = 10;
l2 = 10;

theta1_init = 0;
theta2_init = 0;

theta1_f = 60;
theta2_f = 60;
th1 = linspace(theta1_init,theta1_f,100);
th2 = linspace(theta2_init,theta2_f,100);

v = VideoWriter("Forward_K.avi");
open(v);

for i=1:100
    a = [0 l1 l2];

    c1 = cosd(th1(i)); s1 =sind(th1(i)); c2 =cosd(th2(i)); s2 =sind(th2(i));

    T1 = [c1 -s1 0 a(1);
        s1 c1 0 0;
        0 0 1 0;
        0 0 0 1];

    T2 = [c2 -s2 0 a(2);
        s2 c2 0 0;
        0 0 1 0;
        0 0 0 1];
    
    T3 = [1 0 0 a(3);
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

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
