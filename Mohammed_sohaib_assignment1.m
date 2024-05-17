%%%% ME766 - ASSIGNMENT 1 
%%% N MOHAMMED SOHAIB  200611

%%% Generating C-space for 2R, 2 DOF serial arm planer mechanism

clc;
clear;
close all;

%define origin of link
x1 = 0;
y1 = 0;

%Corresponding Link lengths 
L1 = 2;
L2 = 1;

%Corressponding link thickness
T1 = 0.1;
T2 = 0.1;

sgtitle('$\textbf{N Mohammed sohaib, 200611 - ME766 Assignment - 1}$','Interpreter','latex','FontSize',19)

subplot(1,3,1)

title('$\textbf{Euclidean Space}$','Interpreter','latex','FontSize',10)
xlabel('$\textit{\textbf{x-axis}}$','Interpreter','latex','FontSize',10)
ylabel('$\textit{\textbf{y-axis}}$','Interpreter','latex','FontSize',10)

grid on;
hold on;

axis([-5 5 -5 5])
daspect([1 1 1])


%plot the obstacles and the links
 
%%Link1 given some shape to it.
Link_1_s = [-L1/2, -T1/2;
    L1/2, -T1/2;
    L1/2, T1/2;
    -L1/2, T1/2];

%Link 2 given some shape to it.
Link_2_s =[-L2/2, -T2/2;
    L2/2, -T2/2;
    L2/2, T2/2;
    -L2/2, T2/2];

%%Obstacles.
OBS_1 = [1,1;
    1.5,1;
    1.5,1.5;
    1,1.5];

OBS_2 = 1.5.*[1,-1;
    0,-2;
    2,-2];

OBS_3 = .125.*[-3,1;
    -2,1.5;
    0,1.5;
    -1,2;
    0,2.5;
    -2,2.5;
    -3,3;
    -4,2.5;
    -6,2.5;
    -5,2;
    -6,1.5;
    -4,1.5;
    -3,1];

OBS_3 = [OBS_3(:,1),3.*OBS_3(:,2)];

fill(OBS_1(:,1),OBS_1(:,2),'blue');
fill(OBS_2(:,1),OBS_2(:,2),'green');
fill(OBS_3(:,1),OBS_3(:,2),'magenta');


link_1_arm = fill(Link_1_s(:,1),Link_1_s(:,2),'red');
link_2_arm = fill(Link_2_s(:,1),Link_2_s(:,2),'cyan');

%%now generate a configuration space.

th1 = 0 ;   %defining intial config of the link 1 
th2 = 0 ;   %definng intial config of the link 2

for theta_1 = th1:5:360+th1         %for the rotation of the link 1
    for theta_2 = th2:5:360+th2     %for the rotation of the link2
        
        %we will keep updating the geometrical shape of links, which will
        %be reflected in the Euclidean space.
        
        link_1_updated = Link_1_s*[cosd(theta_1),sind(theta_1);
                                   -sind(theta_1),cosd(theta_1)];                  %corresponding link end cordinates.
        link_2_updated = Link_2_s*[cosd(theta_2+theta_1),sind(theta_1+theta_2);
                                   -sind(theta_2+theta_1),cosd(theta_1+theta_2)];  %corresponding link end cordinates.

        X1 = x1+L1/2*cosd(theta_1); % xcord of middle the link 1 
        Y1 = y1 +L1/2*sind(theta_1); % ycord of middle the link 1

        x2 = x1+L1*cosd(theta_1);  % xcord of the end effector/link2
        y2 = y1+L1*sind(theta_1);  %ycord of the end effector/link2

        X2 = x2+L2/1.75*cosd(theta_1+theta_2);  % xcord of the middle of the link2
        Y2 = y2+L2/1.75*sind(theta_1+theta_2);  % ycord of the middle of the link2

        link_1 = polyshape(X1+link_1_updated(:,1),Y1+link_1_updated(:,2)); %form a complete link1 taking three points to the poly shpace to deal with overlap. 
        link_2 = polyshape(X2+link_2_updated(:,1),Y2+link_2_updated(:,2)); %form a complete link2 taking three points to the poly shpace to deal with overlap.

        set(link_1_arm,'xdata',X1+link_1_updated(:,1),'ydata',Y1+link_1_updated(:,2));

        set(link_2_arm,'xdata',X2+link_2_updated(:,1),'ydata',Y2+link_2_updated(:,2));

        ob_1 = polyshape(OBS_1(:,1),OBS_1(:,2));
        ob_2 = polyshape(OBS_2(:,1),OBS_2(:,2));    %%Plot a 2D Obstacles as command goes polyshape(x,y).
        ob_3 = polyshape(OBS_3(:,1),OBS_3(:,2));




        %%Now we the location of the end effector;

        subplot(1,3,2)
        title('$\textbf{Tracing the End-Effector}$','Interpreter','latex','FontSize',10);
        xlabel('$\textit{\textbf{x-axis}}$','Interpreter','latex','FontSize',10);
        ylabel('$\textit{\textbf{y-axis}}$','Interpreter','latex','FontSize',10);

        anime = animatedline('Marker','.','Color','blue');
        addpoints(anime,X2+0.5*cosd(theta_1+theta_2),Y2+0.5*sind(theta_1+theta_2));
        drawnow limitrate
        grid on 

        axis([-5 5 -5 5])
        daspect([1 1 1])

        %checking if the collision with the obstacle.
        %
        flag = 0 ;
        if (overlaps(link_1,ob_1)||overlaps(link_2,ob_1)) %%Collision with the obstacle 1
            flag = 1;
        elseif (overlaps(link_1,ob_2)||overlaps(link_2,ob_2)) %%Collision with the obstacle 2
            flag = 2;
        elseif (overlaps(link_1,ob_3)||overlaps(link_2,ob_3))  %%Collision with the obstacle 3
            flag = 3;
        elseif (overlaps(link_2,link_1))
            flag = 4;
        end

        % Now let's plot the C-space
        subplot(1,3,3)
        title('$\textbf{Configuration Space}$','Interpreter','latex','FontSize',10);
        xlabel('$\textit{\textbf{Theta 1}}$','Interpreter','latex','FontSize',10);
        ylabel('$\textit{\textbf{Theta 2}}$','Interpreter','latex','FontSize',10);

        x = theta_1-th1+1;    %%record the theta1 if collides with obstacle.
        y = theta_2-th2+1;    %%record the theta2 if collides with obstacle.

        hold on 
        if flag ==1
            plot(x,y,'bx')
        elseif flag == 2
            plot(x,y,'gx')
        elseif flag == 3
            plot(x,y,'magentax')
        elseif flag == 4
            plot(x,y,'blackx')
        else 
            plot(x,y)
        end

        axis([0 360 0 360])
        daspect([1 1 1])
        drawnow limitrate


       
    end
end





