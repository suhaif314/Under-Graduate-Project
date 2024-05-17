%%%% ME766 - ASSIGNMENT 1 %%%%
%%% N MOHAMMED SOHAIB  200611

%%% Generating C-space for 2R, 2 DOF serial arm planer mechanism %%%%

clc;
clear;
close all;

% Define origin of link
x1 = 0;
y1 = 0;

% Corresponding Link lengths 
L1 = 3;
L2 = 2;

% Corressponding link thickness, here we consider a verysmall thickness, If
% need can be made big to like a real link.
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


%% Shape the obstacles and the links.

%Links
%Link1 given some shape to it.
Link_1_s = [-L1/2, -T1/2;
    L1/2, -T1/2;
    L1/2, T1/2;
    -L1/2, T1/2];

%Link 2 given some shape to it.
Link_2_s =[-L2/2, -T2/2;
    L2/2, -T2/2;
    L2/2, T2/2;
    -L2/2, T2/2];

% OBSTACLES.

%RECTANGLE
OBS_1 = [1,1;
    1.5,1;
    1.5,1.5;
    1,1.5];

% TRIANGLE
OBS_2 = .85.*[1,-1;
    0,-2;
    2,-2];

% CRICLE
th_c = linspace(0,360,100);
xcircle = -1 + 0.5*cosd(th_c);
ycircle = 1+0.5*sind(th_c);
OBS_3 = [xcircle,ycircle];


%% Plot the OBSTACLES AND LINKS.

fill(OBS_1(:,1),OBS_1(:,2),'blue');
fill(OBS_2(:,1),OBS_2(:,2),'green');
fill(xcircle,ycircle,'magenta');


link_1_arm = fill(Link_1_s(:,1),Link_1_s(:,2),'red');                      %Made a definite link 1
link_2_arm = fill(Link_2_s(:,1),Link_2_s(:,2),'cyan');                     %Made a definite link 2
 

%% NOW WE GENEREATE THE REQUIRED PLOTS -C SPACE AND LOCUS OF END EFFECTOR.

th1 = 0 ;                                                                  %defining intial config of the link 1 
th2 = 0 ;                                                                  %definng intial config of the link 2

for theta_1 = th1:5:360+th1                                                %for the rotation of the link 1
    for theta_2 = th2:5:360+th2                                            %for the rotation of the link2
        
        %we will keep updating the geometrical shape of links, which will
        %be reflected in the Euclidean space.
        
        link_1_updated = Link_1_s*[cosd(theta_1),sind(theta_1);
                                   -sind(theta_1),cosd(theta_1)];                  %transforming the corresponding link so that it moves like a revolute joint.
        link_2_updated = Link_2_s*[cosd(theta_2+theta_1),sind(theta_1+theta_2);
                                   -sind(theta_2+theta_1),cosd(theta_1+theta_2)];  

        % joinng the link 1 with the origin so as to rotate.
        X1 = x1+L1/2*cosd(theta_1);                                        
        Y1 = y1 +L1/2*sind(theta_1);                                       

        x2 = x1+L1*cosd(theta_1);                                          % xcord of the end effector/link2
        y2 = y1+L1*sind(theta_1);                                          %ycord of the end effector/link2
 
        % joining the link 2 to link 1 so as to rotate about it.
        % Here we can change the distance between the links by changing the
        % parameters of L/2 or anything to find the obstacle intrection
        % with it.

        X2 = x2+L2/2*cosd(theta_1+theta_2);                                
        Y2 = y2+L2/2*sind(theta_1+theta_2);                                

        link_1 = polyshape(X1+link_1_updated(:,1),Y1+link_1_updated(:,2)); %make a polygon link1 
        link_2 = polyshape(X2+link_2_updated(:,1),Y2+link_2_updated(:,2)); %make a polygon link2

        set(link_1_arm,'xdata',X1+link_1_updated(:,1),'ydata',Y1+link_1_updated(:,2)); 

        set(link_2_arm,'xdata',X2+link_2_updated(:,1),'ydata',Y2+link_2_updated(:,2));

        ob_1 = polyshape(OBS_1(:,1),OBS_1(:,2));
        ob_2 = polyshape(OBS_2(:,1),OBS_2(:,2));                           %Plot a 2D Obstacles as command goes polyshape(x,y).
        ob_3 = polyshape(xcircle,ycircle);




        %Now we plot the location of the end effector;
        %This plot make look like torus, with obstacles in it.

        subplot(1,3,2)
        title('$\textbf{Tracing the End-Effector}$','Interpreter','latex','FontSize',10);
        xlabel('$\textit{\textbf{x-axis}}$','Interpreter','latex','FontSize',10);
        ylabel('$\textit{\textbf{y-axis}}$','Interpreter','latex','FontSize',10);

        anime1 = animatedline('Marker','.','Color','blue');
        anime2 = animatedline('Marker','.','Color','green');
        anime3 = animatedline('Marker','.','Color','magenta');
        anime4 = animatedline('Marker','.','Color','black');

        if (overlaps(link_1,ob_1)||overlaps(link_2,ob_1))                  %collision with the obstacle 1
            addpoints(anime1,X2+0.5*cosd(theta_1+theta_2),Y2+0.5*sind(theta_1+theta_2));
        elseif (overlaps(link_1,ob_2)||overlaps(link_2,ob_2))              %Collision with the obstacle 2
            addpoints(anime2,X2+0.5*cosd(theta_1+theta_2),Y2+0.5*sind(theta_1+theta_2));
        elseif (overlaps(link_1,ob_3)||overlaps(link_2,ob_3))              %Collision with the obstacle 3
            addpoints(anime3,X2+0.5*cosd(theta_1+theta_2),Y2+0.5*sind(theta_1+theta_2));
        else
            addpoints(anime4,X2+0.5*cosd(theta_1+theta_2),Y2+0.5*sind(theta_1+theta_2));
        end
       

        drawnow limitrate
        grid on 

        axis([-5 7 -5 5])
        daspect([1 1 1])

        %checking if the collision with the obstacle.
        flag = 0 ;
        if (overlaps(link_1,ob_1)||overlaps(link_2,ob_1))                  %%Collision with the obstacle 1
            flag = 1;
        elseif (overlaps(link_1,ob_2)||overlaps(link_2,ob_2))              %%Collision with the obstacle 2
            flag = 2;
        elseif (overlaps(link_1,ob_3)||overlaps(link_2,ob_3))              %%Collision with the obstacle 3
            flag = 3;
        %elseif (overlaps(link_2,link_1))                                   % Considered only if the link thickness taken into the consideration.
            %flag = 4;

        end

        % If the thickness is more here we also consider the collision with
        % the link1 and the link 2, here in this case I have considered
        % small thickness.

        % Now let's plot the C-space
        subplot(1,3,3)
        title('$\textbf{Configuration Space}$','Interpreter','latex','FontSize',10);
        xlabel('$\textit{\textbf{Theta 1}}$','Interpreter','latex','FontSize',10);
        ylabel('$\textit{\textbf{Theta 2}}$','Interpreter','latex','FontSize',10);

        xc = theta_1-th1+1;                                                %%record the theta1 if collides with obstacle.
        yc = theta_2-th2+1;                                                %%record the theta2 if collides with obstacle.

        hold on 
        if flag ==1
            plot(xc,yc,'b.')
        elseif flag == 2
            plot(xc,yc,'g.')
        elseif flag == 3
            plot(xc,yc,'magenta.')
        %elseif flag ==4 
            %plot(xc,yc,'black.')
        else 
            plot(xc,yc)
        end

        axis([0 360 0 360])
        daspect([1 1 1])
        drawnow limitrate


       
    end
end





