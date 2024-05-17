% how to animate a plot

x = 1:0.001:10;
% Defined an equation 
y = x.^2;

%plot(x,y)
% Animate function 
% eg documented animated line 
% numpoints = 100000; 
%     x = linspace(0,4*pi,numpoints); 
%     y = sin(x); 
%   
%     figure 
%     h = animatedline; 
%     axis([0,4*pi,-1,1]) 
%   
%     for k = 1:numpoints 
%       addpoints(h,x(k),y(k)) 
%       drawnow update 
%     end 
h = animatedline;
%define a loop

for i = 1:length(x)
    addpoints(h,x(i),y(i)) %adds points to the animated lines after each itretion it adds points 
    drawnow; %updates the figures \
    %pause(0.5); % to make a lot more slower we use this .
   
end







