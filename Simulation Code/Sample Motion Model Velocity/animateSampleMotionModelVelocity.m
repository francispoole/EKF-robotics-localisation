% localisation_V1.m
% Probabilistic Robotics Project
%
% Localisation - Agent localisation using odemetry and light
% sensor
% 10/10/14
%
% Version 2 - 13/10/14
% Changes:
%   - Added odometry localisation
% Francis Poole

clc
clear all;
close all;

%Initialisation
dt = 0.1; %Time step
time = 500; %Total run time
T = [1:dt:time]; %Time array
noise = 0.01; %Amount of noise

randAngle = rand*360;

    %Setup agent
X(1,1) = 0; %Starting X coord
Y(1,1) = 0; %Starting Y coord
Theta(1,1) = randAngle; %Starting angle
    %Setup odometry localisation
X(2,1) = 0; %Starting X coord
Y(2,1) = 0; %Starting Y coord
Theta(2,1) = randAngle; %Starting angle

r = 1; %Radius

velR(1) = 0; %Starting right wheel velocty (rest)
velL(1) = 0; %Starting left wheel velocity (rest)

%Initalise random wheel velocity arrays
    %CTRNN to produce random weights
noOfNodes = 100; %Number of nodes
W = 0.5 * randn(noOfNodes); %Weights
Nodes = ones(noOfNodes, length(T)); %Nodes

for t = 1:length(T) - 1
    Nodes(:,t + 1) = Nodes(:,t) + dt * (-Nodes(:,t) + tanh(W * Nodes(:,t))); %Update Nodes
end

velR = abs(Nodes(1,:)); %Assign right wheel velocities
velL = abs(Nodes(2,:)); %Assign left wheel velocities
 
%Simulate
for t = 1:length(T) - 1
    velAvg = (velR(t) + velL(t)) / 2; %Calculate average velocity of wheels
    
    %Noisy agent
    X(1,t+1) = X(1,t) + dt * (velAvg * cos(Theta(1,t)) + rand*noise); %Update X coord
    Y(1,t+1) = Y(1,t) + dt * (velAvg * sin(Theta(1,t)) + rand*noise); %Update Y coord
    Theta(1,t+1) = Theta(1,t) + dt * ((velR(t) - velL(t)) / (2 * r) + ((2*rand-1)*noise)); %Update angle
    
    %Odometry localisation
    X(2,t+1) = X(2,t) + dt * (velAvg * cos(Theta(2,t))); %Update X coord
    Y(2,t+1) = Y(2,t) + dt * (velAvg * sin(Theta(2,t))); %Update Y coord
    Theta(2,t+1) = Theta(2,t) + dt * ((velR(t) - velL(t)) / (2 * r)); %Update angle
end

%Plot Animation
figure()
for t = 1:length(T) - 1
    clf; %Clear last frame
    subplot(2,1,1);
    plotCircle(X(1,t),Y(1,t),r,'k'); %Plot noisy agent
    hold on;
    plotCircle(X(2,t),Y(2,t),r,'r'); %Plot odometry localisation
    plot([X(1,t), X(1,t) + r * cos(Theta(1,t))], [Y(1,t), Y(1,t) + r * sin(Theta(1,t))], 'k'); %Plot noisy agent angle line
    plot([X(2,t), X(2,t) + r * cos(Theta(2,t))], [Y(2,t), Y(2,t) + r * sin(Theta(2,t))], 'r'); %Plot odometry localisation angle line
    plot(X(1,1:t),Y(1,1:t),'k'); %Plot noisy agent trace
    plot(X(2,1:t),Y(2,1:t),'r'); %Plot odometry localisation trace
    plot([X(1,t),X(2,t)],[Y(1,t),Y(2,t)]);
    text(7.5,9,num2str(t)) %Time text
    hold off;
    axis([min([min(X),min(Y)]),max([max(X),max(Y)]),min([min(X),min(Y)]),max([max(X),max(Y)])]); %Limit axis
    
    subplot(2,1,2);
    plot(T(1:t),sqrt((X(1,1:t)-X(2,1:t)).^2 + (Y(1,1:t)-Y(2,1:t)).^2)); %Error
    axis([0,time,0,max(sqrt((X(1,:)-X(2,:)).^2 + (Y(1,:)-Y(2,:)).^2))]);
    pause(0.001); %Wait to update frame
    
end

