% noisyAgentSimulation.m
% Author: Francis Poole
% Date: 10/10/14
% Probabilistic Robotics Project
%
% Noisy Agent Simulation - Simple differential wheeled robot motion in
% simulated noisy environment


function noisyAgentSimulation
    %Setup
    clc
    clear all;
    close all;

    %Simulation initialisation
    dt = 0.1; %Time step
    time = 100; %Total run time
    %T = [1:dt:time]; %Time array
    noise = 0; %Amount of noise

    %Robot initialisation
    X(1) = 0; %Starting x coord
    Y(1) = 0; %Starting y coord
    Theta(1) = 0; %Starting angle

    r = 1; %Radius of robot's wheels
    L = 2; %Distance between robot's wheels

    %Initialise random wheel velocity arrays
        %CTRNN to produce random weights
    noOfNodes = 100; %Number of nodes
    W = 0.5 * randn(noOfNodes); %Weights
    Nodes = ones(noOfNodes, time/dt); %Nodes

    for t = 1:time/dt
        Nodes(:,t + 1) = Nodes(:,t) + dt * (-Nodes(:,t) + tanh(W * Nodes(:,t))); %Update Nodes
    end

    velR = abs(Nodes(1,:)); %Assign right wheel velocities
    velL = abs(Nodes(2,:)); %Assign left wheel velocities

    %Simulate
    for t = 1:time/dt
        velAvg = (velR(t) + velL(t)) / 2; %Calculate average velocity of wheels

        %Noisy agent
        X(t+1) = X(t) + dt * ((r / 2) * (velR(t) + velL(t)) * cos(Theta(t)) + sample(noise)); %Update X coord
        Y(t+1) = Y(t) + dt * ((r / 2) * (velR(t) + velL(t)) * sin(Theta(t)) + sample(noise)); %Update Y coord
        Theta(t+1) = Theta(t) + dt * ((r / L) * (velR(t) - velL(t)) + sample(noise)); %Update angle

    end

    %Plot Animation
    figure
    plotCircle(X(end),Y(end),(L/2),'r'); %Plot noisy agent
    hold on;
    plot([X(end), X(end) + (L/2) * cos(Theta(end))], [Y(end), Y(end) + (L/2) * sin(Theta(end))], 'r'); %Plot noisy agent angle line
    plot(X(:),Y(:),'r'); %Plot noisy agent trace
    plot([X(end),X(end)],[Y(end),Y(end)],'b');
    hold off;
end


function [samp] = sample( b )
    randoms = (2*b).*rand(1,12) -b;
    samp = 1/2 * sum(randoms);
end
