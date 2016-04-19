%Bar chart



function barChart3d
    %Setup
    clc
    clear all;
    close all;

    %Simulation initialisation
    dt = 0.1; %Time step
    time = 0.1; %Total run time
    %T = [1:dt:time]; %Time array
    noise = 0.1; %Amount of noise
    robots = 1000;

    %Robot initialisation
    X(1,:) = zeros(1,robots); %Starting x coord
    Y(1,:) = zeros(1,robots); %Starting y coord
    Theta(1,:) = zeros(1,robots) %Starting angle

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
        for n = 1:robots
            

            %Noisy agent
            X(t+1,n) = X(t,n) + dt * ((r / 2) * (velR(t) + velL(t)) * cos(Theta(t,n)) + sample(noise)); %Update X coord
            Y(t+1,n) = Y(t,n) + dt * ((r / 2) * (velR(t) + velL(t)) * sin(Theta(t,n)) + sample(noise)); %Update Y coord
            Theta(t+1,n) = Theta(t,n) + dt * ((r / L) * (velR(t) - velL(t)) + sample(noise)); %Update angle
        end
    end

    
    %Plot Animation
    figure
    hold on;
    for n = 1:robots
        %plotCircle(X(end,n),Y(end,n),(L/2),'r'); %Plot noisy agent
        
        %plot([X(end,n), X(end,n) + (L/2) * cos(Theta(end,n))], [Y(end,n), Y(end,n) + (L/2) * sin(Theta(end,n))], 'r'); %Plot noisy agent angle line
        %plot(X(:,n),Y(:,n),'r'); %Plot noisy agent trace
        plot([X(end,n),X(end,n)],[Y(end,n),Y(end,n)],'bo');
        
    end
    hold off;
end


function [samp] = sample( b )
    randoms = (2*b).*rand(1,12) -b;
    samp = 1/2 * sum(randoms);
end
