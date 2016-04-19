% combinedMotionModelAndSimulation.m
% Probabilistic Robotics Project
%
% Sample Motion Model Velocity - TO DO
% 23/10/14
%
% Version 1 - 23/10/14
%   - Implemented basic algorithm from book
% Version 2 - 30/10/14
%   - Implemented non-noisy (exact) motion
% Francis Poole
%
% Code has been adapted from: Probabilistic Robotics by Sebastian Thrun


function sampleMotionModelVelocityAnimation
% sampleMotionModelVelocity computes p(x_end | u, x_start) based on velocity 
% information

    %Initalisation
    clc
    clear all;
    close all
    
    dt = 0.1; 
    %T = [1:dt:time]; %Time array
    
    noiseA = 0.1; %Simulation noise
    
    noiseB = 0.1; %Brain noise
    a = ones(6,1) * noiseB; %Motion model noise
    count = 50; %Number of noisy simulations
    
    r = 1; %Radius of robot's wheels
    L = 1; %Distance between robot's wheels
    
    %Model initialisation
    for n = 1:count
        state(1,n,:) = [1,1,pi/2];
    end
    
    %Robot initialisation
    X = ones(1,count); %Starting x coord
    Y = ones(1,count); %Starting y coord
    Theta = ones(1,count) * (2*pi); %Starting angle

    velR = 0.77; %Assign right wheel velocities
    velL = 0.85; %Assign left wheel velocities
    
    

    %Initalise brain
    u(1) = (velR + velL) ./ 2 .* r %Assign translational velocity
    u(2) =  (velR - velL) ./ L .* r %Assign rotational velocity --- 
    x_start = [1,1,2*pi]; %Start pose

    
    
    %Run Simulation
    for n = 1:count
        %Noisy agent
        X(n) = X(n) + dt * ((r / 2) * (velR + velL) * cos(Theta(n)) + sample(noiseA)); %Update X coord
        Y(n) = Y(n) + dt * ((r / 2) * (velR + velL) * sin(Theta(n)) + sample(noiseA)); %Update Y coord
        Theta(n) = Theta(n) + dt * ((r / L) * (velR - velL) + sample(noiseA)); %Update angle
    end
    
    
    max(X)-min(X)
    
    max(Y)-min(Y)
    
    max(Theta)-min(Theta)
    
    %Run Brain
    x = linspace(min(X),max(X),50)
    y = linspace(min(Y),max(Y),50)
    theta = linspace(min(Theta),max(Theta),50)
    for xN = 1:50
        clc
        strcat(num2str(xN),'/',num2str(50))
        
        for yN = 1:50
            
            for thetaN = 1:50
                probabilityTheta(thetaN+1) = runMotionModelVelocity([x(xN), y(yN), theta(thetaN)], u, x_start);
            end
            probability(xN,yN) = sum(probabilityTheta(thetaN+1));
        end
    end
    figure()
    surf(round(probability*10^8))
    %Plot
    
    figure()
    for n = 1:count
        hold on
        plot(X(n),Y(n),'bO'); %Plot robot
        hold off
    end
    
end

function [probability] = runMotionModelVelocity(x_end, u, x_start)

%Initalisation
    %Start phase
    x1 = x_start(1); %x location
    y1 = x_start(2); %y location
    theta1 = x_start(3); %Bearing
    
    %End phase
    x2 = x_end(1); %x location
    y2 = x_end(2); %y location
    theta2 = x_end(3); %Bearing
    
    %Control
    v = u(1); %Translational velocity
    w = u(2); %Rotational velocity
    
    a = [2,2,2,2,0.1,0.1]; %Error terms
    
    dt = 0.1; %Time step 

%Algorithm
    %Turning circle


    
    mu = 1/2 * (((x1 - x2)*cos(theta1) + (y1-y2)*sin(theta1)) / ...
                ((y1 - y2)*cos(theta1) - (x1-x2)*sin(theta1))); %ERROR! when 0 causes NaN
            
    x_star = (x1 + x2)/2 + mu*(y1-y2); %x location of centre of turning circle
    y_star = (y1 + y2)/2 + mu*(x1-x2); %y location of centre of turning circle
    r_star = sqrt((x1-x_star)^2 + (y1-y_star)^2); %Radius of turning circle
    
    %Change of heading direction
    dtheta = atan2(y2-y_star, x2-x_star) - atan2(y1-y_star, x1-x_star);
    
    %Noisy control
    v_hat = dtheta/dt * r_star; %Noisy translational velocity
    w_hat = dtheta/dt; %Noisy rotational velocity
    gamma_hat = (theta2-theta1)/dt - w_hat; %Final rotation from noise
    
    %Probabilities
    prob_v = prob(v-v_hat, a(1)*(v^2) + a(2)*(w^2));
    prob_w = prob(w-w_hat, a(3)*(v^2) + a(4)*(w^2));
    prob_gamma = prob(gamma_hat, a(5)*(v^2) + a(6)*(w^2));
    
    probability = prob_v * prob_w * prob_gamma;
end



function [ probDist ] = prob( a, b )
%prob 

%Normal distribution
probDist = 1/sqrt(2*pi*(b^2)) * exp(-1/2 * (a^2)/(b^2));
end

function [samp] = sample( b )
    randoms = (2*b).*rand(1,12) -b;
    samp = 1/2 * sum(randoms);
end