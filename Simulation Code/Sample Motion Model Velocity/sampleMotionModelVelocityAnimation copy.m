% sampleMotionModelVelocity.m
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


function [state] = sampleMotionModelVelocityAnimation()
% sampleMotionModelVelocity computes p(x_end | u, x_start) based on velocity 
% information

    %Initalisation
    clc
    close all
    ticks = 1000;
    count = 50;
    for n = 1:count
        state(1,n,:) = [1,1,pi];
    end
    dt = 0.05;
    
    
    %Initalise random wheel velocity arrays
    %CTRNN to produce random weights
    noOfNodes = 100; %Number of nodes
    W = 0.5 * randn(noOfNodes); %Weights
    Nodes = ones(noOfNodes, ticks); %Nodes
    
    for t = 1:ticks
        Nodes(:,t + 1) = Nodes(:,t) + dt * (-Nodes(:,t) + tanh(W * Nodes(:,t))); %Update Nodes
    end

    u(1,:) = abs(Nodes(1,:)); %Assign translational velocity
    u(2,:) =  Nodes(2,:);% zeros(ticks+1,1);%Assign rotational velocity

    for t = 2:ticks
        %Run Exact Motion
        state(t,1,:) = runSampleExactMotionModelVelocity( u(:,t-1), state(t-1,1,:), dt );

        %Run Nosiy Motion
        for n = 2:count
            state(t,n,:) = runSampleMotionModelVelocity(u(:,t-1), state(t-1,n,:), dt );
        end
    end
    
    %Plot
    figure()
    axis([min(min(state(:,:,1)))-1,max(max(state(:,:,1)))+1,min(min(state(:,:,2)))-1,max(max(state(:,:,2)))+1]); %Limit axis
    hold on
    for t = 2:ticks
        
        %Plot noisy motion
        for n = 2:count
            plot(state(t,n,1),state(t,n,2),'r');
            
        end
        
        %Plot exact motion
        plot(state(1:t,1,1),state(1:t,1,2));
        pause(0.1);
    end
    hold off
    
end



function [ x_end ] = runSampleExactMotionModelVelocity( u, x_start, dt )
%TO DO
    
    %Initalisation
    x1 = x_start(1);
    y1 = x_start(2);
    theta1 = x_start(3);

    v = u(1);
    w = u(2);
    
    %Generate new pose
    x2 = x1 - v/w * sin(theta1) + v/w * sin(theta1 + w*dt);
    y2 = y1 + v/w * cos(theta1) - v/w * cos(theta1 + w*dt);
    theta2 = theta1 + w*dt;
    
    x_end = [x2,y2,theta2];
end

function [ x_end ] = runSampleMotionModelVelocity( u, x_start, dt )
% TO DO

    %Initalisation
        %x_start = initial pose
    x1 = x_start(1);
    y1 = x_start(2);
    theta1 = x_start(3);

        %u = control
    v = u(1);
    w = u(2);

    a = [0.1,0.1,0.1,0.1,0.1,0.1];


    %Genrate noisy velocities
    v_noisy = v + sample(a(1)*(v^2) + a(2)*(w^2));
    w_noisy = w + sample(a(3)*(v^2) + a(4)*(w^2));
    gamma_noisy = sample(a(5)*(v^2) + a(6)*(w^2));

    %Generate new pose
    x2 = x1 - v_noisy/w_noisy * sin(theta1) + v_noisy/w_noisy * sin(theta1 + w_noisy*dt);
    y2 = y1 + v_noisy/w_noisy * cos(theta1) - v_noisy/w_noisy * cos(theta1 + w_noisy*dt);
    theta2 = theta1 + w_noisy*dt + gamma_noisy*dt;

    x_end = [x2,y2,theta2];
end


function [samp] = sample( b )
    randoms = (2*b).*rand(1,12) -b;
    samp = 1/2 * sum(randoms);
end

