% sampleMotionModelVelocity.m
% Probabilistic Robotics Project
%
% Sample Motion Model Velocity - TO DO
% 23/10/14
%
% Version 1 - 23/10/14
%   - Implemented basic algorithm from book
% Version 2 - 30/10/14
%   - Implemented non-noisy agent
% Francis Poole
%
% Code has been adapted from: Probabilistic Robotics by Sebastian Thrun


function [state] = sampleMotionModelVelocity()
% sampleMotionModelVelocity computes p(x_end | u, x_start) based on velocity 
% information

    %Initalisation
    clc
    
    count = 1000;
    u = [1,0];
    x_start = [1,1,pi];
    dt = 0.1;

    %Run Exact Motion
    state(1,:) = runSampleExactMotionModelVelocity( u, x_start, dt );

    %Run Nosiy Motion
    for n = 2:count
        state(n,:) = runSampleMotionModelVelocity( u, x_start, dt );
    end

    %Plot
    plot([x_start(1),state(1,1)],[x_start(2),state(1,2)],'o-')
    hold on;
%     for n = 2:count
%         plot(state(n,1),state(n,2),'b');
%     end
    hold off;
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

    a = [0.01,0.01,0.01,0.01,0.01,0.01];


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

