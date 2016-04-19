% sampleMotionModelVelocity.m
% Autor: Francis Poole
% Date: 23/10/14
%
% Produces sampled distribution of agents given some motion command
% Based on algorithm provided by Thrun (2006)


function sampleMotionModelVelocity()
% sampleMotionModelVelocity computes p(x_end | u, x_start) based on velocity 
% information

    %Initalisation
    clc
    close all;
    count = 500;
    
    r = 1; %Radius of simulation robot's wheels
    L = 2; %Distance between simulation robot's wheels
    
    velL = 0.6; %Left wheel velocity
    velR = 0.8; %Right wheel velocity
    
    u(1) = (velR + velL) ./ 2 .* r; %Control translational velocity
    u(2) =  (velR - velL) ./ L .* r; %Control rotational velocity
    x_start = [0, 0, 5*pi/4];
    dt = 0.1;
    a = [0.1,0.1,2.5,2.5,0.1,0.1];

    %Run Exact Motion
    state(1,:) = runSampleMotionModelVelocity( u, x_start, dt, [0,0,0,0,0,0]);

    %Run Nosiy Motion
    for n = 2:count
        state(n,:) = runSampleMotionModelVelocity( u, x_start, dt, a );
    end

    min(state(:,3))
    max(state(:,3))
    %Plot
    hold on;
    plot([x_start(1),state(1,1)],[x_start(2),state(1,2)],'o-')
    
    for n = 2:count
        plot(state(n,1),state(n,2),'xb');
    end
    hold off;
end

function [ x_end ] = runSampleMotionModelVelocity( u, x_start, dt, a )
% TO DO

    %Initalisation
        %x_start = initial pose
    x1 = x_start(1);
    y1 = x_start(2);
    theta1 = x_start(3);

        %u = control
    v = u(1);
    w = u(2);

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

