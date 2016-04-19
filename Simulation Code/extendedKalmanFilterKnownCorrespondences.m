%extendedKalmanFilterKnowCorrespondences.m
function extendedKalmanFilterKnownCorrespondences()
    clc
    clear all
    close all
    
%---Initalise world
    t = 1; %Tick
    dt = 0.1; %Change in time per tick
    time = 100/dt; %Simulation time

    %Map features (Add more features!)
    m = [1;  %Property
         1;  %X position
         1]; %Y position
    
%---Agent setup
    r = 1; %Agent wheel radius
    D = 1; %Distance between agent's wheels

    %---Body
    %Pose
    x = [0;  %Initial x position
         0;  %Initial y position
         1]; %Initial orientation
    
    x2 = [0;  %Initial x position
        0;  %Initial y position
        1]; %Initial orientation
    
    %Motion control
%     velL=[0,0.1]
%     velR=[0,0.2]
    [velL, velR] = getRandomWheelVel(time, dt); %Initial smooth random wheel velocities
    
    %---Brain
    %Pose
    mu = [x(1);  %Initial mean x position
          x(2);  %Initial mean y position
          x(3)]; %Initial mean orientation
    
    Sigma = [0.1,0,0;
             0,0.1,0;
             0,0,0.1]; %Initial pose covariance matrix (Must be symmetric)

    %Motion control
    u(1,:) = (velR + velL) ./ 2 .* r;  %Calculate translational velocity
    u(2,:) =  (velR - velL) ./ D .* r; %Calculate rotational velocity
    
    %Measurement (need to add noise)
    z = [sqrt((m(2)-x(1))^2 + (m(3)-x(2))^2); %Initial range
            atan2(m(3)-x(2), m(2)-x(1)) - x(3);  %Initial bearing
            1];                                  %Initial signature
    
    %Correspondences (need to do for multiple features)
    c = 1;
      
%---Simulate

    figure(1)
    hold on
    plot(m(1),m(2),'O');
    for t = (2:time)
        %Body update
        x(1,t) = x(1,t-1) + dt * ((r / 2) * (velR(t) + velL(t)) * cos(x(3,t-1)) + normrnd(0,0.1)); %Update X coord
        x(2,t) = x(2,t-1) + dt * ((r / 2) * (velR(t) + velL(t)) * sin(x(3,t-1)) + normrnd(0,0.1)); %Update Y coord
        x(3,t) = x(3,t-1) + dt * ((r / D) * (velR(t) - velL(t)) + normrnd(0,0.1)); %Update angle
                
        %Measurement update
        z(:,t) = [sqrt((m(2)-x(1,t))^2 + (m(3)-x(2,t))^2);    %Range
                  atan2(m(3)-x(2,t), m(2)-x(1,t)) - x(3,t); %Bearing
                  1];                                       %Signature
        
        
        
        
        %Brain update
        [mu(:,t), Sigma(:,:,t), pZ(t)] = runExtendedKalmanFilterKnownCorrespondences(mu(:,t-1),Sigma(:,:,t-1),u(:,t),z(:,t),c,m);
        
        zHat = [sqrt((m(2)-mu(1,t))^2 + (m(3) - mu(2,t))^2);
                atan2(m(3)-mu(2,t), m(2)-mu(1,t)) - mu(3,t); 
                m(1)]; %Calculated measurement from estimated pose to feature
        
        disp('Body pose = ');
        disp(x(:,t))
        z(:,t)
        disp('Brain updated pose = ');
        disp(mu(:,t))
        zHat(:) 
        disp('-------');
            
        
        
        %Brain simple update (without EKF)
        x2(1,t) = x2(1,t-1) + dt * ((r / 2) * (velR(t) + velL(t)) * cos(x2(3,t-1))); %Update X coord
        x2(2,t) = x2(2,t-1) + dt * ((r / 2) * (velR(t) + velL(t)) * sin(x2(3,t-1))); %Update Y coord
        x2(3,t) = x2(3,t-1) + dt * ((r / D) * (velR(t) - velL(t))); %Update angle
        
%         disp('No EKF pose = ');
%         disp(x2(:,t))
%         disp('---------------------');
        
        plot([x(1,t-1),x(1,t)],[x(2,t-1),x(2,t)]);
        plot([x2(1,t-1),x2(1,t)],[x2(2,t-1),x2(2,t)],'r');
        plot([mu(1,t-1),mu(1,t)],[mu(2,t-1),mu(2,t)],'g');
        pause(0.001); %Wait to update frame
    end
    
    hold off;
    
%---Plot pdf
%     [X1,X2] = meshgrid(linspace(-1,1,250)', linspace(-1,1,250)');
%     XM = [X1(:) X2(:)];
%     p1 = mvnpdf(XM, mu(1:2,1).',Sigma(1:2,1:2,1));
%     p2 = mvnpdf(XM, mu(1:2,2).',Sigma(1:2,1:2,2));
%     hold on;
%     %mesh(X1,X2,reshape(p1,250,250));
%     mesh(X1,X2,reshape(p2,250,250));
%     plot3([X(2),X(2)],[Y(2),Y(2)],[0,2]);
%     hold off;


    
end

function [mu, Sigma, p] = runExtendedKalmanFilterKnownCorrespondences(mu,Sigma,u,z,c,m) 
    
    %Initalise variables
    dt = 0.1;
    a = [1,1,1,1];
    
    %Extract variables
        %Pose mean
    theta = mu(3); %Get gaussian distribution mean of bearing (most likely bearing)
    
        %Motion Control
    v = u(1);
    w = u(2);
    
    %Prediction step
        %Jacobian derivatives
    G = [1, 0, (v/w)*(-cos(theta) + cos(theta + w*dt));...
         0, 1, (v/w)*(-sin(theta) + sin(theta + w*dt));...
         0, 0, 1];
     
    V = [(-sin(theta) + sin(theta + w*dt))/w, (v*(sin(theta) - sin(theta + w*dt)))/w^2 + (v*(cos(theta + w*dt)*dt))/w;...
         (cos(theta) - cos(theta + w*dt))/w, -(v*(cos(theta) - cos(theta + w*dt)))/w^2 + (v*(sin(theta + w*dt)*dt))/w;...
         0, dt];
    
        %Noise in control space
    M = [a(1)*v^2 + a(2)*w^2, 0;...
         0, a(3)*v^2 + a(4)*w^2];
    
        %Motion Updates
            %Mean motion update
    muBar = mu + [-(v/w)*sin(theta) + (v/w)*sin(theta + w*dt);...
                     ((v/w)*cos(theta)) - ((v/w)*cos(theta + w*dt));...
                     w*dt];
                   
            %Sigma motion update
    SigmaBar = (G*Sigma)*(G.')+(V*M)*(V.');
    
%     disp('Brain predicted pose = ');
%     disp(muBar(:));
    %Plot prediction based on odometry
%     [X1,X2] = meshgrid(linspace(-1,1,250)', linspace(-1,1,250)');
%     XM = [X1(:) X2(:)];
%     p2 = mvnpdf(XM, muBar(1:2).',Sigma(1:2,1:2));
%     hold on;
%     mesh(X1,X2,reshape(p2,250,250));
%     hold off;
%---Measurement Update
    Q = [0.000001,0,0;
         0,0.000001,0;
         0,0,0.000001]; %Measurement noise matrix (how much the measurement affects the equation
   
    %For each feature
    j = 1; %Corresponding feature
    
    q = (m(2)-muBar(1))^2 + (m(3) - muBar(2))^2; %squared distance between estimated pose and feature
    
    zHat = [sqrt(q);
            atan2(m(3)-muBar(2), m(2)-muBar(1)) - muBar(3); 
            m(1)]; %Calculated measurement from estimated pose to feature
        
    H = [-((m(2)-muBar(1))/sqrt(q)), -((m(3)-muBar(2))/sqrt(q)), 0;
         (m(3)-muBar(2))/q, -((m(2)-muBar(1))/q), -1;
         0, 0, 0]; %Jacobian stuff?
    
    S = H*SigmaBar*(H.') + Q; %
    
    K = (SigmaBar*(H.'))/(S); %Kalman gain matrix

    muBar = muBar + K*(z-zHat);
    
    SigmaBar = ([1, 0, 0;
                 0, 1, 0;
                 0, 0, 1] - K*H)*SigmaBar;
    %Loop back
    mu = muBar;
    Sigma = SigmaBar;
    p = 0;
    
end

function [velL, velR] = getRandomWheelVel(time, dt)
    %Initialise random wheel velocity arrays using a CTRNN to produce
    %smooth random values
    noOfNodes = 100; %Number of nodes
    W = 0.5 * randn(noOfNodes); %Weights
    Nodes = ones(noOfNodes, time/dt); %Nodes

    for t = 1:time/dt
        Nodes(:,t + 1) = Nodes(:,t) + dt * (-Nodes(:,t) + tanh(W * Nodes(:,t))); %Update Nodes
    end
    
    velL = abs(Nodes(1,:)); %Assign left wheel velocities
    velR = abs(Nodes(2,:)); %Assign right wheel velocities
end