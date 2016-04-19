%extendedKalmanFilterKnowCorrespondences.m
function EKFMotionModel()
    clc
    clear all
    close all
    
    dt = 0.1; %Change in time per tick
    time = 100/dt; %Simulation time
    
    runs = 5;
    
    %Initalise errors
    distErr = zeros(runs, time);
    bearingErr = zeros(runs, time);
    

    %Map of features
    m = [1;  %Property
         -1;  %X position
         1]; %Y position
     
    %Correspondences
    c = [1];
    
%---Agent setup
    r = 1; %Agent wheel radius
    D = 1; %Distance between agent's wheels

%-----Body
    %Pose
    x = [0;  %Initial x position
         0;  %Initial y position
         0]; %Initial orientation
    
%-----Brain
%---Initial Pose Estimation
    %Pose
    mu = [x(1);  %Initial mean x position
          x(2);  %Initial mean y position
          x(3)]; %Initial mean orientation
    
    Sigma = [0.1,0,0;
             0,0.1,0;
             0,0,0.1]; %Initial pose covariance matrix (Must be symmetric)
    
    %---Velocity Motion Model
    %Pose
    mu2 = mu;
    
    Sigma2 = Sigma;
    
    
    for run = 1:runs
        run
        %---Initalise world
        t = 1; %Tick
        
        
        %Motion command
        [velL, velR] = getRandomWheelVel(time, dt); %Initial smooth random wheel velocities

        u(1,:) = (velR + velL) ./ 2 .* r;  %Calculate translational velocity
        u(2,:) =  (velR - velL) ./ D .* r; %Calculate rotational velocity

        %---Initial Sensory Measurement
        for i = 1:size(m,2)
            z(:,i) = [sqrt((m(2,i)-x(1))^2 + (m(3,i)-x(2))^2) + normrnd(0,0.1);                      %Initial range
                      mod(atan2(m(3,i)-x(2), m(2,i)-x(1)) - x(3) + normrnd(0,0.1) + pi, 2*pi) - pi;  %Initial bearing (normalised to between pi and -pi)
                      m(1,i) + normrnd(0,0.1)];                                                      %Initial signature
        end

        %---Simulate
        for t = 2:time
            %Body update
            x(1,t) = x(1,t-1) + dt * ((r / 2) * (velR(t) + velL(t) + normrnd(0,0.5)) * cos(x(3,t-1))); %Update X coord
            x(2,t) = x(2,t-1) + dt * ((r / 2) * (velR(t) + velL(t) + normrnd(0,0.5)) * sin(x(3,t-1))); %Update Y coord
            x(3,t) = x(3,t-1) + dt * ((r / D) * (velR(t) - velL(t) + normrnd(0,0.5)));                 %Update angle

            %Measurement update
            for i = 1:size(m,2)
                z(:,i) = [sqrt((m(2,i)-x(1,t))^2 + (m(3,i)-x(2,t))^2);  %Range
                          mod(atan2(m(3,i)-x(2,t), m(2,i)-x(1,t)) - x(3,t) + normrnd(0,0.1) + pi, 2*pi) - pi; %Bearing
                          m(1,i)];                                      %Signature
            end

            %Brain update
            %[mu(:,t), Sigma(:,:,t), pZ(t)] = runExtendedKalmanFilterKnownCorrespondences(mu(:,t-1),Sigma(:,:,t-1),u(:,t),z,c,m);

            %Brain2 update (without EKF)
            [mu2(:,t), Sigma2(:,:,t)] = runMotionUpdate(mu2(:,t-1),Sigma2(:,:,t-1),u(:,t));


            distErr(run,t) = sqrt((x(1,t)-mu2(1,t))^2 + (x(2,t)-mu2(2,t))^2);
            bearingErr(run,t) = abs(mod(x(3,t)-mu2(3,t) + pi, 2*pi) - pi);
        end
    end
    
    
    %Plot Snapshots of last run
    for n = 1:4
        figure(n)
        a = floor(n*time/4);
        %plot(m(1,:),m(2,:),'O'); %Plot map features
        
        hold on
%         [X1,X2] = meshgrid(linspace(mu2(1,a)-10,mu2(1,a)+10,50)', linspace(mu2(2,a)-10,mu2(2,a)+10,50)');
%         XM = [X1(:) X2(:)];
%         p1 = mvnpdf(XM, mu2(1:2,a).',Sigma2(1:2,1:2,a));
%         mesh(X1,X2,reshape(p1,50,50));
        
        
        plot(x(1,1:a), x(2,1:a),'k'); %Body
        %plot([mu(1,t-1),mu(1,t)],[mu(2,t-1),mu(2,t)],'b'); %Brain
        plot(mu2(1,1:a),mu2(2,1:a),'b'); %Motion model
        
        %Plot distance error
        plot([x(1,a),mu2(1,a)],[x(2,a),mu2(2,a)],'--r');
        
        set(gca,'fontsize',17);
        title('Localisation Over Time - Body vs Motion Model');
        legend('Body','Motion Model','Position Error');
        xlabel('x-position')
        ylabel('y-position')
        
        %Plot agents
        plotCircle(x(1,a), x(2,a),D/2,'k');
        plotCircle(mu2(1,a), mu2(2,a),D/2,'b');
        plot([x(1,a), x(1,a) + (D/2) * cos(x(3,a))], [x(2,a), x(2,a) + (D/2) * sin(x(3,a))], 'k'); %Plot noisy agent angle line
        plot([mu2(1,a), mu2(1,a) + (D/2) * cos(mu2(3,a))], [mu2(2,a), mu2(2,a) + (D/2) * sin(mu2(3,a))], 'b'); %Plot noisy agent angle line
        
        %Axis - Centered
        xMax = max(max(x(1,:)),max(mu2(1,:)));
        xMin = min(min(x(1,:)),min(mu2(1,:)));
        yMax = max(max(x(2,:)),max(mu2(2,:)));
        yMin = min(min(x(2,:)),min(mu2(2,:)));
        
        xRange = xMax - xMin;
        yRange = yMax - yMin;
        if xRange > yRange
            axis([xMin-1, xMax+1, yMin-(xRange+2-yRange)/2, yMax+(xRange+2-yRange)/2]);
        else
            axis([xMin-(yRange+2-xRange)/2, xMax+(yRange+2-xRange)/2, yMin-1, yMax+1]);
        end
        
        hold off
    end
    
    %Plot Distance Error
    figure(5)
    mean(distErr,1)
    plot(mean(distErr,1))
    set(gca,'fontsize',17);
    title('Position Error Over Time');
    xlabel('Time (steps)');
    ylabel('Euclidean Distance Between Models (error)');
    
    %Plot Bearing Error
    figure(6)
    mean(bearingErr,1)
    plot(mean(bearingErr,1))
    set(gca,'fontsize',17);
    title('Bearing Error over Time');
    xlabel('Time (steps)');
    ylabel('Difference in Radians Between Models (error)');
    
    
%---Plot Animation
%     figure(1)
%     for t = 2:time
%         %Pose
% %         subplot(2,2,1);
%         
%         hold on
%         plot(m(1,:),m(2,:),'O'); %Plot map features
%         plot([x(1,t-1),x(1,t)],[x(2,t-1),x(2,t)],'k'); %Body
%         %plot([mu(1,t-1),mu(1,t)],[mu(2,t-1),mu(2,t)],'b'); %Brain
%         plot([mu2(1,t-1),mu2(1,t)],[mu2(2,t-1),mu2(2,t)],'r'); %Measurement model
% 
%         [X1,X2] = meshgrid(linspace(mu(1,t)-0.05,mu(1,t)+0.1,50)', linspace(mu(2,t)-0.05,mu(2,t)+0.1,50)');
%         XM = [X1(:) X2(:)];
%         p1 = mvnpdf(XM, mu(1:2,t).',Sigma(1:2,1:2,t));
%         mesh(X1,X2,reshape(p1,50,50));
%          hold off
%         pause(0.001); %Wait to update frame
%     end
end


function [mu, Sigma] = runMotionUpdate(mu,Sigma,u)
    
    %Initalise variables
    dt = 0.1;
    a = [0.1,0.1,0.1,0.1];
    
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
    mu = mu + [-(v/w)*sin(theta) + (v/w)*sin(theta + w*dt);...
                  ((v/w)*cos(theta)) - ((v/w)*cos(theta + w*dt));...
                  w*dt];
                 
            %Sigma motion update
    Sigma = (G*Sigma)*(G.')+(V*M)*(V.');
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

function [ ] = plotCircle( x, y, r, c )
%plotCircle plots a circle at coord (x,y) with radius r and colour c
%   Detailed explanation goes here
    t = linspace(0,2*pi);
    plot(x + (r*cos(t)),y + (r*sin(t)),c);
end
