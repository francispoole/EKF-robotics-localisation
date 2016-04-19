clc
clear all
close all

motionModel = [
    75, 52, 2.2;
    85, 74, 3.6;
    65, 52, 2.7;
    63, 49, 2.7;
    55, 42, 2;
    58, 45, 2.2;
    61, 46, 2.1;
    80, 45, 2.3;
    66, 44, 2.5;
    71, 54, 3.1
    ];

ekf = [
    94, -52, -1.0;
    94, 81, -1.3;
    65, -73, -2.1;
    77, 48, 2.6;
    52, 68, 1.2
    58, 66, -0.6;
    -25, -87, -2.0;
    -93, -14, -2.7;
    87, -24, 1.75;
    32, 89, -0.9;
    ];

real = [
    87, 55, 2.2;
    91, 84, 2.2;
    80, 57, 1.3;
    70, 50, 2.4;
    66, 46, 2.1;
    58,48, 2.4;
    68, 47, 2.2;
    78, 44, 2.4;
    73, 45, 2.4;
    78, 52, 2.6;
    ];

motionModelPositionErr = sqrt(sum((motionModel(:,1:2)-real(:,1:2)).^2,2))
ekfPositionErr = sqrt(sum((ekf(:,1:2)-real(:,1:2)).^2,2))


motionModelBearingErr = abs(mod(motionModel(:,3)-real(:,3) + pi, 2*pi) - pi)
ekfBearingErr = abs(mod(ekf(:,3)-real(:,3) + pi, 2*pi) - pi)

realRange = sqrt(sum(real(:,1:2).^2,2))
motionModelRange = sqrt(sum(motionModel(:,1:2).^2,2))
ekfRange = sqrt(sum(ekf(:,1:2).^2,2))

motionModelRangeErr = abs(motionModelRange - realRange)
ekfRangeErr = abs(ekfRange - realRange)

%Plot Agent positions
for n = 1:10
    figure(n)
    set(gca,'fontsize',17);
    plot([real(n,1), real(n,1) + 7 * cos(real(n,3))], [real(n,2), real(n,2) + 7 * sin(real(n,3))], 'k');
    hold on
    plot([ekf(n,1), ekf(n,1) + 7 * cos(ekf(n,3))], [ekf(n,2), ekf(n,2) + 7 * sin(ekf(n,3))], 'b');
    plot([motionModel(n,1), motionModel(n,1) + 7 * cos(motionModel(n,3))], [motionModel(n,2), motionModel(n,2) + 7 * sin(motionModel(n,3))], 'r');
    plot([real(n,1),ekf(n,1)],[real(n,2),ekf(n,2)],'--b');
    plot([real(n,1),motionModel(n,1)],[real(n,2),motionModel(n,2)],'--r');
    
    plot(0,0,'mO')
    plot(50,50,'g*')
    legend('Robot', 'EKF Prediction', 'Motion Model Prediction', 'EKF Position Error', 'Motion Model Position Error', 'Light Source', 'Robot Starting Position','Location','northwest')
    title('Localisation Methods of Mobile Robot')
    xlabel('x-position')
    ylabel('y-position')
    
    plotCircle(motionModel(n,1), motionModel(n,2),7,'r');
    plotCircle(ekf(n,1), ekf(n,2),7,'b');
    plotCircle(real(n,1), real(n,2),7,'k');
    
    hold off
    axis([-100,100,-100,100])
end

%Plot Position Error
figure(11)
hold on
bar(1:2,[mean(motionModelPositionErr),mean(ekfPositionErr)])
errorbar(1:2,[mean(motionModelPositionErr),mean(ekfPositionErr)],[std(motionModelPositionErr),std(ekfPositionErr)],'r.')
hold off
set(gca,'fontsize',17);
title('Average Position Error of Different Localisation Algorithms');
ylabel('Position Error (cm)');
xlabel('Localisation Technique')
set(gca, 'XTick', 1:2, 'XTickLabel', {'Motion Model', 'EKF'});

%Plot Position Error
figure(12)
hold on
bar(1:2,[mean(motionModelBearingErr),mean(ekfBearingErr)])
errorbar(1:2,[mean(motionModelBearingErr),mean(ekfBearingErr)],[std(motionModelBearingErr),std(ekfBearingErr)],'r.')
hold off
set(gca,'fontsize',17);
title('Average Bearing Error of Different Localisation Algorithms');
ylabel('Bearing Error (radians)');
xlabel('Localisation Technique')
set(gca, 'XTick', 1:2, 'XTickLabel', {'Motion Model', 'EKF'});

%Range Error
figure(13)
hold on
bar(1:2,[mean(motionModelRangeErr),mean(ekfRangeErr)])
errorbar(1:2,[mean(motionModelRangeErr),mean(ekfRangeErr)],[std(motionModelRangeErr),std(ekfRangeErr)],'r.')
hold off
set(gca,'fontsize',17);
title('Average Bearing Error of Different Localisation Algorithms');
ylabel('Bearing Error (radians)');
xlabel('Localisation Technique')
set(gca, 'XTick', 1:2, 'XTickLabel', {'Motion Model', 'EKF'});


