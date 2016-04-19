y = [750 811 771 760
731	803	760	743
564	673	601	616
447	566	473	500
382	497	408	450
350	455	365	398
319	420	321	367]

figure(1)
hold on
plot(y(:,1),'r')
plot(y(:,2))
plot(y(:,3),'k')
plot(y(:,4),'g')

hold off
legend('Back Sensor','Right Sensor','Front Sensor','Left Sensor')