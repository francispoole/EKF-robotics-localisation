%trilateration.m
%Test
clc
clear all


p = [100,100];

A=[0,0];
B=[20,0];
C=[10,10];
D=[1,-1];

rA = norm(p-A)
rB = norm(p-B)
rC = norm(p-C)

x = (rA^2 - rB^2 + B(1)^2) / (2 * B(1))

y = (rA^2 - rC^2 - (2 * x * C(1)) + C(1)^2 + C(2)^2) / (2 * C(2))






%Plot
plot(p(1),p(2),'x')
hold on
plot([A(1),B(1),C(1)],[A(2),B(2),C(2)],'o')
plot(x,y,'ro')
hold off