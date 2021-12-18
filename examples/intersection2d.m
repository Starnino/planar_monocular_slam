close all
clear
clc

source '../tools/geometry_helpers.m'
syms x t y1 y2 y3 y4 y5

y1 = 2*x + 1;
y2 = 8 - x;
y3 = -10*x + 50;
y4 = 10*x - 20;

p1 = [solve(y1 == 0); 0];
p2 = [solve(y2 == 0); 0];
p3 = [solve(y3 == 0); 0];
p4 = [solve(y4 == 0); 0];

d1 = [diff(solve(y1 == t, x)); 1];
d2 = [diff(solve(y2 == t, x)); 1];
d3 = [diff(solve(y3 == t, x)); 1];
d4 = [diff(solve(y4 == t, x)); 1];

p = linesIntersection([p1,p2,p3,p4],[d1,d2,d3,d4]);

x = 0:10;
y1 = eval(y1);
y2 = eval(y2);
y3 = eval(y3);
y4 = eval(y4);
p = eval(p);

plot(x, y1, x, y2, x, y3, x, y4)
hold on 
scatter(p(1), p(2), 'filled') 
hold off
grid on