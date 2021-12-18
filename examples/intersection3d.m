close all
clear
clc

source '../tools/geometry_helpers.m'
syms x z t y1 y2 y3 y4 y5

y1 = 2*x + 1;
y2 = 8 - x;
y3 = -10*x + 50;
y4 = 10*x - 20;
z1 = y1;
z2 = y2;
z3 = y3;
z4 = y4;

p1 = [solve(y1 == 0); 0; 0];
p2 = [solve(y2 == 0); 0; 0];
p3 = [solve(y3 == 0); 0; 0];
p4 = [solve(y4 == 0); 0; 0];

d1 = [diff(solve(y1 == t, x)); 1; 1];
d2 = [diff(solve(y2 == t, x)); 1; 1];
d3 = [diff(solve(y3 == t, x)); 1; 1];
d4 = [diff(solve(y4 == t, x)); 1; 1];

p = linesIntersection([p1,p2,p3,p4],[d1,d2,d3,d4]);

x = 0:10;
y1 = eval(y1);
y2 = eval(y2);
y3 = eval(y3);
y4 = eval(y4);
z1 = eval(z1);
z2 = eval(z2);
z3 = eval(z3);
z4 = eval(z4);
p = eval(p);

plot3(x, y1, z1, x, y2, z2, x, y3, z3, x, y4, z4)
hold on 
scatter3(p(1), p(2), p(3), 'filled') 
hold off
grid on