clc
clear all

u0 = 0.01;
syms dist
% altitude_solo = piecewise((dist >= 0) & (dist < 20*u0),-dist/4+5*u0,(dist >= 20*u0) & (dist < 40*u0),0,(dist >= 40*u0) & (dist < 50*u0),(dist-40*u0)^2,(dist >= 50*u0) & (dist < 60*u0),log(2*dist-50*u0)-((50-40)*u0)^2,(dist >= 60*u0) & (dist < 80*u0),5*u0);
% fplot(altitude_solo)

altitude_solo2 = piecewise((dist >= 0) & (dist < 20*u0),-dist/4+5*u0,(dist >= 20*u0) & (dist < 40*u0),0,(dist >= 40*u0) & (dist < 50*u0),(dist-50*u0)^(1/3)+(2.5*u0)^(1/3),(dist >= 60*u0) & (dist < 80*u0),5*u0);
fplot(altitude_solo2)
% if dist >= 0 && dist < 20*u0
%     altitude_solo = -dist/4 + 5*u0;
% elseif dist >= 20*u0 && dist < 40*u0
%     altitude_solo = 0;
% elseif dist >= 40*u0 && dist < 50*u0
%     altitude_solo = (dist-40*u0)^2;
% elseif dist >= 50*u0 && dist < 60*u0
%     altitude_solo = log(2*dist-50*u0)+((50-40)*u0)^2;
% elseif dist >= 60*u0 && dist <= 80*u0
%     altitude_solo = 5*u0;
% end