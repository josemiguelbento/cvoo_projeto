%tema 50 - Ximas 1

clear
close all
clc

g = 9.81;

[cond_ini, max_deflec, inert, wing, deriv] = def_model();

a=[
deriv.xu deriv.xw -cond_ini.aa0*cond_ini.u0 -g*cos(cond_ini.tt0);
deriv.zu deriv.zw cond_ini.u0 -g*sin(cond_ini.tt0);
deriv.mu+deriv.mwp*deriv.zu deriv.mw+deriv.mwp*deriv.zw deriv.mq+deriv.mwp*cond_ini.u0 -deriv.mwp*g*sin(cond_ini.tt0);
0 0 1 0];

%b=[deriv.xde deriv.xdt;deriv.zde 0;deriv.mde+deriv.mwp*deriv.zde deriv.mdt;0 0];
b=[deriv.xde deriv.xdsp;deriv.zde 0;deriv.mde+deriv.mwp*deriv.zde deriv.mdsp;0 0];

damp(a)

