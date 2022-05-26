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
b=[deriv.xde deriv.xdsp;deriv.zde deriv.zdsp;deriv.mde+deriv.mwp*deriv.zde deriv.mdsp;0 0];

c = eye(size(a));

d = zeros(size(b));

sys = ss(a,b,c,d);

[num_de,den_de]=ss2tf(a,b,c,d,1);

[num_sp,den_sp]=ss2tf(a,b,c,d,2);

%sys = tf(num(1,:),den);
%rlocus(sys)
pzmap(sys)
damp(a)

%% fugoide aproximado 
%(com diferentes variaveis pq o polinomio caracteristico
%de um sistema nao depende das variaveis de estado que o definem)

a_fug=[
deriv.xu -g*cond_ini.tt0;
-deriv.zu/cond_ini.u0 0];

%b=[deriv.xde deriv.xdt;deriv.zde 0;deriv.mde+deriv.mwp*deriv.zde deriv.mdt;0 0];
%b_fug=[deriv.xde deriv.xdsp;-deriv.zde/cond_ini.u0 -deriv.zdsp/cond_ini.u0];

damp(a_fug) %fugoide estavel????


%% Período Curto Aproximado

a_pc=[
deriv.zw cond_ini.u0;
deriv.mw+deriv.mwp*deriv.zw deriv.mq+deriv.mwp*cond_ini.u0];

damp(a_pc)


