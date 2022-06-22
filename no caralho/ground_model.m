clc
clear all

% u0 = 0.01;

dist_aux = 40*u0:10*u0:60*u0;
grd_aux = 0:2.5*u0:5*u0;
p = polyfit(dist_aux,grd_aux,2);

syms dist
grd = piecewise((dist >= 0) & (dist < 20*u0),-dist/4 + 5*u0,(dist >= 20*u0) & (dist < 40*u0),0,(dist >= 40*u0) & (dist < 60*u0),p(1)*grd^2 + p(2)*grd + p(3),(dist >= 60*u0) & (dist <= 80*u0),5*u0);

grd = piecewise((dist >= 0) & (dist < 20*u0),-dist/4 + 5*u0,(dist >= 20*u0) & (dist < 40*u0),0,(dist >= 40*u0) & (dist < 50*u0),-sqrt((2.5*u0)^2-dist^2),(dist >= 50*u0) & (dist < 60*u0),sqrt((2.5*u0)^2-dist^2),(dist >= 60*u0) & (dist <= 80*u0),5*u0);


fplot(grd)

if dist >= 0 && dist < 20*u0
    grd = -dist/4 + 5*u0;
elseif dist >= 20*u0 && dist < 40*u0
    grd = 0;
elseif dist >= 40*u0 && dist < 60*u0
    dist_aux = 40*u0:10*u0:60*u0
    grd_aux = 0:2.5*u0:5*u0
    p = polyfit(x,y,3);
    grd = p(1)*grd^3 + p(2)*grd^2 + p(3)*grd + p(4)
elseif dist >= 60*u0 && dist <= 80*u0
    grd = 5*u0
end


fileID = fopen('perfil_solo.txt','r');
h_ref = fscanf(fileID,'%f');
fclose(fileID);