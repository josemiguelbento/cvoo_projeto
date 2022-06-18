function []=plots_sae(val)
%plots
f=figure();
f.Position = [50 100 1500 450];

subplot(2,3,1)
gg=plot(val.tout,180/pi*val.de_step(:,:),val.tout, 180/pi*val.dsp_step(:,:));
legend('elevator','spoiler');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('entradas em degrau (deg)');


subplot(2,3,2)
gg=plot(val.tout,val.u(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('velocidade ar (m/s)');


subplot(2,3,3)
gg=plot(val.tout,val.w(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('velocidade subida (m/s)');


subplot(2,3,4)
gg=plot(val.tout,val.q(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('razão de picada (rad/s)');


subplot(2,3,5)
gg=plot(val.tout,180/pi*val.tt(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('angulo de picada (deg)');


subplot(2,3,6)
gg=plot(val.tout,val.h(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('altitude (m)');
end