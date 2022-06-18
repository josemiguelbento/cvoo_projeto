function []=plots(val)
%plots
f=figure();
f.Position = [50 100 1500 450];

subplot(3,3,3)
gg=plot(val.tout,180/pi*val.de(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('elevator (deg)');


subplot(3,3,6)
gg=plot(val.tout,180/pi*val.dsp(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('spoiler (deg)');


subplot(3,3,4)
gg=plot(val.tout,val.u(:,:),val.tout,val.u_ref(:,:));
legend('state','reference');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('velocidade ar (m/s)');


subplot(3,3,5)
gg=plot(val.tout,val.h_pt(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('velocidade subida (m/s)');



subplot(3,3,7)
gg=plot(val.tout,val.w(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('velocidade vertical (m/s)');


subplot(3,3,8)
gg=plot(val.tout,val.q(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('razão picada (rad/s)');


subplot(3,3,9)
gg=plot(val.tout,180/pi*val.tt(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('angulo de picada (deg)');


subplot(3,3,[1,2])
gg=plot(val.tout,val.h(:,:), val.tout,val.h_ref(:,:));
legend('state','reference');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('altitude (m)');
end
