function []=plots(val, cond_ini)
%plots
f=figure();
f.Position = [50 100 1500 450];

subplot(3,3,[1,2])
gg=plot(val.tout,val.u(:,:),val.tout,val.u_e(:,:),val.tout,val.u_ref(:,:));
%legend('state','reference');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('velocidade ar (m/s)');


subplot(3,3,3)
gg=plot(val.de.time,180/pi*val.de.signals.values(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('elevator (deg)');


subplot(3,3,[4,5])
gg=plot(val.tout,val.h_pt(:,:),val.tout,val.h_pt_e(:,:),val.tout,val.h_pt_ref(:,:));
%legend('state','reference');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('velocidade subida (m/s)');


subplot(3,3,6)
gg=plot(val.dsp.time,180/pi*val.dsp.signals.values(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('spoiler (deg)');


subplot(3,3,7)
gg=plot(val.tout,val.w(:,:),val.tout,val.w_e(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('velocidade vertical (m/s)');


subplot(3,3,8)
gg=plot(val.tout,val.q(:,:),val.tout,val.q_e(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');

gg=ylabel('razão picada (rad/s)');


% subplot(3,3,9)
% gg=plot(val.tout,180/pi*val.tt(:,:));
% set(gg,'LineWidth',1.5)
% gg=xlabel('Time (s)');
% 
% gg=ylabel('angulo de picada (deg)');


subplot(3,3,9)
gg=plot(val.tout,val.h(:,:),val.tout,val.h1(:,:),val.tout,val.h_solo(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('altitude (m)');
gg = legend('h_abs','h_abs_ref','h_solo');
end
