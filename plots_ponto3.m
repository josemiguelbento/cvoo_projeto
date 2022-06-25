function []=plots(val, cond_ini)
%plots
f=figure();
f.Position = [50 100 1500 450];

subplot(3,3,[1,2])
gg=plot(val.tout,val.h_pt_ref(:,:),val.tout,val.h_pt(:,:));
legend('reference','state');
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Velocidade Subida (m/s)');


subplot(3,3,3)
gg=plot(val.tout,val.de*180/pi);
legend('reference','state');
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Deflexão do Elevador (º)');


subplot(3,3,[4,5])
gg=plot(val.tout,val.u_ref(:,:),val.tout,val.u);
legend('reference','state');
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Velocidade Ar (m/s)');



subplot(3,3,6)
gg=plot(val.tout,val.dsp*180/pi);
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Deflexão do Spoiler (º)');


subplot(3,3,7)
gg=plot(val.tout,val.h(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Altitude (m)');


subplot(3,3,8)
gg=plot(val.tout,val.theta(:,:)*180/pi);
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Ângulo de Picada (º)');

subplot(3,3,9)
gg=plot(val.tout,val.q(:,:)*180/pi);
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Razão Picada (º/s)');


end
