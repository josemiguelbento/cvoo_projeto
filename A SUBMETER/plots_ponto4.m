function []=plots(val, cond_ini)
f=figure();
f.Position = [50 100 1500 450];

%ascent speed
subplot(3,3,1)
gg=plot(val.tout,val.h_pt(:,:),val.tout,val.h_pt_e(:,:),val.tout,val.h_pt_ref(:,:));
%gg=plot(val.tout,val.h_pt_ref(:,:),val.tout,val.h_pt(:,:));
legend('real','estimated','reference');
%legend('reference','state');
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Velocidade Subida (m/s)');

%elevator deflection
subplot(3,3,3)
gg=plot(val.de_ctr.time,val.de_ctr.signals.values*180/pi,val.de.time,val.de.signals.values*180/pi);
legend('requested','actuator');
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Deflexão do Elevador (º)');

%airspeed 
subplot(3,3,2)
gg=plot(val.tout,val.u,val.tout,val.u_e,val.tout,val.u_ref(:,:));
legend('real','estimated','reference');
% gg=plot(val.tout,val.u_ref(:,:),val.tout,val.u);
% legend('reference','state');
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Velocidade Ar (m/s)');

%vertical airspeed 
subplot(3,3,4)
gg=plot(val.tout,val.w(:,:),val.tout,val.w_e(:,:));
legend('real','estimated','Location','Southeast');
% gg=plot(val.tout,val.w(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Velocidade Vertical (m/s)');

%spoiler deflection
subplot(3,3,5)
gg=plot(val.dsp_ctr.time,val.dsp_ctr.signals.values*180/pi,val.dsp.time,val.dsp.signals.values*180/pi);
legend('requested','actuator','Location','Southeast');
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Deflexão do Spoiler (º)');

%altitude
subplot(3,3,6)
gg=plot(val.tout,val.h(:,:),val.tout,val.h_e(:,:),'--');
legend('real','estimated');
% gg=plot(val.tout,val.h(:,:));
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Altitude (m)');

%angulo picada theta
subplot(3,3,7)
gg=plot(val.tout,val.theta(:,:)*180/pi,val.tout,val.theta_e(:,:)*180/pi,'--');
legend('real','estimated');
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Ângulo de Picada (º)');

%razao picada q
subplot(3,3,8)
gg=plot(val.tout,val.q(:,:)*180/pi,val.tout,val.q_e(:,:)*180/pi);
legend('real','estimated','Location','Southeast');
%gg=plot(val.tout,val.q(:,:)*180/pi);
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Razão Picada (º/s)');

%angulo de ataque
subplot(3,3,9)
gg=plot(val.tout,val.aa(:,:)*180/pi,val.tout,val.aa_e(:,:)*180/pi);
legend('real','estimated','Location','Southeast');
set(gg,'LineWidth',1.5)
gg=xlabel('Tempo (s)');
gg=ylabel('Angulo Ataque (º)');


end
