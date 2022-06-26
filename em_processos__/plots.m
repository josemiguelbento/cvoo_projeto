function []=plots(val)
%plots
finaltime = 180;
f=figure();
f.Position = [50 100 1500 600];

subplot(4,3,[1,2])
gg=plot(val.tout,val.u(:,:),val.tout,val.u_e(:,:),val.tout,val.u_ref(:,:));
legend('real','estimated','reference','Location','northeast');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('velocidade ar (m/s)');
xlim([0 finaltime])


subplot(4,3,3)
gg=plot(val.de_ref.time,180/pi*val.de_ref.signals.values(:,:),val.de.time,180/pi*val.de.signals.values(:,:));
legend('requested','actuator','Location','northeast');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('elevator (deg)');
xlim([0 finaltime])


subplot(4,3,[4,5])
gg=plot(val.tout,val.h_pt(:,:),val.tout,val.h_pt_e(:,:),val.tout,val.h_pt_ref(:,:));
legend('real','estimated','reference','Location','southeast');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('velocidade subida (m/s)');
xlim([0 finaltime])


subplot(4,3,6)
gg=plot(val.dsp_ref.time,180/pi*val.dsp_ref.signals.values(:,:),val.dsp.time,180/pi*val.dsp.signals.values(:,:));
legend('requested','actuator','Location','southwest');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('spoiler (deg)');
xlim([0 finaltime])


subplot(4,3,9)
gg=plot(val.tout,val.q(:,:)*180/pi,val.tout,val.q_e(:,:)*180/pi);
legend('real','estimated','Location','southwest');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('razão picada (º/s)');
xlim([0 finaltime])


subplot(4,3,[7,8])
gg=plot(val.tout,val.h(:,:),val.tout,val.h_e(:,:),'--',val.tout,val.h_ref(:,:),val.tout,val.h_solo(:,:));
legend('real','estimated','reference','ground','Location','northeast');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('altitude (m)');
xlim([0 finaltime])


subplot(4,3,10)
gg=plot(val.tout,val.w(:,:),val.tout,val.w_e(:,:));
legend('real','estimated','Location','southwest');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('velocidade vertical (m/s)');
xlim([0 finaltime])

subplot(4,3,11)
gg=plot(val.tout,val.aoa(:,:)*180/pi,val.tout,val.aoa_e(:,:)*180/pi);
legend('real','estimated','Location','southwest');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('ângulo de ataque (º)');
xlim([0 finaltime])

subplot(4,3,12)
gg=plot(val.tout,val.tt(:,:)*150/pi,val.tout,val.tt_e(:,:)*150/pi,'--');
legend('real','estimated','Location','northwest');
set(gg,'LineWidth',1.5)
gg=xlabel('Time (s)');
gg=ylabel('ângulo de picada (º)');
xlim([0 finaltime])

end
