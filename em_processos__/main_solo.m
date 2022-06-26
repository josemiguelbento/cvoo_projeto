close all
clear all
clc

finaltime = 100; % tempo de duração da simulação
StepSize = 0.01;
val_solo=sim('solo','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

gg=plot(val_solo.dist(:,:),val_solo.solo_nosso(:,:),val_solo.dist(:,:),val_solo.solo_prof(:,:));
legend('adaptado','enunciado','Location','northeast');
set(gg,'LineWidth',1.5)
gg=xlabel('Distância percorrida (m)');
gg=ylabel('Altura do solo (m)');
