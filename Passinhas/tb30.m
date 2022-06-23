%Conversao de graus para radianos
pi= 3.14159265359; deg=pi/180;
%Conversao de n?s para m/s
ms=0.514444444;
%Conversao de metros para p?s
ft=0.3048;
%tb-30 flight condition: 4
h=500; M=0.13; aa0=5.27*deg; gg0=10*deg; u0=85.4*ms; flaps=40*deg;
th0=0.13; de0=0.00; da0=0.00; dr0=0.00; tt0=aa0+gg0; g=9.81;
Teng=0.10; demax=28*deg; demin=-21*deg; damax=17*deg; drmax=23*deg; flapmax=40*deg;
%Inertial Data:
m=642; Ix=786; Iy=947; Iz=1576; Ixz=1; 
%Wing Data:
S=16.26; b=10.998; c=1.033; aamax=9.47;
%Derivadas de estabilidade
xu=-0.0968; xw=-0.0462; zu=-0.4451; zw=-5.5493; zwp= -0.0200; zq=-2.6208; mu=0.0000; mw=-1.1196; mq=-1.2875; mwp=-0.1407;        
ybb=-0.2859; lbb=-6.8951; nbb=12.5093; yp=0.0397; lp=-7.9402; np=0.7591; yr=-0.0370; lr=-1.3647; nr=-0.1174; 
xde=0.000; zde=-10.653; mde=-14.673; xdsp=-2.736; zdsp=0.000; mdsp=0.000; xdt=8.237; zdt=0.000; mdt=0.000;
Lda=-201.239; Nda=-0.598; Ydr=-0.053; Ldr=-4.930; Ndr=-8.198;


% __________ Ponto 1 _______________

%Matrizes A,B,C,D iniciais
a=[xu xw -aa0*u0 -g*cos(tt0);
   zu zw u0 -g*sin(tt0);
   mu+mwp*zu mw+mwp*zw mq+mwp*u0 -mwp*g*sin(tt0); 
   0 0 1 0];
b=[xde xdt; zde 0; mde+mwp*zde mdt; 0 0];
c=eye(4);
d=zeros(4,2);
%damp(a)

% __________ Ponto 2 _______________

% rlocus(a,b(:,2),[1 0 0 0],0)
% sgrid
% k=rlocfind(a,b(:,2),[1 0 0 0],0)
% damp(a-b(:,2)*k*[1 0 0 0])
% af=a-b(:,2)*[0.0340 0 0 0]; %u dt fugoide
% damp(af);

% __________ Ponto 3 _______________

%Novas matrizes com o estado altitude
ah=[xu xw -aa0*u0 -g*cos(tt0) 0;
   zu zw u0 -g*sin(tt0) 0; 
   mu+mwp*zu mw+mwp*zw mq+mwp*u0 -mwp*g*sin(tt0) 0;
   0 0 1 0 0;
   0 -1 0 u0 0];
bh=[xde xdt; zde 0; mde+mwp*zde mdt; 0 0; 0 0];
ch=eye(5);
dh=zeros(5,2);

%M?todo de Bryson
wmax=10*deg; 
qmax=2.5*deg;
ttmax=3*deg;
umax=1;
dtmax=0.2;
demax=3*deg;
hmax=1; 
q=diag([1/umax^2 1/wmax^2 1/qmax^2 1/ttmax^2 1/hmax^2]);
r=diag([1/demax^2 1/dtmax^2]);

k2=lqr(ah,bh,q,r)

damp(ah-bh*k2);
sys=ss(ah-bh*k2,bh,ch,dh);
g=dcgain(sys);
%Calculo da matriz de pre-multiplicacao para MIMO
f=([1 0 0 0 0;0 0 0 0 1]*g)^-1;

%Gr?fico da resposta a pedidos de velocidade e altitude
% simulacao = 'ponto3'; eval(simulacao)
% sim(simulacao)
% subplot(211),plot(var.time,var.signals.values(:,1)),ylabel('u [m/s]')
% subplot(212),plot(var.time,var.signals.values(:,5)),ylabel('h [m]')


