%% tema 50 - Ximas-1 - velocidades ar e subida - aseguimento de solo
%% RP1
clear
close all
clc

g = 9.81;
rad = 180/pi;
kn = 0.514444444;
deg = pi/180;
%vamos buscar os dados da aeronave (planador Ximas), dados no enunciado
%funcao def_model() definida noutro ficheiro
[cond_ini, max_deflec, inert, wing, deriv] = def_model();

%vamos definir as variáveis úteis para
%simplificar a leitura do código e a definição das matrizes
w0 = cond_ini.aa0*cond_ini.u0;
Mu_til = deriv.mwp*deriv.zu/(1-deriv.zwp);
Mw_til = deriv.mw + deriv.mwp*deriv.zw/(1-deriv.zwp);
Mq_til = deriv.mq + deriv.mwp/(1-deriv.zwp)*(cond_ini.u0+deriv.zq);
Mtt_til = -deriv.mwp*g*sin(cond_ini.tt0)/(1-deriv.zwp);
Zu_til = deriv.zu/(1-deriv.zwp);
Zw_til = deriv.zw/(1-deriv.zwp);
Zq_til = (deriv.zq + cond_ini.u0)/(1-deriv.zwp);
Ztt_til = -g*sin(cond_ini.tt0)/(1-deriv.zwp);
Zde_til = deriv.zde/(1-deriv.zwp);
Zdf_til = deriv.zdf/(1-deriv.zwp);
Z_dsp_til = deriv.zdsp/(1-deriv.zwp);
Mde_til = deriv.mde+deriv.mwp*deriv.zde/(1-deriv.zwp);
Mdf_til = deriv.mdf+deriv.mwp*deriv.zdf/(1-deriv.zwp);
Mdsp_til = deriv.mdsp+deriv.mwp*deriv.zdsp/(1-deriv.zwp);


%% RP2 - LQR
%definiçao de novas matrizes do estado trocando theta por velocidade de
%subida (h_pt)

% x = [u; w; q; h_pt]
% definindo a matriz a para este novo estado
a_h_pt(1,1) = deriv.xu;
a_h_pt(1,2) = deriv.xw-g*cos(cond_ini.tt0)/cond_ini.u0;
a_h_pt(1,3) = -w0;
a_h_pt(1,4) = -g*cos(cond_ini.tt0)/cond_ini.u0;
a_h_pt(2,1) = deriv.zu/(1-deriv.zwp);
a_h_pt(2,2) = deriv.zw/(1-deriv.zwp)-g*sin(cond_ini.tt0)/((1-deriv.zwp)*cond_ini.u0);
a_h_pt(2,3) = (deriv.zq + cond_ini.u0)/(1-deriv.zwp);
a_h_pt(2,4) = -g*sin(cond_ini.tt0)/((1-deriv.zwp)*cond_ini.u0);
a_h_pt(3,1) = deriv.mu+deriv.mwp*deriv.zu/(1-deriv.zwp);
a_h_pt(3,2) = deriv.mw+deriv.mwp*deriv.zw/(1-deriv.zwp)-g*sin(cond_ini.tt0)*deriv.mwp/((1-deriv.zwp)*cond_ini.u0);
a_h_pt(3,3) = deriv.mq+deriv.mwp*(cond_ini.u0+deriv.zq)/(1-deriv.zwp);
a_h_pt(3,4) = -g*sin(cond_ini.tt0)*deriv.mwp/((1-deriv.zwp)*cond_ini.u0);
a_h_pt(4,1) = -deriv.zu/(1-deriv.zwp);
a_h_pt(4,2) = -deriv.zu/(1-deriv.zwp)+g*sin(cond_ini.tt0)/((1-deriv.zwp)*cond_ini.u0);
a_h_pt(4,3) = -(deriv.zq + cond_ini.u0)/(1-deriv.zwp)+cond_ini.u0;
a_h_pt(4,4) = g*sin(cond_ini.tt0)/((1-deriv.zwp)*cond_ini.u0);

b_h_pt = [
    deriv.xde deriv.xdsp;
    deriv.zde/(1-deriv.zwp) deriv.zdsp/(1-deriv.zwp);
    deriv.mde+deriv.mwp*deriv.zde/(1-deriv.zwp) deriv.mdsp+deriv.mwp*deriv.zdsp/(1-deriv.zwp);
    -deriv.zde/(1-deriv.zwp) -deriv.zdsp/(1-deriv.zwp);
    ];

c_h_pt = eye(size(a_h_pt));

d_h_pt = zeros(size(b_h_pt));

% Aumentar o estado x = [u; w; q; h_pt; u_int; h]
a_h_pt_int = [
    a_h_pt zeros(4,2);
    1 0 0 0 0 0;
    0 0 0 1 0 0;
    ];

b_h_pt_int = [
    b_h_pt;
    0 0;
    0 0;
    ];

c_h_pt_int = eye(size(a_h_pt_int));

d_h_pt_int = zeros(size(b_h_pt_int));
% Matrices for LQR control
% Bryson Method
% extremos adequados para u w q tt h
du_max=0.5; %m/s
dw_max=0.2; %m/s;
dq_max=5*deg; %deg/s to rad/s
dh_pt_max=0.5;
%dh_max=5; %m

% extremos usados na matriz R:
de_max = 0.3*deg;
dsp_max = 5*deg;

Q = diag([1/du_max^2 1/dw_max^2 1/dq_max^2 1/dh_pt_max^2 0.1/du_max^2 0.1/dh_pt_max^2]);
%R = diag([1/max_deflec.demin^2 1/max_deflec.spmax^2]);
R = diag([1/de_max^2 1/dsp_max^2]);

K_lqr = lqr(a_h_pt_int, b_h_pt_int, Q, R);

%g_h_pt = -c_h_pt*inv(a_h_pt-b_h_pt*K_lqr)*b_h_pt;

%f_h_pt = inv(g_h_pt);

fprintf('\n\nDamp com o LQR')
damp(a_h_pt_int-b_h_pt_int*K_lqr)

%definição das condicoes iniciais para o simulink
x0 = [0 0 0 0];
%x0_h = [0 0 0 0 0];
finaltime = 180; % tempo de duração da simulação
StepSize = 0.01;

%parametros dos sensores pq isto estava a ficar uma confusao

%atuadores
vel_max_at = 60; %º/s
tau = 0.1; %s, cte tempo atuadores

%razao angular q
f_amost = 100; %Hz
V_max_sensq = 4.3; %V
V_min_sensq = 0.7; %V
q_max_sensq = 300; %º/s
q_min_sensq = -300; %º/s
G_sensq = (V_max_sensq-V_min_sensq)/(q_max_sensq-q_min_sensq); %V/(º/s)
rms_sensq = 4.4; %º/s
Offset_sensq = 2.5;

%conversor A/D
conv_max = 5; %V
conv_min = 0; %V
conv_bit = 12;
conv_res = (conv_max - conv_min)/(2^conv_bit);
conv_rms = 1.5*conv_res;
fat_quant = (conv_max-conv_min)/2^12; 

%sensor p_din
p_din_tau = 0.01; %s
pdin_max = 2e3; %Pa
pdin_min = -2e3; %Pa
v_pdin_max = 5; %V
v_pdin_min = 0; %V
rho0 = 1.225; %constante, baixa altitude, kg/m^3
Gp_din = (v_pdin_max-v_pdin_min)/(pdin_max-pdin_min);
offset_pdin = 2.5;

%radio altimetro
ft = 0.3048; %ft to m
h_ra_max = 1500*ft;
h_ra_min = 0;
G_ra = 10e-3/ft; %Vdc/ft
ra_rms = 0.015; %ruido relativo
Offset_ra = 0;
offset_alt = 0;
fat_conv_ra = 1/3;
%[ra_rms^2/f_amost]

h_ref = 100; % 100 m acima do solo
%h_solo = 100; 
%h_pt_ref = -1;
%u_ref = 2;
h_pt_ref = 0;
u_ref = 0;
u0 =6;
dist_aux = 40*u0:10*u0:60*u0;
ref_aux = 0:2.5*u0:5*u0;
p = polyfit(dist_aux,ref_aux,2);
altitude_solo = 5*u0;

%simulacao
vel_vento = 10; %m/s
h_solo_0 = 5* cond_ini.u0+100; %m
h_seguimento = 100; %m


%% Estimador - vamos usar as matrizes do estado aumentado com h
a_h_pt_h = [
    a_h_pt zeros(4,1);
    0 0 0 1 0 ;
    ];

b_h_pt_h = [
    b_h_pt;
    0 0;
    ];

c_h_pt_h = [
    1 0 0 0 0
    0 0 1 0 0
    0 0 0 0 1
    ];

%c_h_pt_h = eye(size(a_h_pt_h));
d_h_pt_h = zeros(3,2);
%d_h_pt_h = zeros(size(b_h_pt_h));

ge=eye(size(a_h_pt_h));
%qe=diag([1 1 1 1 1]);
%re=100000*diag([1 1 1]);
%re=100*diag([1 1 1]);
qe=diag([0.01 0.01 0.01 0.01 0.01]);
re=diag([1.5*5/2^12 4.4*pi/180+1.5*5/2^12 0.015*100]);
L=lqe(a_h_pt_h,ge,c_h_pt_h,qe,re);
[ae,be,ce,de]=estim(a_h_pt_h,b_h_pt_h,c_h_pt_h,d_h_pt_h,L,[1, 2, 3],[1, 2]);

x0_e = [0 0 0 0 cond_ini.h0+h_solo_0];


val=sim('cvoo_g19_ponto5','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

%plots
plots_ponto5(val)
