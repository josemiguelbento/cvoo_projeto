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


%vamos definir a matriz da dinâmica tradicional, a, com entradas:
%   x = [u; w; q; tt]

a = [
    deriv.xu deriv.xw -w0 -g*cos(cond_ini.tt0);
    Zu_til  Zw_til Zq_til Ztt_til;
    Mu_til Mw_til Mq_til Mtt_til;
    0 0 1 0;
    ];

%vamos definir a matriz do controlo b, com entradas de controlo:
%   u = [de; dsp]

b_sf = [
    deriv.xde deriv.xdsp;
    Zde_til Z_dsp_til;
    Mde_til Mdsp_til;
    0 0;
    ];

%as matrizes c e d ficam então definidas, assumindo y = x (a saída é o 
%próprio estado)
c = eye(size(a));

d_sf = zeros(size(b_sf));

%vamos definir a matriz da dinâmica, a_h, com entradas (incluindo h):
%   x_h = [u; w; q; tt; h]
a_h = [
    deriv.xu deriv.xw -w0 -g*cos(cond_ini.tt0) 0;
    Zu_til  Zw_til Zq_til Ztt_til 0;
    Mu_til Mw_til Mq_til Mtt_til 0;
    0 0 1 0 0;
    0 -1 0 cond_ini.u0 0;
    ];

%vamos definir a matriz do controlo b_h, com entradas de controlo:
%   u = [de; df; dsp]
b_h = [
    deriv.xde deriv.xdf deriv.xdsp;
    Zde_til Zdf_til Z_dsp_til;
    Mde_til Mdf_til Mdsp_til;
    0 0 0;
    0 0 0;
    ];

%as matrizes c_h e d_h ficam então definidas, assumindo y_h = x_h (a saída  
%é o próprio estado)
c_h = eye(size(a_h));

d_h = zeros(size(b_h));


%vamos analisar as características dinâmicas do sistema, nomeadamente a
%localização dos pólos e as suas frequências naturais e amortecimento
fprintf('\n\nDamp do sistema inicial')
damp(a_h)
%como se pode verificar há dois pólos instáveis (no semiplano complexo 
%direito)

%vamos obter a representação em espaço de estados associada a esta
%formulação do modelo da aeronave e representar graficamente os seus polos
%e zeros
sys = ss(a_h,b_h,c_h,d_h);
%pzmap(sys)

%agora vamos estudar o comportamento dos modos longitudinais
[wn,zeta,p]=damp(a_h);
p_re = nonzeros(real(p)); %vetor dos polos com parte real não nula

%primeiro passo é separar os pólos do Período Curto dos pólos do Fugóide
%    Fugóide (pólos com menor parte real, em valor absoluto):
[M_fug,I_fug] = min(abs(p_re));
t_fug = log(2)/M_fug;

%    Período Curto (pólos com maior parte real, em valor absoluto):
[M_pc,I_pc]=max(abs(p_re));
t_pc = log(2)/M_pc;

fprintf('\n\nFugóide:')
if(p_re(I_fug)>0)
    str = ['\n\tT_2 =' ' ' num2str(t_fug) 's'];
    fprintf(str)
elseif(p_re(I_fug)<0)
    str = ['\n\tT_{1/2} =' ' ' num2str(t_fug) 's'];
    fprintf(str)
end

fprintf('\n\nPeríodo Curto:')
if(p_re(I_pc)>0)
    str = ['\n\tT_2 = ' num2str(t_pc) 's' '\n'];
    fprintf(str)
elseif(p_re(I_pc)<0)
    str = ['\n\tT_{1/2} = ' num2str(t_pc) 's' '\n'];
    fprintf(str)
end

% %vamos obter as funções de transferência para cada um dos atuadores
[num_de,den_de]=ss2tf(a_h,b_h,c_h,d_h,1); %de (elevador)
[num_df,den_df]=ss2tf(a_h,b_h,c_h,d_h,2); %df (flaps)
[num_dsp,den_dsp]=ss2tf(a_h,b_h,c_h,d_h,3); %dsp (spoiler)

%% RP2 - SAE -------------------------------------------------------------
% Tirando flaps como superficie de controlo
b_h_sf = b_h(:,[1,3]);
d_h_sf = d_h(:,[1,3]);
%avaliar controlabilidade e observabilidade
Co = ctrb(a_h,b_h);
k_co = rank(Co);
% como a característica da matriz controlabilidade é igual ao número de
% estados, o sistema é controlável

Ob = obsv(a_h,c_h);
k_ob = rank(Ob);
% como a característica da matriz observabilidade é igual ao número de
% estados, o sistema é observável

% como o problema é o fugoide vamos realimentar a velocidade (o estado que
% melhor traduz este modo) para a entrada do spoiler. (u para dsp)
num_dsp_u = num_dsp(1,:); %1- estamos a avaliar a estabilização do estado velocidade ar com spoiler
sys_usp = tf(num_dsp_u,den_dsp);
%figure()
%rlocus(-sys_usp) %realimentação positiva - já fica estável tanto para fugoide e PC
%sgrid

%escolhemos um K tal q os polos do fugoide apresentem pelo menos damping de
%0.6

k_u_dsp = 0.65;
k_siso = [0 -k_u_dsp;0 0; 0 0; 0 0; 0 0]; %negativo pq este k era para realimentaçao positiva

fprintf('\n\nDamp realimentação positiva de u para o dsp')
damp(a_h-b_h_sf*k_siso')

% realimentação de w para o dsp
a_h_cl_fug = a_h-b_h_sf*k_siso';
[num_dsp_2o,den_dsp_2o]=ss2tf(a_h_cl_fug,b_h,c_h,d_h,3); %dsp (spoiler)
num_dsp_w = num_dsp_2o(2,:); %2- estamos a avaliar a estabilização do estado velocidade subida com elevator
sys_wsp = tf(num_dsp_w,den_dsp_2o);
% figure()
% rlocus(sys_wsp)
% sgrid

k_w_dsp = 4.1;
k_siso2 = [0 -k_u_dsp;0 k_w_dsp; 0 0; 0 0; 0 0];
fprintf('\n\nDamp realimentação positiva de w para o dsp')
damp(a_h-b_h_sf*k_siso2')


sys_SAE = ss(a_h-b_h_sf*k_siso2', b_h_sf, c_h, d_h_sf);
figure()
t_final_step = 100;
opt = stepDataOptions('InputOffset',0,'StepAmplitude',1/180*pi);

step(sys_SAE, t_final_step,opt)

a_h_SAE = a_h-b_h_sf*k_siso2';

%definição das condicoes iniciais para o simulink
x0_h = [0 0 0 0 0];
finaltime = 100; % tempo de duração da simulação sistema com SAE
StepSize = 0.01;

%definição dos retângulos de input
t_de_ini = 1;
de_step = 1*pi/180;
t_de_step = 2;

t_dsp_ini = 50;
dsp_step = 10*pi/180;
t_dsp_step = 10;

val_sae=sim('cvoo_g19_sae','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

plots_sae(val_sae)

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
de_max = 0.2*deg;
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
finaltime = 100; % tempo de duração da simulação
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
conv_res = (conv_max - conv_min)/(2*(2^conv_bit)-1);
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
h_solo_0 = 5* cond_ini.u0; %m
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
qe=diag([1 1 1 1 1]);
%re=100000*diag([1 1 1]);
re=1000*diag([1 1 1]);
L=lqe(a_h_pt_h,ge,c_h_pt_h,qe,re);
[ae,be,ce,de]=estim(a_h_pt_h,b_h_pt_h,c_h_pt_h,d_h_pt_h,L,[1, 2, 3],[1, 2]);

x0_e = [0 0 0 0 cond_ini.h0+h_solo_0];


%val=sim('cvoo_g19_servomecanismo','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
val=sim('cvoo_g19_seguimento_solo','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
%val=sim('cvoo_g19','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

%plots
plots(val,cond_ini)

% %% RP2 - controlo modal
% % fugoide
% wn_fug = 0.6;
% xi_fug = 0.7;
% % periodo curto
% wn_pc = 8;
% xi_pc = 0.8;
% 
% % valores proprios
% val_p = [
%     -xi_pc*wn_pc + 1i*(wn_pc*sqrt(1-xi_pc^2));
%     -xi_pc*wn_pc - 1i*(wn_pc*sqrt(1-xi_pc^2));
%     -xi_fug*wn_fug + 1i*(wn_fug*sqrt(1-xi_fug^2));
%     -xi_fug*wn_fug - 1i*(wn_fug*sqrt(1-xi_fug^2));
%     ];
% % vetores proprios
% 
