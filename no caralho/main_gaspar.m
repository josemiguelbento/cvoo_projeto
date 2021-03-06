%% tema 50 - Ximas-1 - velocidades ar e subida - seguimento de solo
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
pzmap(sys)

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
figure()
rlocus(-sys_usp) %realimentação positiva - já fica estável tanto para fugoide e PC
sgrid

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
figure()
rlocus(sys_wsp)
sgrid

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

%% LQR
% x = [u; w; q; tt; h; u_i; h_i]

%definiçao de novas matrizes do estado aumentado com altitude e
%integradores
a_h_int = [
    deriv.xu deriv.xw -w0 -g*cos(cond_ini.tt0) 0 0 0;
    Zu_til  Zw_til Zq_til Ztt_til 0 0 0;
    Mu_til Mw_til Mq_til Mtt_til 0 0 0;
    0 0 1 0 0 0 0;
    0 -1 0 cond_ini.u0 0 0 0;
    1 0 0 0 0 0 0;
    0 0 0 0 1 0 0;
    ];

b_h_sf_int = [
    deriv.xde deriv.xdsp;
    Zde_til Z_dsp_til;
    Mde_til Mdsp_til;
    0 0;
    0 0;
    0 0;
    0 0;
    ];

c_h_int = eye(size(a_h_int));

d_h_sf_int = zeros(size(b_h_sf_int));

% Matrices for LQR control
% Bryson Method
% extremos adequados para u w q tt h
du_max=0.5; %m/s
dw_max=0.2; %m/s;
dq_max=5*deg; %deg/s to rad/s
dtt_max=5*deg; %deg to rad
dh_max=0.5; %m

% extremos usados na matriz R explícitos no enunciado

Q = diag([1/du_max^2 1/dw_max^2 1/dq_max^2 1/dtt_max^2 1/dh_max^2 0.1/du_max^2 0.1/dh_max^2]);
%R = diag([1/max_deflec.demin^2 1/max_deflec.spmax^2]);
R = diag([1/(5 *deg)^2 1/(5 *deg)^2]);

K_lqr = lqr(a_h_int, b_h_sf_int, Q, R);

fprintf('\n\nDamp com o LQR')
damp(a_h_int-b_h_sf_int*K_lqr)
%damp(a_h_SAE-b_h_sf*K_lqr) %caracteristica do anel fechado do lqr por cima do SAE

%definição das condicoes iniciais para o simulink
x0 = [0 0 0 0];
x0_h = [0 0 0 0 0];
finaltime = 100; % tempo de duração da simulação
StepSize = 0.01;


%h_ref = -10;
h_pt_ref = -1;
u_ref = -5;
val=sim('cvoo_g19','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

%plots
plots(val)

