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

fprintf('\n\nDamp com o LQR')
damp(a_h_pt_int-b_h_pt_int*K_lqr)

%definição das condicoes iniciais para o simulink
x0 = [0 0 0 0];
%x0_h = [0 0 0 0 0];
finaltime = 90; % tempo de duração da simulação
StepSize = 0.01;

h_pt_ref = -4;
u_ref = 1.5;
val=sim('cvoo_g19_servomecanismo','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

%plots
plots_ponto3(val,cond_ini);
