%tema 50 - Ximas-1 - velocidades ar e subida - seguimento de solo

clear
close all
clc

g = 9.81;

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


% %vamos definir a matriz da dinâmica tradicional, a, com entradas:
% %   x = [u; w; q; tt]
% a = [
%     deriv.xu deriv.xw -w0 -g*cos(cond_ini.tt0);
%     Zu_til  Zw_til Zq_til Ztt_til;
%     Mu_til Mw_til Mq_til Mtt_til;
%     0 0 1 0;
%     ];
% 
% %vamos definir a matriz do controlo b, com entradas de controlo:
% %   u = [de; df; dsp]
% b = [
%     deriv.xde deriv.xdf deriv.xdsp;
%     Zde_til Zdf_til Z_dsp_til;
%     Mde_til Mdf_til Mdsp_til;
%     0 0 0;
%     ];
% 
% %as matrizes c e d ficam então definidas, assumindo y = x (a saída é o 
% %próprio estado)
% c = eye(size(a));
% 
% d = zeros(size(b));



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

%% RP2--------------------------------------------------------------------
%avaliar controlabilidade e observabilidade
Co = ctrb(a_h,b_h);
k_co = rank(Co);
% como a característica da matriz controlabilidade é igual ao número de
% estados, o sistema é controlável

Ob = obsv(a_h,c_h);
k_ob = rank(Ob);
% como a característica da matriz observabilidade é igual ao número de
% estados, o sistema é observável

%definição das condicoes iniciais para o simulink
x0 = [cond_ini.u0 cond_ini.aa0*cond_ini.u0 0 cond_ini.tt0 cond_ini.h];
%T=20; % tempo de duração da simulação
%res=sim('cvoo_g19',T);

%como o problema é o fugoide vamos realimentar a velocidade (o estado que
%melhor traduz este modo) para a entrada do spoiler.
num_dsp_u = num_dsp(1,:); %1- estamos a avaliar a estabilização do estado velocidade com spoiler
sys_sp = tf(num_dsp_u,den_dsp);
rlocus(sys_sp)

% figure()
% num_de_u = num_de(1,:); %1- estamos a avaliar a estabilização do estado velocidade com elevator
% sys_e = tf(num_de_u,den_de);
% rlocus(sys_e)
