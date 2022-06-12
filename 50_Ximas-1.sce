//Tema 50 - Ximas-1
// Seguimento de Solo - velocidades ar e subida

clear all
clc

// Definicao da Aceleracao Gravitica
g = 9.81 // m.s^-2

// Definicao dos Fatores de Conversao
deg = %pi/180
kt = 0.514444444 // 1kt = 0.514444444 m/s


// Initial Flight Conditions
h0 = 50 // m
aa0 = 3.84*deg // rad
gg0 = 0*deg // rad
u0 = 66.1*kt // m/s
flaps0 = 5*deg // rad
de0 = -7.44*deg // rad
da0 = 0.58*deg // rad
dr0 = -0.01*deg // rad
q0 = 0 // rad/s

w0 = u0*aa0 // m/s
tt0 = aa0+gg0 // rad


// Definicao do estado inicial
X0 = [u0;w0;q0;tt0]


// Maximum Deflections
delim = [28*deg 21*deg] // rad
damax = 17*deg // rad
drmax = 23*deg // rad
dfmax = 40*deg // rad
dspmax = 60*deg // rad


// Inertial Data
m = 3236 // kg
Ix = 10990 //kg.m^2
Iy = 15395 //kg.m^2
Iz = 20571 //kg.m^2
Ixz = 149 //kg.m^2


// Wing Data
S = 87.51 // m^2
b = 12.497 // m
c = 1.592 // m
aamax = 14.99*deg // deg


// Stability Derivatives (no units / SI units)
xu = -0.0822
xw = 0.0058
zu = -0.5737
zw = -3.2532
zwp = -0.0399
zq = -3.1076
mu = 0.0000
mw = -0.0576
mq = -0.0555
mwp = 0.0687
ybb = -0.0767
lbb = 1.7130
nbb = 4.0268
yp = 0.0040
lp = -1.3553
np = -0.0673
yr = 0.0126
lr = -0.5544
nr = -0.1641
xde = 0.000
zde = 1.182
mde = -5.296
xdf = -1.117
zdf = 0.000
mdf = 0.000
xdsp = -1.359
zdsp = 0.000
mdsp = 0.000
Lda = -5.957
Nda = 0.354
Ydr = -0.026
Ldr = 0.000
Ndr = -2.611

// Definicao das Entradas Auxiliares para a Matriz da Dinâmica
Mu_til = mu + mwp*zu/(1-zwp)
Mw_til = mw + mwp*zw/(1-zwp)
Mq_til = mq + mwp/(1-zwp)*(u0+zq)
Mtt_til = -mwp*g*sin(tt0)/(1-zwp)
Zu_til = zu/(1-zwp)
Zw_til = zw/(1-zwp)
Zq_til = (zq + u0)/(1-zwp)
Ztt_til = -g*sin(tt0)/(1-zwp)
Zde_til = zde/(1-zwp)
Zdf_til = zdf/(1-zwp)
Z_dsp_til = zdsp/(1-zwp)
Mde_til = mde+mwp*zde/(1-zwp)
Mdf_til = mdf+mwp*zdf/(1-zwp)
Mdsp_til = mdsp+mwp*zdsp/(1-zwp)


// Definicao da matriz da dinâmica, A, com entradas:
//  x = [u; w; q; tt]
A = [
    xu xw -w0 -g*cos(tt0);
    Zu_til  Zw_til Zq_til Ztt_til;
    Mu_til Mw_til Mq_til Mtt_til;
    0 0 1 0;
    ];

// Definicao da matriz do controlo B, com entradas de controlo:
//  u = [de; df; dsp]
B = [
    xde xdf xdsp;
    Zde_til Zdf_til Z_dsp_til;
    Mde_til Mdf_til Mdsp_til;
    0 0 0;
    ];

// Definicao das matrizes C e D, assumindo y = x (a saída  
// é o próprio estado)
C = eye(A)
D = zeros(B)


// Criacao de sistema dinamico linear do tipo xp=Ax+Bu, com saida yp=Cx+Du
sys = syslin('c',A,B,C,D,X0)

// Calculo dos valores proprios da matriz da dinamica, A
p = spec(A)
// Calculo das frequencias naturais e dos fatores de amortecimento de cada polo
[wn zz] = damp(sys)

printf(' Pólos:')
disp(p)
printf('\n Frequências Naturais:')
disp(wn)
printf('\n Fatores de Amortecimento:')
disp(zz)


// Representacao Grafica dos polos do sistema
plzr(sys)


// Calculo dos tempos de atenuacao ou amplificacao dos modos longitudinais
p_re = real(p((real(p))~=0)); // vetor dos polos com parte real não nula

// Primeiro passo é separar os pólos do Período Curto dos pólos do Fugóide
//    Fugóide (pólos com menor parte real, em valor absoluto):
[M_fug,I_fug] = min(abs(p_re))
t_fug = log(2)/M_fug

//    Período Curto (pólos com maior parte real, em valor absoluto):
[M_pc,I_pc]=max(abs(p_re));
t_pc = log(2)/M_pc;

printf('\n\nModo Fugóide:')
if(real(p_re(I_fug))>0)
    printf('\n\tT_2 = %f', t_fug)
elseif(real(p_re(I_fug))<0)
    printf('\n\tT_{1/2} = %f', t_fug)
end

printf('\n\nModo Período Curto:')
if(real(p_re(I_pc))>0)
    printf('\n\tT_2 = %f', t_pc)
elseif(real(p_re(I_pc))<0)
    printf('\n\tT_{1/2} = %f', t_pc)
end


// ------------------- RP2 - LQR -------------------
//x = [u; w; q; tt; h; u_i; h_i]

A_h_int = [
    xu xw -w0 -g*cos(tt0) 0 0 0;
    Zu_til  Zw_til Zq_til Ztt_til 0 0 0;
    Mu_til Mw_til Mq_til Mtt_til 0 0 0;
    0 0 1 0 0 0 0;
    0 -1 0 u0 0 0 0;
    1 0 0 0 0 0 0;
    0 0 0 0 1 0 0;
    ];

B_h_sf_int = [
    xde xdsp;
    Zde_til Z_dsp_til;
    Mde_til Mdsp_til;
    0 0;
    0 0;
    0 0;
    0 0;
    ];

C_h_int = eye(A_h_int)
D_h_sf_int = zeros(B_h_sf_int)

P = syslin('c',A_h_int,B_h_sf_int,C_h_int,D_h_sf_int)


// Método-Bryson
// Escolha de Variação Paramétrica Máxima
du_max=0.5 //m/s
dw_max=0.2 //m/s;
dq_max=5*deg //deg/s to rad/s
dtt_max=5*deg //deg to rad
dh_max=0.5 //m

// Matrices para Controlo LQR
Q = diag([1/du_max^2 1/dw_max^2 1/dq_max^2 1/dtt_max^2 1/dh_max^2 0.01/du_max^2 0.01/dh_max^2])
R = diag([1/delim(2)^2 1/dspmax^2])
// Matriz dos Ganhos
K_lqr = lqr(P, Q, R)


// Calculo dos valores proprios da nova matriz da dinamica, A
p_lqr = spec(A_h_int)
// Calculo das frequencias naturais e dos fatores de amortecimento de cada polo
[wn_lqr zz_lqr] = damp(A_h_int-B_h_sf_int*K_lqr)

printf('\n\nDamp com o LQR')
printf(' Pólos:')
disp(p_lqr)
printf('\n Frequências Naturais:')
disp(wn_lqr)
printf('\n Fatores de Amortecimento:')
disp(zz_lqr)


// Definição das Condicoes Iniciais para o XCOS
x0 = [0 0 0 0]
x0_h = [0 0 0 0 0]
finaltime = 100 // tempo de duração da simulação
StepSize = 0.01


h_ref = 10
//h_ref = 5*cond_ini.u0;
u_ref = -5


// info = xcos_simulate(sim_lqr, 4)
// info = scicos_simulate(sim_lqr)
//val=sim('cvoo_g19','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize))

sim_lqr = importXcosDiagram("C:\Users\franc\OneDrive\Documentos\GitHub\cvoo_projeto\sim_lqr.zcos")

typeof(sim_lqr) //The diagram data structure

//first batch simulation with the parameters embedded in the diagram
xcos_simulate(sim_lqr, 4);



