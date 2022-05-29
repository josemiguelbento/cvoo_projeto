clear all
clc

g = 9.81 // m.s^-2

deg = %pi/180
kt = 0.514444444 // 1kt = 0.514444444 m/s


// Flight Condition

h = 50 // m
aa0 = 3.84*deg // deg
gg0 = 0*deg // deg
u0 = 66.1*kt //kt
flaps = 5*deg // deg
de0 = -7.44*deg // deg
da0 = 0.58*deg // deg
dr0 = -0.01*deg // deg
q0 = 0 // rad/s

w0 = u0*aa0
tt0 = aa0+gg0

X0 = [u0;w0;q0;tt0]


// Maximum Deflections

delim = [28*deg 21*deg] // deg
damax = 17*deg // deg
drmax = 23*deg // deg
flapmax = 40*deg // deg
spmax = 60*deg // deg


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



// Model Definition

A = [xu xw -w0 -g*cos(tt0);zu/(1-zwp) zw/(1-zwp) (zq+u0)/(1-zwp) -g*sin(tt0)/(1-zwp); mwp*zu/(1-zwp) mw+mwp*zw/(1-zwp) mq+mwp*(zq+u0)/(1-zwp) -mwp*g*sin(tt0)/(1-zwp);0 0 1 0]

B = [xde xdf xdsp; zde 0 0; mde+mwp*zde/(1-zwp) 0 0;0 0 0]

C = eye(A)

D = zeros(B)

sys = syslin([],A,B,C,D,X0)

[wn zz] = damp(sys)

disp(spec(A))
disp(wn)
disp(zz)

plzr(sys)
