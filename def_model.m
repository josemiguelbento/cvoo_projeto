function [cond_ini, max_deflec, inert, wing, deriv] = def_model()
    
    deg = pi/180;
    kn = 0.514444444; %1 kn = 0.514444444 m
    
    %Initial flight condition
    cond_ini.h =50;
    cond_ini.aa0 =3.84 *deg ;
    cond_ini.gg0 =0 *deg ;
    cond_ini.tt0 = cond_ini.aa0 + cond_ini.gg0;
    cond_ini.u0 =66.1 *kn ;
    cond_ini.flaps =5 *deg;
    cond_ini.de0 = -7.44 *deg ;
    cond_ini.da0 =0.58 *deg ;
    cond_ini.dr0 = -0.01 *deg ;
    
    %maximum control surfaces deflection
    max_deflec.demax = +28 *deg ;
    max_deflec.demin = -21 *deg;
    max_deflec.damax = 17 *deg ;
    max_deflec.drmax = 23 *deg ;
    max_deflec.flapmax = 40 *deg ;
    max_deflec.spmax = 60 *deg;
    
    %inertial data
    inert.m = 3236; %kg
    inert.Ix =10990; %kg .m ^2
    inert.Iy =15395; %kg .m ^2
    inert.Iz =20571; %kg .m ^2
    inert.Ixz =149; %kg .m ^1
    
    %wing data
    wing.S =87.51; %m ^2
    wing.b =12.497; %m
    wing.c =1.592; %m
    wing.aamax =14.99 *deg;
    
    %derivatives - SI or no units
    deriv.xu = -0.0822;
    deriv.xw = 0.0058;
    deriv.zu = -0.5737;
    deriv.zw = -3.2532;
    deriv.zwp = -0.0399; %falta
    deriv.zq = -3.1076; %falta
    deriv.mu = 0.0000;
    deriv.mw = -0.0576;
    deriv.mq = -0.0555;
    deriv.mwp = 0.0687;
    
    deriv.ybb = -0.0767;
    deriv.lbb = 1.7130;
    deriv.nbb = 4.0268;
    deriv.yp = 0.0040;
    deriv.lp = -1.3553;
    deriv.np = -0.0673;
    deriv.yr = 0.0126;
    deriv.lr = -0.5544;
    deriv.nr = -0.1641;
    
    deriv.xde = 0.000;
    deriv.zde = 1.182;
    deriv.mde = -5.296;
    deriv.xdf = -1.117;
    deriv.zdf = 0.000;
    deriv.mdf = 0.000;
    deriv.xdsp = -1.359;
    deriv.zdsp = 0.000;
    deriv.mdsp = 0.000;
    
    deriv.Lda = -5.957;
    deriv.Nda = 0.354;
    deriv.Ydr = -0.026;
    deriv.Ldr = 0.000;
    deriv.Ndr = -2.611;
    
end