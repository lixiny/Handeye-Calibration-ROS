clear
clc
syms qw qx qy qz
syms tx ty tz

quantx = [qw; qx; qy; qz];
translx = [0; tx; ty; tz];

syms qiw qix qiy qiz
syms tix tiy tiz

quantbg = [qiw; qix; qiy; qiz];
translbg = [0; tix; tiy; tiz ];


syms tcx tcy tcz 
sphereInCamera = [0; tcx; tcy; tcz];

Pg = M ( M(quantx, sphereInCamera), Conj(quantx)) + translx;

Pb = M( M(quantbg, Pg), Conj(quantbg)) + translbg;

Pb(1,1)
Pb(2,1)
Pb(3,1)
Pb(4,1)