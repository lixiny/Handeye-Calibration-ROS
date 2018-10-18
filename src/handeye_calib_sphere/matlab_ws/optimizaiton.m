clear
% argument
syms qx0 qx1 qx2 qx3;  
qx = [qx0;qx1;qx2;qx3];
syms tx1 tx2 tx3;
tx = [0;tx1;tx2;tx3]
syms tb1 tb2 tb3;
tb = [0;tb1;tb2;tb3]

% parameter
syms ti1 ti2 ti3; 
ti = [0;ti1; ti2; ti3]
syms tj1 tj2 tj3;
ticp = [0;tj1;tj2;tj3]
syms qi0 qi1 qi2 qi3;
qi = [qi0;qi1;qi2;qi3]

Di = D(ti,ticp)
Ci = C(qi)
Fi = Di * qx + W(qx) * tx + Q(qx) * Ci * tb