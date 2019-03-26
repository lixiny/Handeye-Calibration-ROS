function [ M ] = M( q1, q2 )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
sa = q1(1,1);
xa = q1(2,1);
ya = q1(3,1);
za = q1(4,1);
sb = q2(1,1);
xb = q2(2,1);
yb = q2(3,1);
zb = q2(4,1);

M = [sa * sb - xa * xb - ya * yb - za * zb;
     sa * xb + xa * sb + ya * zb - za * yb;
     sa * yb - xa * zb + ya * sb + za * xb;
     sa * zb + xa * yb - ya * xb + za * sb;
    ];

end

