function [ C ] = C( qx )
%UNTITLED5 此处显示有关此函数的摘要
%   此处显示详细说明
C = W(qx).' * Q(qx);

% C = W(q0,q1,q2,q3).' * Q(q0,q1,q2,q3);

end

