function [ D ] = D( ti,tj )
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
Wti = W(ti);
Qtj = Q(tj);

D = Wti - Qtj;

end

