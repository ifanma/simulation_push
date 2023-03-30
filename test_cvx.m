m = 20; n = 10; p = 4;
A = randn(m,n); b = randn(m,1);
C = randn(p,n); d = randn(p,1); e = rand;
cvx_begin
    variable x(n)
    minimize( norm( A * x - b, 2 ) )
    subject to
        C * x == d
        norm( x, Inf ) <= e
cvx_end

% cvx_begin 
%     {variables declaration} 
%     minimize({objective function}) or maximize{{objective function})   
%     subject to
%         {constraints} 
% cvx_end

% cvx只支持凸函数作为目标函数和约束函数
% 约束里，
% f(x)<=g(x)
% g(x)>=f(x)
% 其中要求f是凸的，g是凹的，然后
% h(x)==s(x)
% 其中要求h和s都得是仿射函数

% 原文链接：https://blog.csdn.net/qq_39942341/article/details/123010373