function [x, u, K, P, poles] = dlqr_control(A, B, Q, R, x_0, dare_tol, end_tol, maxsteps)
%DLQR_CONTROL 此处显示有关此函数的摘要
%   此处显示详细说明
[K,P,poles] = dlqr_params(A, B, Q, R, dare_tol);

x = x_0;
u = [];
x_cur = x_0;

for i=1:maxsteps
    u_cur = -K*x_cur;
    u = [u, u_cur];
    
    x_cur = A*x_cur + B*u_cur;
    x = [x, x_cur];
    
    if norm(x(end) - x(end-1)) < end_tol
        break;
    end
end

end

