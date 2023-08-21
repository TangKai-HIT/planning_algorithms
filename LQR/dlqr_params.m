function [K,P,poles] = dlqr_params(A, B, Q, R, tol)
%DLQR_PARAMS solve dlqr controller feed-back gain, solution of riccati equation, feed-back system poles 
%   此处显示详细说明
P = solve_dare(A, B, Q, R, tol);
K = (R+B'*P*B)\(B'*P*A);
poles = eig(A-B*K);
end

