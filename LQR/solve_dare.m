function P_new = solve_dare(A, B, Q, R, tol)
%SOLVE_DARE solve discrete Riccati equation: find fixed point P
%   Detailed explanation goes here
P_old = Q;
P_new = Q;

while true
   P_new = Q + A'*P_old*A - A'*P_old*B*(R+B'*P_old*B)\B'*P_old*A; 

   if norm(P_new-P_old, "fro")< tol^2
       break;
   end
    
   P_old = P_new;
end

end