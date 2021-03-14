function traj = gen_forward_fix_traj(p, ic, L)
ds = 0.1;
ns = floor(L/ds);

traj = zeros(ns, 5);

A = [  0     1     0;
       0     0     0;
       0     0    -1/p.trailerWheelbase];
B = [0;
    1/p.truckWheelbase;
    -1/p.truckWheelbase];
p.feedbackGain = lqr(-A, -B, p.Q, p.R);

% https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator
Q = diag([10 10 0]);
R = 1;
Ab = -A+B*p.feedbackGain;
F = expm(Ab)'*diag([1,1,1])*expm(Ab);
P = F;
K = zeros(ns, 3);

for ct = ns:-1:1    
    P = P + ds * (A'*P+P*A-(P*B)/R*(B'*P)+Q);
    K(ct, :) = R\(B'*P);
end

ic = ic(:);
u = -K(1,:)*ic(2:end);
u = min(u, deg2rad(25));
u = max(u, -deg2rad(25));
traj(1, :) = [ic' u];

%
x = ic(2:end);

for ct = 2:ns   
    u = -K(ct-1, :) * x;
    u = min(u, deg2rad(25));
    u = max(u, -deg2rad(25));
    x = x + ds*(A*x + B*tan(u));
    traj(ct, 1) = traj(ct-1, 1) + ds; 
    traj(ct, 2:4) = x;
    traj(ct, 5) = u;
end

end