%% Description:
% This file is used to solve the observer gain of the distributed observer
% for a platoon of vehicles. It uses LMI to find the observer gain
clear all

%% 
global L1 L2 L3 
global P1 P2 P3
global K10 K20 K21 K30 K31 K32 

% Define system parameters
global n0 N n m
global A_tau B_tau A_h B_delta A_h_tau A_delta
global F1 F2 F3

 
global n0 N n m

n0 = 3; % Number of states (position, velocity, acceleration)
N = 3;  % Number of vehicles in the platoon
n = n0*N;   % Total number of states
m = 2; % Number of outputs (position and velocity)
q = 1; % Number of disturbances

global tau h d
tau = 0.16; % Time constant
h = 0.3; 
d = 0.4; % Distance between vehicles

global A_tau B_tau A_h B_delta A_h_tau A_delta

A_tau = [0 1 0;
          0 0 1;
          0 0 -1/tau];
B_tau = [0; 0; 1/tau];
A_h = [0 0 h;
        0 0 0;
        0 0 0];
B_delta = blkdiag(B_tau, B_tau, B_tau);

A_h_tau = A_h + A_tau;

A_delta = [A_h_tau, zeros(n0, n0),  zeros(n0, n0);
            A_h,    A_h_tau,        zeros(n0, n0);
            A_h,    A_h,            A_h_tau];

D1 = [10;5;2];
% D1 = [0;0;0];
D = blkdiag(D1, D1, D1);

global C1 C2 C3  Cv Cd

Cf = [1 -h 0;
       0 1 0;];
Cp = [-1 0 0;
       0 0 0;];

C1 = [Cf, zeros(2, n0), zeros(2, n0)];
C2 = [Cp, Cf, zeros(2, n0), ];
C3 = [zeros(2, n0), Cp, Cf, ];

Cv = [-h;1];
Cd = [-1;0];

global F1 F2 F3  

F1 = [eye(n0), zeros(n0,n0), zeros(n0,n0)];
F2 = [zeros(n0,n0), eye(n0), zeros(n0,n0)];
F3 = [zeros(n0,n0), zeros(n0,n0), eye(n0)];

global gamma 
a = [0 1 0;
        1 0 1;
        0 1 0];

% $\bar{F}_{k} \in \bR^{n \times n}$
barF1 = [F1; zeros(n0,n); zeros(n0,n)];
barF2 = [F2; F2 - F1; zeros(n0,n)];
barF3 = [F3; F3 - F1; F3 - F2];

% F =& \begin{bmatrix}
% 			\bar{F}_{1} & \bar{F}_{2} & \cdots &
% 			\bar{F}_{k} & \cdots & \bar{F}_{N}
% 		\end{bmatrix} \in \bR^{n \times nN},\\
F = [barF1, barF2, barF3];

% \mathcal{F} =& \begin{bmatrix}
% 			\bar{F}_{1}^{\top} &
% 			 \bar{F}_{2}^{\top} &
% 			  \cdots &
% 			   \bar{F}_{N}^{\top}
% 		\end{bmatrix}^{\top} \in \bR^{nN \times n}
calF = [barF1; barF2; barF3];


global C1 C2 C3

% Check the observability matrix
C = [C1; C2; C3];
O = obsv(A_delta, C);
if rank(O) == n
    disp('The collective system is jointly observable');
else
    error('The collective system is not observable');
end

%% Communication topology
% Define the communication topology
Lap = [ 1  -1   0;
     -1   2  -1 ;
      0  -1   1 ];

r = [1, 1, 1]; % Example solution vector for testing

R = diag(r);
hatLap = R*Lap + Lap'*R;
lamda = eig(hatLap);
lamda2 = lamda(2);

iota1 = 0.5;
iota2 = 0.5;
alpha = 0.5; 
%% Solve the observer gain using LMI
yalmip('clear')

mu = sdpvar(1, 1, 'full');
% mu = 0.01;

% Define variables for distributed observer
P1 = sdpvar(n, n, 'symmetric'); % Lyapunov matrix for the first vehicle
P2 = sdpvar(n, n, 'symmetric'); % Lyapunov matrix for the second vehicle
P3 = sdpvar(n, n, 'symmetric'); % Lyapunov matrix for the third vehicle
P = blkdiag(P1, P2, P3); % in R (n*N x n*N)

X1 = sdpvar(n, m, 'full'); % Observer gain for the first vehicle
X2 = sdpvar(n, m, 'full'); % Observer gain for the second vehicle
X3 = sdpvar(n, m, 'full'); % Observer gain for the third vehicle

gamma = sdpvar(1, 1, 'full');

% Distributed observer design using LMI

% \Lambda_{i} = & \Sym{P_{i} A_{\varepsilon} - P_{i} L_{i} C_{i}} \in \bR^{n \times n}, \\
Lambda1 = A_delta'*P1 + P1*A_delta - C1'*X1' - X1*C1 ;
Lambda2 = A_delta'*P2 + P2*A_delta - C2'*X2' - X2*C2 ;
Lambda3 = A_delta'*P3 + P3*A_delta - C3'*X3' - X3*C3 ;

% \Lambda_p = & \begin{bmatrix} \Lambda_1 + I_n & \Lambda_2 + I_n & \cdots & \Lambda_N + I_n \end{bmatrix} \in \bR^{n \times nN}, \\
Lambda_p = [Lambda1 + eye(n), Lambda2 + eye(n), Lambda3 + eye(n)];

% \Lambda = & \diag{\Lambda_1, \Lambda_2, \cdots , \Lambda_N} \in  \bR^{nN \times nN}, \\
Lambda = blkdiag(Lambda1, Lambda2, Lambda3);

% \varGamma_p = & \begin{bmatrix} D^{\top}P_1 & D^{\top}P_2 & \cdots & D^{\top}P_N \end{bmatrix} \in \bR^{p \times nN}, \\
Gamma_p = [D'*P1, D'*P2, D'*P3];

% M_{c} =  \sum \Lambda_i + N I_n
Mc = Lambda1 + Lambda2 + Lambda3 + N*eye(n);

% M_{d} = \Lambda + I_{nN} - 2\gamma \lambda_{2}(\hat{\L}) I_{nN}
Md = Lambda + eye(n*N) - 2*gamma*lamda2*eye(n*N);
dim_Md = size(Md,1);

% Define the variables for  controller
Q1 = sdpvar(n0, n0, 'symmetric'); % Lyapunov matrix for the first vehicle
Q2 = sdpvar(n0, n0, 'symmetric'); % Lyapunov matrix for the second vehicle
Q3 = sdpvar(n0, n0, 'symmetric'); % Lyapunov matrix for the third vehicle

Q = blkdiag(Q1, Q2, Q3); % in R (n0*N x n0*N) in R (n x n)

Y11 = sdpvar(1, n0, 'full'); % Observer gain for the first vehicle
Y22 = sdpvar(1, n0, 'full'); % Observer gain for the second vehicle
Y21 = sdpvar(1, n0, 'full'); % Observer gain for the second vehicle
Y33 = sdpvar(1, n0, 'full'); % Observer gain for the third vehicle
Y31 = sdpvar(1, n0, 'full'); % Observer gain for the third vehicle
Y32 = sdpvar(1, n0, 'full'); % Observer gain for the third vehicle

% Y_{k} = \begin{bmatrix}
% 			Y_{k0} & Y_{k1} & \cdots & Y_{k,k-1} & 0 & \cdots & 0
% 			\end{bmatrix} \in \bR^{1 \times n},
Y1 = [Y11, zeros(1,n0), zeros(1,n0)];
Y2 = [-Y21, Y22, zeros(1,n0)];
Y3 = [-Y31, -Y32, Y33];


% Y = \begin{bmatrix}
% 			Y_{1}^{\top} &
% 			Y_{2}^{\top} &
% 			\vdots &
% 			Y_{k}^{\top} &
% 			\vdots &
% 			Y_{N}^{\top}
% 		\end{bmatrix}^{\top} \in \bR^{N \times n}, \\
Y = [Y1; Y2; Y3;];

% &\mathcal{Y} = \diag{Y_{1}, Y_{2}, \cdots, Y_{N}} \in \bR^{N \times nN}, \\
calY = blkdiag(Y1, Y2, Y3);

% % % % % Diagonal Position 
% {M}_{\varepsilon} =  \Sym{A_{\varepsilon} Q + B_{\varepsilon} \left( K \mathbf{1}_N - K \right) Q}
Me = A_delta*Q + B_delta*Y + Q*A_delta' + Y'*B_delta';

% \mathbb{M}_{\varepsilon} = \begin{bmatrix}
% 				M_{\varepsilon} & Q \\ 
% 				* & -I
% 			\end{bmatrix}
bb_Me = [Me , Q;
         Q', -eye(n)];
dim_bb_Me = size(bb_Me,1);

% \mathbb{M}_{c} = \begin{bmatrix} 
% 				M_c & -\sum P_i D \\
% 				* & -\mu^2 I_q
% 			\end{bmatrix}         
bb_Mc = [Mc, - (P1*D + P2*D + P3*D );
         -(P1*D + P2*D + P3*D)', -mu*eye(N*q)];
dim_bb_Mc = size(bb_Mc,1);

% \mathbb{D} = \begin{bmatrix}
% 				0 & D \\ 
% 				0 & 0
% 			\end{bmatrix}       
bb_D = [zeros(n,n), D;
        zeros(n,n), zeros(n,N*q)];

% \mathbb{Y}_1 = \begin{bmatrix}
% 				B_{\varepsilon} Y & B_{\varepsilon} \mathcal{Y} \\ 
% 				0 & 0
% 			\end{bmatrix}
bb_Y1 = [B_delta*Y, B_delta*calY;
         zeros(n,n), zeros(n,n*N)];

% \mathbb{P} = \begin{bmatrix}
% 				\Lambda_p^{\top} & -\varGamma_p^{\top} \\ 
% 				0 & 0 \\
% 				% 0 & 0 \\
% 				% 0 & 0 \\ 
% 				% 0 & 0
% 			\end{bmatrix},
dim1 = n*(N^2+N);
bb_P = [Lambda_p', -Gamma_p';
        zeros(dim1,n), zeros(dim1,q*N);
        zeros(dim1,n), zeros(dim1,q*N);
        zeros(n*N,n), zeros(n*N,q*N);
        zeros(n*N,n), zeros(n*N,q*N)];

% \mathbb{F}_1 = \begin{bmatrix}
% 				F^{\top} & 0 \\ 
% 				0 & \mathcal{F}^{\top} \\
% 				0 & 0 \\
% 				% 0 & 0 \\ 
% 				% 0 & 0	
% 			\end{bmatrix}
bb_F1 = [F', zeros(n*N,n*N);
         zeros(dim1,n), zeros(dim1,n*N);
         zeros(dim1,n), zeros(dim1,n*N);
         zeros(n*N,n), zeros(n*N,n*N);
         zeros(n*N,n), zeros(n*N,n*N)];

% \mathbb{F}_2 = \begin{bmatrix}
% 				0 & \mathcal{F}^{\top} \\ 
% 				0 & 0 \\
% 			\end{bmatrix}
bb_F2 = [zeros(n,n), calF';
         zeros(N*q,n),    zeros(N*q,n*N)];

% \mathbb{F}_3 = \begin{bmatrix} 
% 				\diag[N]{\mathcal{F}^{\top}} & -\left(1_N \otimes F \right)^{\top} \\
% 			\end{bmatrix}
bb_F3 = [makeBlockDiag(calF', N), -kron(ones(N,1), F)'];

% \mathbb{Y}_2 = \begin{bmatrix}
% 				\diag[N]{B_{\varepsilon}\mathcal{Y}} & \diag{B_{\varepsilon}Y}
bb_Y2 = [makeBlockDiag(B_delta*calY, N), makeBlockDiag(B_delta*Y, N)];

% \mathbb{M}_{d} = {\begin{bmatrix}
% 				M_d & 0& \mathbb{F}_2 & P & 0\\ 
% 				* & -\dfrac{1}{\iota_1} \diag[N^2+N]{Q} & 0 & 0 & \mathbb{Y}_2^{\top} \\
% 				* & * & -\iota_1 \diag[N^2+N]{Q} & 0 & 0 \\
% 				* & * & * & -\dfrac{1}{\iota_2} I_{nN} & 0 \\
% 				* & * & * & * & -\iota_2 I_{nN}
% 			\end{bmatrix}}
bb_Md = [Md,                zeros(n*N,dim1),                    bb_F3,                              P,                  zeros(n*N,n*N);
         zeros(dim1,n*N),   -1/iota1*makeBlockDiag(Q, N^2+N),   zeros(dim1,dim1),                   zeros(dim1,n*N),    bb_Y2';
         bb_F3',            zeros(dim1,dim1),                   -iota1*makeBlockDiag(Q, N^2+N),     zeros(dim1,n*N),    zeros(dim1,n*N);
         P,                 zeros(n*N,dim1),                    zeros(n*N,dim1),                    -1/iota2*eye(n*N),  zeros(n*N,n*N);
         zeros(n*N,n*N),    bb_Y2,                    zeros(n*N,dim1),                    zeros(n*N,n*N),     -iota2*eye(n*N)];

dim_bb_Md = size(bb_Md,1);
% LMI conditions

dim_alpha = n * (N+1);

M = [bb_Me,                         zeros(dim_bb_Me, dim_bb_Md),    bb_D,                           bb_Y1,                              zeros(dim_bb_Me,dim_alpha);
       zeros(dim_bb_Md, dim_bb_Me), bb_Md,                          bb_P,                           zeros(dim_bb_Md, dim_alpha),        bb_F1;
       bb_D',                       bb_P',                          bb_Mc,                          zeros(dim_bb_Mc, dim_alpha),        bb_F2;
       bb_Y1',                      zeros(dim_alpha, dim_bb_Md),    zeros(dim_alpha, dim_bb_Mc),    -1/alpha*makeBlockDiag(Q, N+1),     zeros(dim_alpha, dim_alpha);
       zeros(dim_alpha,dim_bb_Me),  bb_F1',                         bb_F2',                         zeros(dim_alpha, dim_alpha),        -alpha*makeBlockDiag(Q, N+1)];

% M = [bb_Me, bb_D; 
%     bb_D', bb_Mc;];

constraint_M = [M <= 0];
constraint_Md = [bb_Md <= 0];
constraint_P = [P1>=0, P2>=0, P3>=0];

% 为了控制 K 的量级，同时尽量保持可行性：
% 这里只给 Q 一个较小的正定下界，避免 Q 过于奇异；
% 不再强加上界，把“Q 接近 I”放在目标函数里软惩罚。
qMin = 1e-5;   % Q 的最小特征值下界，可根据需要调大/调小
constraint_Q = [Q1 >= qMin*eye(n0), Q2 >= qMin*eye(n0), Q3 >= qMin*eye(n0)];
constraint_mu = [mu>=0];
constraint_gamma = [gamma>=0];
% 
LMI = constraint_M + constraint_P + constraint_gamma + constraint_Q + constraint_mu;

% 目标函数：
% 1) 尽量减小所有 Y_ij（间接让 K_ij 和 K_i0 不至于过大）；
% 2) 轻微地把 Q_i 拉向单位阵，使得 Q 量级接近 1，从而 K ≈ Y；
% 3) 通过惩罚各个 Y_ij 之间的差异，使得各 K_ij 的大小更接近，不要差别太大。
wY   = 0;      % 惩罚 Y 的权重（控制整体大小）
wQ   = 0.01;   % 让 Q 接近 I 的权重（通常远小于 wY）
wEq  = 400;    % 惩罚各 Y_ij 之间差异的权重（控制它们彼此接近）
wM   = 0.001;

% 1) 所有 Y_ij 本身尽量小
objY = norm(Y11,'fro') + norm(Y22,'fro') + norm(Y33,'fro') + ...
       norm(Y21,'fro') + norm(Y31,'fro') + norm(Y32,'fro');

% 2) Q_i 尽量接近单位阵
objQ = norm(Q1 - eye(n0),'fro') + norm(Q2 - eye(n0),'fro') + norm(Q3 - eye(n0),'fro');

% 3) 各 Y_ij 尽量彼此接近：用它们的“平均值”作为中心，惩罚偏离
Ymean = (Y11 + Y22 + Y33) / 3;
objEq = norm(Y21 - Ymean,'fro') + 2*norm(Y31 - Ymean,'fro') + 5*norm(Y32 - Ymean,'fro');
% norm(Y11 - Ymean,'fro') + norm(Y22 - Ymean,'fro') + norm(Y33 - Ymean,'fro') + ...

obj = wY*objY + wQ*objQ + wEq*objEq + wM*mu;

ops=sdpsettings('solver','mosek');% Choose solver: sdpt3 mosek sedumi
sol=optimize(LMI,[],ops)

% Check the feasible
if sol.problem == 0
        disp(['LMI is feasible']);
        P1 = value(P1);
        P2 = value(P2);
        P3 = value(P3);

        X1 = value(X1);
        X2 = value(X2);
        X3 = value(X3);

        L1 = inv(P1)*X1;
        L2 = inv(P2)*X2;
        L3 = inv(P3)*X3;

        Q1 = value(Q1);
        Q2 = value(Q2);
        Q3 = value(Q3);

        Y11 = value(Y11);
        Y22 = value(Y22);
        Y21 = value(Y21);
        Y33 = value(Y33);
        Y31 = value(Y31);
        Y32 = value(Y32);
  
        gamma = value(gamma);
        mu = value(mu);

        % &K_{ij} = Y_{ij} Q_{j}^{-1}, \quad \forall i \in \V, j \in \{1,\ \cdots ,\ i-1\}, \\ 
        % &K_{i0} = Y_{ii}Q_{i}^{-1} - \sum_{k=1}^{i-1}K_{ik}, \quad \forall i\in \V, \\
        K10 = Y11*inv(Q1);
        K21 = Y21*inv(Q1);
        K20 = Y22*inv(Q2) - K21;
        K31 = Y31*inv(Q1);
        K32 = Y32*inv(Q2);
        K30 = Y33*inv(Q3) - K31 - K32;

    else
        error('LMI is not feasible!');
    end
