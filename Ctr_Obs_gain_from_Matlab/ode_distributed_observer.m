function dx = ode_distributed_observer(t,x,v0,y1,y2,y3,u)
%ODE_SYSTEM ODE system for the platoon model, including the controller and observer dynamics.

% Define the system structure
% LOflag = 'off'; % Local observer: off or on
% DOflag = 'off'; % Distributed observer: off or on

global n0 N n
global tau h d
global A_tau B_tau A_h B_delta A_h_tau A_delta


% The distributed observer gain
global L1 L2 L3

global P1 P2 P3


% The output matrix
global  C1 C2 C3 Cv Cd

% The coupling gain gamma and the adjacency matrix a
global gamma a

a = [0 1 0;
     1 0 1;
     0 1 0];

% Divide the state
Delta =  x; % The collection state of all distributed observer

% The state of each distributed observer
hat_delta_1 = [eye(n), zeros(n,n), zeros(n,n)]*Delta;
hat_delta_2 = [zeros(n,n), eye(n), zeros(n,n)]*Delta;
hat_delta_3 = [zeros(n,n), zeros(n,n), eye(n)]*Delta;


%% Vehicle one

hat_y_1 = C1 * hat_delta_1 + Cv * v0 + Cd * d;
hat_u_delta_1 = u;
% % hat_u_delta_1 = get_hat_u_delta_i(hat_delta_1);

d_hat_delta1 = A_delta * hat_delta_1 + B_delta * hat_u_delta_1...
            + L1 * (y1 - hat_y_1)...
            -gamma * inv(P1) * a(1,2)*(hat_delta_1 - hat_delta_2);

%% Vehicle two

hat_u_delta_2 = u;
% hat_u_delta_2 = get_hat_u_delta_i(hat_delta_2);

d_hat_delta2 = A_delta * hat_delta_2 + B_delta * hat_u_delta_2 + ...
       L2 * (y2 - C2 * hat_delta_2 - Cv * v0 - Cd * d) - ...
       gamma * inv(P2) * (a(2,1)*(hat_delta_2 - hat_delta_1) + (a(2,3) * (hat_delta_2 - hat_delta_3)));


%% Vehicle three

hat_u_delta_3 = u;
% hat_u_delta_3 = get_hat_u_delta_i(hat_delta_3);

d_hat_delta3 = A_delta * hat_delta_3 +  B_delta * hat_u_delta_3 + ...
       L3 * (y3 - C3 * hat_delta_3 - Cv * v0 - Cd * d) - ...
       gamma * inv(P3) * (a(3,2)*(hat_delta_3 - hat_delta_2) );


d_hat_delta = [d_hat_delta1;d_hat_delta2;d_hat_delta3];

dx = [d_hat_delta]; % Concatenate the state derivatives


end

function hat_u_delta_i = get_hat_u_delta_i(hat_delta_i)
       global F1 F2 F3
       global K10 K20 K21 K30 K31 K32 


       hat_u_1 = K10 * F1 * hat_delta_i;
       hat_u_2 = K20 * F2 * hat_delta_i + K21 * (F2 - F1) * hat_delta_i;
       hat_u_3 = K30 * F3 * hat_delta_i + K31 * (F3 - F1) * hat_delta_i + K32 * (F3 - F2) * hat_delta_i;

       hat_u_delta_i = [hat_u_1;hat_u_2;hat_u_3;];

end