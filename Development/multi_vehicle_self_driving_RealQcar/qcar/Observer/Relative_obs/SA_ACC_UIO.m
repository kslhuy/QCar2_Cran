% 
%% 
%%% Here Code work 
%% 

clear all
close all
clc

Ts = 0.01;
t = 0:Ts:20;
Nt = length(t);
I3 = eye(3);
%% System and Controller parameters
tau = 0.4;
li = 5;
Li = 8;
h = 0.5;
k1 = -0.8;
k2 = 2.5;
k3 = (1-h*k1*k2);
ai_1 = 3*sin(2*pi/10*(t));
%% Continuous-time model of the following vehicle(usync as input)
Ai = [0 1 0;0 0 1;0 0 -1/tau];
Bi = [0;0;1/tau];
Ci = zeros(1,3);
Di = zeros(1,1);
% Descritization of the c-state space model
Aid = (eye(length(Ai))+Ts*Ai);
Bid = Ts*Bi;
Cid = Ci;
Did = Di;
%% Continuous-time model of the preceeding vehicle(acceleration as input
Ai_1 = [0 1 0;0 0 0;0 0 0];
Bi_1 = [0;1;0];
Ci_1 = zeros(1,3);
Di_1 = zeros(1,1);
% discretization of the continuous model of the preceeding vehicle
Ai_1d = (eye(length(Ai_1))+Ts*Ai_1);
Bi_1d = Ts*Bi_1;
Ci_1d = Ci_1;
Di_1d = Di_1;
%% Continuous-time SA-ACC system
Ac = [0 1 0;0 0 1;-k2/(tau*h) -k3/(tau*h) (k1-k3)/(tau)];
Bc = [0;0;k2/tau];
Fc = [0;0;k3/tau];
Wc = [0;0;(k3-k1)/tau];
deltac = [0;0;k2*(Li-li)/(tau*h)];
Cc = [1 0 0;0 1 0];
%% Discretization
Ad = (eye(length(Ac))+Ts*Ac);
Bd = Ts*Bc;
Fd = Ts*Fc;
Wd = Ts*Wc;
deltad = Ts*deltac;
Cd = Cc;

%% Verifying rank condition
disp('Before transformation')
if rank(Cd*Fd)<rank(Fd)
    disp('Unknown input observer does not exist')
else
    disp('Unknown input observer exists')
end
%% Descriptor form
E = [eye(length(Ad)) Wd];
Ae = [Ad zeros(3,1)];
Be = Bd;
Fe = Fd;
deltae = deltad;
Cbar = Cd*Ad;
Ce = [Cd*Ad zeros(2,1)];

E_w = [1 ; 0 ; 0];
D_w = [1 ; 0];
% Rank verification
% disp('After transformation')
% if rank(Cbar*Wd)<rank(Wd)
%     disp('Unknown input observer does not exist')
% else
%     disp('Unknown input observer exists')
% end
% check if rank([E Ce]') is full
% if rank([E;Ce])==4
%     disp('[E;Ce] is invertible')
% else
%     disp('[E;Ce] is singular')
% end
%% Unknown input observer for the descriptor system
g_invEC = inv([E;Ce]'*[E;Ce])*[E;Ce]';
Pz = g_invEC(1:4,1:3);
Qz = g_invEC(:,4:5);
M = Pz*Be;
G = Pz*Fe;
% Observer gain computation
p = 2;          % Number of outputs
Az = Pz*Ae;
D_z = Qz*D_w;
xi_z = Pz*E_w;
nx = length(Az);
% Check the detectability of the pair (Pz*Ae,Ce)
if rank(obsv(Pz*Ae,Ce))<=length(Pz*Ae) || abs(eig(Pz*Ae))<eye(4)
    x = length(Pz*Ae)-rank(obsv(Pz*Ae,Ce));
    x1 =  'The pair (Pz*Ae,Ce) is detectable and the number of unobservable modes is %4.2f';
    fprintf(x1,x);   
else
    disp('System is not detectable')
end

P = sdpvar(nx,nx);
Z = sdpvar(p,nx);
S = sdpvar(p,p);
alpha = 0.5;
% LMI = [Az'*P*Az-Az'*X*Ce-Ce'*X'*Az-P,  X*Ce; Ce'*X' , -P ];
% LMI2 = [-alpha*eye(p) X';X -P];

% LMI = [-alpha*P, Az'*P - Ce'*X';
%         P*Az - X*Ce , -P];

sup_LMI = [(D_w'*Z - xi_z'*P); D_z'*P];
LMI = [-alpha*P             , zeros(4,2)         , Az'*P - Ce'*Z;
        zeros(4,2)'          ,-S         , sup_LMI   
        (Az'*P - Ce'*Z)'   ,sup_LMI'   , -P];
pb = [P>=0];
pb = pb+[S>=0]+[LMI<=0];
% pb = pb+[LMI2<=0];
solvesdp(pb);
Ps = double(P);
Xs = double(Z);
Ss = double(S);
K = inv(Ps)*Xs'
N = Az-K*Ce;
% poles = eig(N)
L = K+N*Qz


%% System simulation
x_p0 = [10 10 0]; %Preceding car
x_f0 = [0 0 0];   % Following car
psi0 = [x_p0(1)-x_f0(1)-li, x_p0(2)-x_f0(2), 0];
w(:,1) = [0;0;0;0]';
w(:,2) = w(:,1);
w(:,3) = w(:,1);
xp(:,1) = x_p0;
xf(:,1) = x_f0;
psi(:,1) = psi0;
ksi_hat(:,1) = w(:,1)+Qz*Cd*psi(:,1);
ksi_hat(:,2) = ksi_hat(:,1);
ksi_hat(:,3) = ksi_hat(:,1);
t0 = 0;
t(1) = t0;
ai_1(1) = 3*sin(2*pi/10*t(1));
fc(1) = 0;

for i=1:length(t)
    
    ai_1(i) = 3*sin(2*pi/10*t(i));
    
    fc(i) = attack_fuction(t(i), ai_1(i));
    
    mu(i) = ai_1(i)+fc(i);
    
    noise = noise_dist(i*Ts);
    % Preceeding vehicle : xi
    xp(:,i+1) = Ai_1d*xp(:,i)+Bi_1d*ai_1(i);
    % % SAACC Model : Error dynamics
    psi(:,i+1) = Ad*psi(:,i)+Bd*xf(2,i)+Fd*mu(i)-Wd*fc(i)+deltad + E_w*noise;
    y(:,i) = Cd*psi(:,i) + D_w*noise;
%     Unknown input linear descriptor observer
    if i<3
        ksi_hat(:,i) = w(:,i)+Qz*y(:,i);
    else
        w(:,i+1) = N*w(:,i)+L*y(:,i)+M*xf(2,i-1)+G*mu(i-1)+Pz*deltae;
        
        ksi_hat(:,i) = w(:,i)+Qz*y(:,i);
    end

    %% Control law

    usync(i) = h*k1*k2*mu(i)-k2*xf(2,i)-(k1+h*k1*k2)*(ksi_hat(3,i))+...
            k3/h*(xp(2,i)-xf(2,i))+k2/h*(psi(1,i))-k2/h*(Li-li)-(k1+h*k1*k2)*sat(ksi_hat(4,i),20,-20);
        
%     if (i*Ts > 2 && abs(ksi_hat(4,i)) > 0)
%         usync = h*k1*k2*mu(i)-k2*xf(2,i)-(k1+h*k1*k2)*(ksi_hat(3,i))+...
%             k3/h*(xp(2,i)-xf(2,i))+k2/h*(psi(1,i))-k2/h*(Li-li)-(k1+h*k1*k2)*sat(ksi_hat(4,i),20,-20);
%     else 
%         usync(i) = -k1*mu+(k1+h*k1*k2)*xf(3,i) ...
%                        -1/h*(1-h*k1*k2)*(-xp(2,i)+xf(2,i))-k2/h*(-xp(1,i)+xf(1,i))-k2*xf(2,i); % original
%     end

    xf(:,i+1) = Aid*xf(:,i)+Bid*usync(i);
end

%% Check Eq (26)------------------------
% Compute eigenvalues
eigenvalues_P = eig(Ps);
eigenvalues_S = eig(Ss);
% Find maximum and minimum eigenvalues
maxEigenvalue_P = max(eigenvalues_P);
minEigenvalue_P = min(eigenvalues_P);

maxEigenvalue_S = max(eigenvalues_S);

xi = [psi0 , 0];
% alpha = 0.9
bounds_detal = sqrt(maxEigenvalue_P / minEigenvalue_P ) *xi*alpha^1000 + sqrt(maxEigenvalue_S / (1 - alpha)*minEigenvalue_P )*0.028

ksi = [psi(:,1:end-1) ; fc(1:end)];
error_ksi = ksi_hat - ksi;
norm(mean(error_ksi,2))

%% Results Plot NEW  (Only error between State and Estimation)
figsize = [10,4.5];
linewidth = 1;

% space_left = 1; % left position of the figure window
% 
% spacing = 1; % defaut constant spacing between 2 subplot
% add_sup_spacing = 0; % defaut constant spacing between 2 subplot

fontsize = 8;

figure(1)


% set(gcf,'color','w');
plot(t, ksi_hat(1,:) - psi(1,1:end-1),'r','LineWidth',linewidth); hold on
% plot(t, ksi_hat(2,:) - psi(2,1:end-1) ,'b-','LineWidth',1); hold on;
plot(t, ksi_hat(3,:) - psi(3,1:end-1) ,'g-','LineWidth',1); hold on;
plot(t, ksi_hat(2,:) - psi(2,1:end-1) ,'b-','LineWidth',1); hold on;

set(gcf,'unit','centimeters','position',[10,1,figsize])
set(gca,'FontSize',fontsize)
grid on;
% 
d1 = legend('$\tilde{\xi}_{{\delta}_i}$','$$\tilde{\xi}_{\ddot{\delta}_i}$$' ,'$$\tilde{\xi}_{\dot{\delta}_i}$$'  );
d1.Interpreter = "latex";
% d1.Location = "best";
d1.Orientation= "horizontal";

d1.FontSize = fontsize;
xlabel('Time (s)')

ylim([-0.6 0.6]);


%%------------------
figsize = [10,3*2];
linewidth = 1;

space_left = 1; % left position of the figure window

spacing = 1; % defaut constant spacing between 2 subplot
add_sup_spacing = 0; % defaut constant spacing between 2 subplot

% %% NEW for case attack 1 -------------------- 
% figure(2)
% 
% subwidth = figsize(1);
% subheight= figsize(2)/2;
% 
% axissize = [subwidth , subheight];
% 
% fontsize = 8;
% 
% % set(gcf,'color','w');
% rgb1 = [0.4660 0.6740 0.1880];
% rgb2 = [0 0.4470 0.7410];
% 
% subplot(2,1,1)
% 
% plot(t, ksi_hat(4,:),'b--','LineWidth',linewidth);grid on
% hold on
% 
% plot(t, fc,'Color',rgb1,'LineWidth',linewidth);grid on
% 
% set(gcf,'unit','centimeters','position',[10,1,figsize])
% 
% set(gca,'unit','centimeters','position',[space_left,spacing + subheight + add_sup_spacing ,axissize - [1,1]])
% 
% % xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% % ylabel('$$f_{c}$$','Interpreter','LaTeX','Fontsize',12);
% d4 = legend('${\hat{f}^c_{k-2}}$','$$f^{c}$$');
% d4.Interpreter = "latex";
% d4.FontSize = fontsize;
% d41 = title('Cyber-attack and its estimation');
% d41.Interpreter = "latex";
% d41.FontSize = fontsize;
% ylim([-8 8 ]);
% 
% 
% subplot(2,1,2)
% 
% plot(t, ksi_hat(4,:),'b','LineWidth',linewidth);grid on
% hold on
% plot(t, fc,'Color',rgb1,'LineWidth',linewidth);grid on
% 
% 
% % plot(t,fc(1:end-2) - ksi_hat(4,:), 'b-','LineWidth',linewidth);
% % hold on 
% % plot(mu.Time,mu.Data,'r','LineWidth',linewidth);
% 
% set(gca,'unit','centimeters','position',[space_left,spacing + add_sup_spacing ,axissize - [1,1]])
% ylim([-5 5 ]);
% xlim([6 6.5 ]);
% 
% 
% xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',fontsize);
% % d51 = title('Inter-Vehicle Comunication');
% % d51.Interpreter = "latex";
% % d51.FontSize = fontsize;
% 
% % d5 = legend('$a_{i-1}$' ,'$\mu$');
% d5 = legend('${\hat{f}^c_{k-2}}$','$$f^{c}$$');
% d5.Interpreter = "latex";
% grid on
% 
% 
% % ax = axes;
% % set(ax,'units','centimeters','position',[space_left,spacing + subheight + add_sup_spacing,[2,1]])
% % grid on
% % plot(fc.Time,fc.Data,'Color',rgb1,'LineWidth',linewidth);
% % hold on
% % plot(fc.Time,fc_est.Data,'b--','LineWidth',linewidth);grid on
% % % xlabel('Time [sec]', 'Interpreter', 'LaTeX', 'Fontsize', 10);
% % % ylabel('$$f_{c}$$', 'Interpreter', 'LaTeX', 'Fontsize', 10);
% % set(ax,'xlim',[9.98,10.04],'ylim',[11,13])
% 
% 










%% NEW for case attack 2
figure(2)

subwidth = figsize(1);
subheight= figsize(2)/2;

axissize = [subwidth , subheight];

fontsize = 8;

% set(gcf,'color','w');
rgb1 = [0.4660 0.6740 0.1880];
rgb2 = [0 0.4470 0.7410];

subplot(2,1,1)

plot(t, ksi_hat(4,:),'b--','LineWidth',linewidth);grid on
hold on

plot(t, fc,'Color',rgb1,'LineWidth',linewidth);grid on

set(gcf,'unit','centimeters','position',[10,1,figsize])

set(gca,'unit','centimeters','position',[space_left,spacing + subheight + add_sup_spacing ,axissize - [1,1]])

% xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel('$$f_{c}$$','Interpreter','LaTeX','Fontsize',12);
d4 = legend('${\hat{f}^c_{k-2}}$','$$f^{c}$$');
d4.Interpreter = "latex";
d4.FontSize = fontsize;
% d41 = title('Cyber-attack and its estimation');
% d41.Interpreter = "latex";
% d41.FontSize = fontsize;
% ylim([-10 25 ]);
ylim([-7 7 ]);




ax = axes;
set(ax,'units','centimeters','position',[space_left,spacing + subheight + add_sup_spacing,[2,1]])
grid on
plot(t, ksi_hat(4,:),'b--','LineWidth',linewidth);grid on
hold on
plot(t, fc,'Color',rgb1,'LineWidth',linewidth);grid on
% xlabel('Time [sec]', 'Interpreter', 'LaTeX', 'Fontsize', 10);
% ylabel('$$f_{c}$$', 'Interpreter', 'LaTeX', 'Fontsize', 10);
set(ax,'xlim',[9.98,10.04], 'ylim',[-1,14])

subplot(2,1,2)


fc = [0, 0 ,fc];
plot(t,ksi_hat(4,:) - fc(1:end-2)  , 'r','LineWidth',1);
plot(t , error_ksi(4,:))
set(gca,'unit','centimeters','position',[space_left,spacing + add_sup_spacing ,axissize - [1,1]])
ylim([-3 3 ]);
% xlim([6 6.5 ]);


xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',fontsize);
% d51 = title('Inter-Vehicle Comunication');
% d51.Interpreter = "latex";
% d51.FontSize = fontsize;

% d5 = legend('$a_{i-1}$' ,'$\mu$');

d5 = legend('$\tilde{\xi}_{f^{c}_{k-2}} $');
d5.Interpreter = "latex";
grid on





% %% Results Plot OLD --------------------------
% figure(1)
% subplot(3,1,1)
% set(gcf,'color','w');
% plot(t,ksi_hat(1,:),'r',t,psi(1,1:end-1),'b--','LineWidth',1.5);grid on
% xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel('$$\delta_i$$','Interpreter','LaTeX','Fontsize',12);
% d1 = legend('$$\hat{\delta_i}$$','$$\delta_i$$');
% d1.Interpreter = "latex";
% d1.FontSize = 13;
% d11 = title('Relative position between host and preceding vehicle and its estimation');
% d11.Interpreter = "latex";
% d11.FontSize = 10;
% 
% ax = axes;
% set(ax,'units','normalized','position',[0.2,0.2,0.2,0.2])
% box(ax,'on')
% plot(t,ksi_hat(1,:),'r',t,psi(1,1:end-1),'b--','LineWidth',1.5,'parent',ax);grid on
% plot(t,ksi_hat(1,:),'r',t,psi(1,1:end-1),'b--','LineWidth',1.5,'parent',ax);grid on
% 
% set(ax,'xlim',[0.0,0.5],'ylim',[5,18])
% % ylim([-10 20]);
% 
% subplot(3,1,2)
% set(gcf,'color','w');
% plot(t,ksi_hat(2,:),'r',t,psi(2,1:end-1),'b--','LineWidth',1.5);grid on
% xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel('\boldmath$\dot{\delta}_i$','Interpreter','LaTeX','Fontsize',12);
% d2 = legend('$\dot{\hat{\delta}_i}$','$$\boldmath{\dot{\delta}_i}$$');
% d2.Interpreter = "latex";
% d2.FontSize = 13;
% d21 = title('Relative velocity between host and preceding vehicle and its estimation');
% d21.Interpreter = "latex";
% d21.FontSize = 10;
% 
% ax = axes;
% set(ax,'units','normalized','position',[0.2,0.2,0.2,0.2])
% box(ax,'on')
% plot(t,ksi_hat(2,:),'r',t,psi(2,1:end-1),'b--','LineWidth',1.5);grid on
% set(ax,'xlim',[0.0,0.05],'ylim',[-5,12])
% % ylim([-10 20]);
% 
% subplot(3,1,3)
% set(gcf,'color','w');
% plot(t,ksi_hat(3,:),'r',t,psi(3,1:end-1),'b--','LineWidth',1.5);grid on
% xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel('\boldmath$\ddot{\delta}_i$','Interpreter','LaTeX','Fontsize',12);
% d3 = legend('$$\ddot{\hat{\delta}_i}$$','$$\boldmath{\ddot{\delta}_i}$$');
% d3.Interpreter = "latex";
% d3.FontSize = 13;
% d31 = title('Relative acceleration between host and preceding vehicle and its estimation');
% d31.Interpreter = "latex";
% d31.FontSize = 10;
% ylim([-20 10]);
% 
% ax = axes;
% set(ax,'units','normalized','position',[0.2,0.2,0.2,0.2])
% box(ax,'on')
% plot(t,ksi_hat(3,:),'r',t,psi(3,1:end-1),'b--','LineWidth',1.5);grid on
% set(ax,'xlim',[0.0,0.25],'ylim',[-20,10])
% 
% figure(2)
% set(gcf,'color','w');
% rgb1 = [0.4660 0.6740 0.1880];
% rgb2 = [0 0.4470 0.7410];
% 
% plot(t,ksi_hat(4,:),'Color',rgb1,'LineWidth',1.5);grid on
% hold on
% plot(t,fc,'b--','LineWidth',1.5);grid on
% xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel('$$f_{c}$$','Interpreter','LaTeX','Fontsize',12);
% d4 = legend('${\hat{f}_c}$','$$f_{c}$$');
% d4.Interpreter = "latex";
% d4.FontSize = 13;
% d41 = title('Cyber-attack and its estimation');
% d41.Interpreter = "latex";
% d41.FontSize = 10;
% ylim([-15 15 ]);
% 
% ax = axes;
% set(ax,'units','normalized','position',[0.2,0.2,0.2,0.2])
% box(ax,'on')
% plot(t,ksi_hat(4,:),'Color',rgb1,'LineWidth',1.5);grid on
% hold on
% plot(t,fc,'b--','LineWidth',1.5);grid on
% set(ax,'xlim',[3.98,4.2],'ylim',[-12,12])

%------------------
% figure(3)
% set(gcf,'color','w');
% plot(t,ai_1,'g','LineWidth',2);
% hold on;
% 
% 

%---------------------------
% figure(3)
% set(gcf,'color','w');
% plot(t,y(1,:),'g','LineWidth',2);
% hold on;
% plot(t,-(xf(1,2:668)-xp(1,2:668)-li),'k','LineWidth',2)
% grid on
% xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel({'Inter-vehicle communication'},'Interpreter','LaTeX','FontSize',12)
% d5 = legend('a_{i-1}','\mu','Interpreter','LaTex');
% d5.FontSize = 13;
% d51 = title('True and corrputed transmitted acceleration and its estimation');
% d51.Interpreter = "latex";
% d51.FontSize = 10;

% -------------------------------% figure(3)
% set(gcf,'color','w');
% plot(t,ai_1,t,mu,'k','LineWidth',2);grid on
% xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel({'Inter-vehicle communication'},'Interpreter','LaTeX','FontSize',12)
% d5 = legend('a_{i-1}','\mu','Interpreter','LaTex');
% d5.FontSize = 13;
% d51 = title('True and corrputed transmitted acceleration and its estimation');
% d51.Interpreter = "latex";
% d51.FontSize = 10;
