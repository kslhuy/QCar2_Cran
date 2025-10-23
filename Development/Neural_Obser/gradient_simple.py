import casadi as ca
import numpy as np

"""
The KF_gradient_solver class solves for the explicit solutions of the gradients of optimal trajectories
w.r.t the tunable parameters 
"""
class Gradient_solver:
    def __init__(self , obser):
        
        self.obser = obser

        # self.v_x  = ca.SX.sym('v_x',1,1)
        self.v_y   = ca.SX.sym('v_y',1,1)
        self.psi    = ca.SX.sym('psi',1,1)
        self.psi_dot    = ca.SX.sym('psi_dot',1,1)
        self.Y    = ca.SX.sym('Y',1,1)
        # Position
        self.x_state      = ca.vertcat(self.v_y, self.psi, self.psi_dot,self.Y)



        self.ref    = ca.SX.sym('ref',4,1)
       
        # self.x_vp   = ca.vertcat(self.psi,self.Y) 

        self.Kloss = 1

        self.traj_e = self.x_state - self.ref   # error between state estimate and the ref value  

        # Weight_v_x, Weight_v_y , Weight_psi   = 10,100, 1  # weight value for losss 
        # weight      = np.array([Weight_v_x , Weight_v_y, Weight_psi])

        Weight_v_y , Weight_psi,Weight_psi_dot, Weight_Y    = 1,1 ,1 , 1 # weight value for losss 
        weight      = np.array([Weight_v_y, Weight_psi,Weight_psi_dot ,Weight_Y])

        self.loss   = ca.mtimes(ca.mtimes(ca.transpose(self.traj_e), np.diag(weight)), self.traj_e)

        # self.A = self.obser.A_small_fix
        self.A = self.obser.A_small_fix_conti

        self.C = self.obser.C_small_fix
        self.K_P = self.obser.K_P_continue

        self.Ts = self.obser.constants[6]


    def GradientSolver_general(self, sensitiveti, D_out): 
       
        # sensitiveti = - np.linalg.inv((self.A + self.K_P@self.C))
        sensitiveti = ((self.A + self.K_P@self.C)@sensitiveti + np.eye(D_out))*self.Ts + sensitiveti
        return sensitiveti
    
   
    def loss_tracking(self, xpa, ref):
        loss_fn = ca.Function('loss', [self.x_state, self.ref], [self.loss], ['xpa0', 'ref0'], ['lossf'])
        loss_track = loss_fn(xpa0=xpa, ref0=ref)['lossf'].full()
        return loss_track
    
    def ChainRule(self, ref, x_hat, dx_df):

        x_hat = x_hat.reshape(-1,1)

        
        # Define the gradient of loss w.r.t state
        Ddlds = ca.jacobian(self.loss, self.x_state)
        Ddlds_fn = ca.Function('Ddlds', [self.x_state , self.ref], [Ddlds], ['xfull' , 'ref0'], ['dldsf'])
        # Initialize the parameter gradient
        # dp = np.zeros((1, self.n_para))

        loss_track = self.Kloss * self.loss_tracking(x_hat, ref)
        dLdx =   self.Kloss * Ddlds_fn(xfull =x_hat, ref0=ref)['dldsf'].full()
        
        dp  = np.matmul(dLdx, dx_df)
           
        return dp , loss_track