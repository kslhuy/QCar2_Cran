import casadi as ca
import numpy as np

"""
The KF_gradient_solver class solves for the explicit solutions of the gradients of optimal trajectories
w.r.t the tunable parameters 
"""
class Gradient_solver:
    def __init__(self , obser):
        
        self.obser = obser

        self.v_x  = ca.SX.sym('v_x',1,1)
        self.v_y   = ca.SX.sym('v_y',1,1)
        self.psi    = ca.SX.sym('psi',1,1)
        self.psi_dot    = ca.SX.sym('psi_dot',1,1)
        # self.Y    = ca.SX.sym('Y',1,1)
        # Position
        self.x_state      = ca.vertcat(self.v_x,self.v_y, self.psi, self.psi_dot)



        self.ref    = ca.SX.sym('ref',4,1)
       
        # self.x_vp   = ca.vertcat(self.v_x,self.v_y, self.psi, self.psi_dot) 

        self.Kloss = 1

        self.traj_e = self.x_state - self.ref
        
        # ------- Weight penatiy , less sensitive on v_y and psi_dot 
        Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 1,1, 1,1
        # Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 1, 0.4, 1 , 0.4
        weight      = np.array([Weight_v_x , Weight_v_y, Weight_psi,Weight_psi_dot])
        self.weight_error_traj = np.diag(weight)



        self.loss   = ca.mtimes(ca.mtimes(ca.transpose(self.traj_e), self.weight_error_traj), self.traj_e)

        #---------------- Continue
        self.A = self.obser.A_small_fix_conti

        self.C = self.obser.C_small_fix
        self.D = self.obser.D_small_fix_conti

        self.K_P = self.obser.K_P_continue


        self.K_P_3mesure = self.obser.K_P_3_continue
        self.C_3mesure = self.obser.C_3mesure_obs



        self.K_P_w_D = self.obser.K_P_continue_w_D
        # self.K_P_w_D = self.obser.K_Huy_continue

        #---------------- Discret
        self.A_discret = self.obser.A_small_fix

        self.C_discret = self.obser.C_small_fix
        self.D_discret = self.obser.D_small_fix


        self.K_P_discret_w_D = self.obser.K_P_dis_w_D


        self.Kloss  = 1  # 2.5e2

        # self.A = self.obser.A_small_fix_conti

        # self.C = self.obser.C_small_fix
        # self.K_P = self.obser.K_P_continue
        self.Ts = self.obser.constants['Ts']

    def GradientSolver_general(self, sensitiveti): 
       
        sensitiveti = ((self.A + self.K_P@self.C)@sensitiveti + np.eye(4))*self.Ts + sensitiveti
        return sensitiveti
    
    def GradientSolver_general_3mesure(self, sensitiveti): 
       
        sensitiveti = ((self.A + self.K_P_3mesure@self.C_3mesure)@sensitiveti + np.eye(4))*self.Ts + sensitiveti
        return sensitiveti
    
    def GradientSolver_general_w_D(self, sensitiveti): 
       
        sensitiveti = ((self.A + self.K_P@self.C)@sensitiveti + self.D)*self.Ts + sensitiveti
        return sensitiveti
    
    def GradientSolver_discret_general_w_D(self, sensitiveti): 
       
        sensitiveti = ((self.A_discret + self.K_P_discret_w_D@self.C_discret)@sensitiveti + self.D_discret) 
        return sensitiveti
    
   
    def loss_tracking(self, xpa, ref):
        loss_fn = ca.Function('loss', [self.x_state, self.ref], [self.loss], ['xpa0', 'ref0'], ['lossf'])
        loss_track = loss_fn(xpa0=xpa, ref0=ref)['lossf'].full()
        return loss_track
    
    def ChainRule(self, ref, x_hat, dx_df,f_hat,f_nn, weight_error_traj , lamda):

        # Track ref 
        x_hat = x_hat.reshape(-1,1)

        x_tile = x_hat - ref

        # loss1 = np.linalg.norm(y_tile, ord=2)
        loss1 = self.Kloss*(((x_tile).T @  weight_error_traj)@x_tile)

        # return dLdx ,loss_track
       
        # loss1 = np.linalg.norm(x_hat - ref, ord=2)
        # loss1 = loss1**2
        f_tile = f_nn.T -  f_hat.reshape(1,-1)
        loss2 = lamda * np.linalg.norm( f_tile, ord=2)
        # loss2 = loss2**2

        loss_track = loss1 + loss2



        dLdx = weight_error_traj@(x_tile) # analytic in paper , with the wegith matrix 
        
        dLdf_1  = np.matmul(dLdx.T, dx_df)

        dLdf_2 = lamda*f_tile  # analytic in paper
        
        sum_dLdf_dLdx =  dLdf_1 + dLdf_2
        
        
        return sum_dLdf_dLdx ,loss_track
    

    def ChainRule2(self, mesurement, x_hat, dx_df,f_hat,f_nn, weight_error_traj , lamda):

        x_hat = x_hat.reshape(-1,1)
        mesurement = mesurement.reshape(-1,1)

        y_hat = self.C@x_hat
        y_tile = y_hat - mesurement

        loss1 = self.Kloss*(((y_tile).T @  weight_error_traj)@y_tile)


        f_tile = f_nn -  f_hat

        loss2 = lamda * np.linalg.norm(f_tile, ord=2)
        
        loss_track = loss1 + loss2
        # if (lamda != 0 ):
        #     loss_track = loss1 + loss2 
        # else:
        #     loss_track = loss1
            
        dLdy_tile =  weight_error_traj@(y_tile)


        dy_tiledx = self.C # analytic in paper , with the wegith matrix 
        
        dLdx = np.matmul(dLdy_tile.T , dy_tiledx)

        dLdf_1  = np.matmul(dLdx, dx_df)
        dLdf_2 = f_nn.T - f_hat.reshape(1,-1)  # analytic in paper

        sum_dLdf_dLdx =  dLdf_1 + lamda*dLdf_2
        # sum_dLdf_dLdx =  dLdf_1 
        
        return sum_dLdf_dLdx ,loss_track
    

    def ChainRule2_test(self, mesurement, x_hat, dx_df,f_hat,f_nn, weight_error_traj , lamda):

        x_hat = x_hat.reshape(-1,1)
        mesurement = mesurement.reshape(-1,1)

        y_hat = x_hat
        y_tile = y_hat - mesurement

        loss1 = np.linalg.norm(y_tile, ord=2)
        loss_track = loss1 


        dLdy_tile =  weight_error_traj@(y_tile)


        dLdf_1  = np.matmul(dLdy_tile.T, dx_df)

        sum_dLdf_dLdx =  dLdf_1 
        # sum_dLdf_dLdx = dLdy_tile.T
        
        
        return sum_dLdf_dLdx ,loss_track
    
    def ChainRule3_test(self, mesurement, x_hat, dx_df,f_hat,f_nn, weight_error_traj , lamda):

        # Note : f_hat = f_uk True
        x_hat = x_hat.reshape(-1,1)
        mesurement = mesurement.reshape(-1,1)

        y_hat = x_hat
        y_tile = y_hat - mesurement

        # loss1 = np.linalg.norm(y_tile, ord=2)
        loss1 = self.Kloss*(((y_tile).T @  weight_error_traj)@y_tile)

        f_tile =  f_nn.T - f_hat.reshape(1,-1)

        loss2 = lamda * np.linalg.norm(f_tile, ord=2)
        
        loss_track = loss1 + loss2
        # if (lamda != 0 ):
        #     loss_track = loss1 + loss2 
        # else:
        #     loss_track = loss1

        dLdy_tile =  self.Kloss * (weight_error_traj@(y_tile))


        dLdf_1  = np.matmul(dLdy_tile.T, dx_df)
        dLdf_2 =  lamda*f_tile  # analytic in paper

        sum_dLdf_dLdx =  dLdf_1 + dLdf_2
        # sum_dLdf_dLdx = dLdy_tile.T
        
        
        return sum_dLdf_dLdx ,loss_track