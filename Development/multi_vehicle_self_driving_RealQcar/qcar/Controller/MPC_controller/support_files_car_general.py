'''
LICENSE AGREEMENT

In relation to this Python file:

1. Copyright of this Python file is owned by the author: Mark Misin
2. This Python code can be freely used and distributed
3. The copyright label in this Python file such as

copyright=ax_main.text(x,y,'© Mark Misin Engineering',size=z)
that indicate that the Copyright is owned by Mark Misin MUST NOT be removed.

WARRANTY DISCLAIMER!

This Python file comes with absolutely NO WARRANTY! In no event can the author
of this Python file be held responsible for whatever happens in relation to this Python file.
For example, if there is a bug in the code and because of that a project, invention,
or anything else it was used for fails - the author is NOT RESPONSIBLE!
'''

import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
import scipy.integrate

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints


class SupportFilesCar:
    ''' The following functions interact with the main file'''

    def __init__(self , trajec_number , ver = 1 ):
        ''' Load the constants that do not change'''

        # Constants
        g=9.81
        m=1500
        Iz=3000

        Cf_true=38000
        Cr_true=66000

        # Cf_true = Cf_1 +  Cf_nonlinear
        Cf_1 = 37000
        Cr_1 = 65000

        Cf_nonlinear = 1000
        Cr_nonlinear = 1000

        lf=2
        lr=3
        mju=0.02 # friction coefficient
        Ts=0.02

        # ------------- Faux parameter
        Cf_faux=30000
        Cr_faux=60000

        lf_faux=2.1
        lr_faux=3.5

        mju_faux=0.03 # friction coefficient
        # ------------- Faux parameter

        ####################### Lateral control #################################

        outputs=4 # number of outputs
        inputs=2 # number of inputs
        hz = 10 # horizon period

        trajectory=trajec_number # Choose 1, 2 or 3, nothing else
        version = ver # This is only for trajectory 3 (Choose 1 or 2) , Version 2 is longger in time

       
        
        # Matrix weights for the cost function (They must be diagonal)

        if trajectory==3 and version==2:
            # Weights for trajectory 3, version 2
            Q=np.matrix('10 0 0 0;0 1000 0 0;0 0 10 0;0 0 0 10') # weights for outputs (all samples, except the last one)
            S=np.matrix('10 0 0 0;0 1000 0 0;0 0 10 0;0 0 0 10') # weights for the final horizon period outputs
            R=np.matrix('100 0;0 1') # weights for inputs
        elif trajectory==3:
            # Weights for trajectory 3, version 1
            Q=np.matrix('100 0 0 0;0 100 0 0;0 0 100 0;0 0 0 10') # weights for outputs (all samples, except the last one)
            S=np.matrix('100 0 0 0;0 1000 0 0;0 0 100 0;0 0 0 1000') # weights for the final horizon period outputs
            R=np.matrix('100 0;0 1') # weights for inputs
        else:
            # Weights for trajectories 1 & 2
            Q=np.matrix('1 0 0 0;0 200 0 0;0 0 50 0;0 0 0 50') # weights for outputs (all samples, except the last one)
            S=np.matrix('1 0 0 0;0 200 0 0;0 0 50 0;0 0 0 50') # weights for the final horizon period outputs
            R=np.matrix('100 0;0 1') # weights for inputs

        # Please do not modify the time_length!
        delay=0
        if trajectory==1:
            time_length = 60.
            x_lim=1000
            y_lim=1000
        elif trajectory ==2:
            time_length = 140.
            x_lim=1000
            y_lim=1000
        elif trajectory == 3:
            if version==1:
                x_lim=170
                y_lim=160
            else:
                x_lim=170*version
                y_lim=160*version
            first_section=14
            other_sections=14
            time_length=first_section+other_sections*10
            delay=np.zeros(12)
            for dly in range(1,len(delay)):
                delay[dly]=first_section+(dly-1)*other_sections
            # print(delay)
            # exit()
        else:
            print("trajectory: 1,2 or 3; version: 1 or 2")

        self.constants={'g':g,'m':m,'Iz':Iz,'Cf':Cf_true,'Cr':Cr_true,'Cf_1':Cf_1,'Cr_1':Cr_1,'Cf_nonlinear':Cf_nonlinear,'Cr_nonlinear':Cr_nonlinear,'lf':lf,'lr':lr,\
                        'Ts':Ts,'mju':mju,'Q':Q,'S':S,'R':R,'outputs':outputs,'inputs':inputs,\
                        'hz':hz,'delay':delay,'time_length':time_length,'trajectory':trajectory,\
                        'version':version,'x_lim':x_lim,'y_lim':y_lim}
        # exit()

        self.constants_faux = {'Cf_faux':Cf_faux,'Cr_faux':Cr_faux,'lf_faux':lf_faux,'lr_faux':lr_faux,'mju_faux':mju_faux }




        ### Observer Matrix fix

        state00 = np.array([4 , 0.0 , 3 ])
        state00_2 = np.array([4 , 0.0 , 3 ])


        delta00 = 0

        self.Cd_small=np.matrix('1 0 0 0 0 0 ;\
                                0 0 1 0 0 0 ')
        

        self.Cd_small_3mesure=np.matrix('1 0 0 0 0 0 ;\
                                        0 0 1 0 0 0;\
                                        0 0 0 1 0 0 ')
        

        self.C_3mesure_obs = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        self.Cd_Full=np.matrix('1 0 0 0 0 0 ;\
                                0 1 0 0 0 0 ;\
                                0 0 1 0 0 0 ;\
                                0 0 0 1 0 0 ')
        

        self.D_fix=np.matrix('  0.02 0 0 0  ;\
                                0 1 0 0  ;\
                                0 0 0.02 0  ;\
                                0 0 0 1  ')
        


        # ------------- Discret Gain 
        self.A_small_fix, self.B_small_fix, self.C_small_fix, self.D_small_fix = self.matrix_state_space_simple_observer_discret(state00 , delta00 , 0 )
        self.A_small_fix_2, self.B_small_fix_2, self.C_small_fix, self.D_small_fix = self.matrix_state_space_simple_observer_discret(state00_2 , delta00 , 0 )

        self.Gain_observer_discret()

        
        # ------------- Continue Gain (  )
        self.A_small_fix_conti, self.B_small_fix_conti, self.C_small_fix_conti, self.D_small_fix_conti = self.matrix_state_space_simple_observer(state00 , delta00 , 0 )

        self.Gain_observer_continu()


        # Not use this matrix
        self.B_d_pseudo_inverse = np.linalg.inv((self.B_small_fix).T @ self.B_small_fix) @ (self.B_small_fix).T

        return None

    
    def trajectory_generator(self,t):
        '''This method creates the trajectory for a car to follow'''

        Ts=self.constants['Ts']
        trajectory=self.constants['trajectory']
        x_lim=self.constants['x_lim']
        y_lim=self.constants['y_lim']

        # Define trajectories
        if trajectory==1:
            X=15*t
            Y=750/900**2*X**2+250

            # # Plot the world
            # plt.plot(X,Y,'b',linewidth=2,label='The trajectory')
            # plt.xlabel('X-position [m]',fontsize=15)
            # plt.ylabel('Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,x_lim)
            # plt.ylim(0,y_lim)
            # plt.xticks(np.arange(0,x_lim+1,int(x_lim/10)))
            # plt.yticks(np.arange(0,y_lim+1,int(y_lim/10)))
            # plt.show()
            #
            # plt.plot(t,X,'b',linewidth=2,label='X ref')
            # plt.plot(t,Y,'r',linewidth=2,label='Y ref')
            # plt.xlabel('t-position [s]',fontsize=15)
            # plt.ylabel('X,Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,t[-1])
            # plt.show()
            # # exit()

        elif trajectory==2:

            X1=15*t[0:int(40/Ts+1)]
            Y1=50*np.sin(2*np.pi*0.75/40*t[0:int(40/Ts+1)])+250

            X2=300*np.cos(2*np.pi*0.5/60*(t[int(40/Ts+1):int(100/Ts+1)]-40)-np.pi/2)+600
            Y2=300*np.sin(2*np.pi*0.5/60*(t[int(40/Ts+1):int(100/Ts+1)]-40)-np.pi/2)+500

            X3=600-15*(t[int(100/Ts+1):int(140/Ts+1)]-100)
            Y3=50*np.cos(2*np.pi*0.75/40*(t[int(100/Ts+1):int(140/Ts+1)]-100))+750

            X=np.concatenate((X1,X2),axis=0)
            Y=np.concatenate((Y1,Y2),axis=0)

            X=np.concatenate((X,X3),axis=0)
            Y=np.concatenate((Y,Y3),axis=0)

            # # Plot the world
            # plt.plot(X,Y,'b',linewidth=2,label='The trajectory')
            # plt.xlabel('X-position [m]',fontsize=15)
            # plt.ylabel('Y-position [m]',fontsize=15)
            # plt.xlim(0,x_lim)
            # plt.ylim(0,y_lim)
            # plt.xticks(np.arange(0,x_lim+1,int(x_lim/10)))
            # plt.yticks(np.arange(0,y_lim+1,int(y_lim/10)))
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.show()
            #
            # plt.plot(t,X,'b',linewidth=2,label='X ref')
            # plt.plot(t,Y,'r',linewidth=2,label='Y ref')
            # plt.xlabel('t-position [s]',fontsize=15)
            # plt.ylabel('X,Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,t[-1])
            # plt.show()
            # # exit()

        else:

            # Attention! Do not let x_dot become 0 m/s. It is in the denominator.
            delay=self.constants['delay']
            version=self.constants['version']

            # X & Y levels
            f_x=np.array([0,60,110,140,160,110,40,10,40,70,110,150])*version
            f_y=np.array([40,20,20,60,100,140,140,80,60,60,90,90])*version

            # X & Y derivatives
            f_x_dot=np.array([2,1,1,1,0,-1,-1,0,1,1,1,1])*3*version
            f_y_dot=np.array([0,0,0,1,1,0,0,-1,0,0,0,0])*3*version


            X=[]
            Y=[]
            for i in range(0,len(delay)-1):
                # Extract the time elements for each section separately
                if i != len(delay)-2:
                    t_temp=t[int(delay[i]/Ts):int(delay[i+1]/Ts)]
                else:
                    t_temp=t[int(delay[i]/Ts):int(delay[i+1]/Ts+1)]

                # Generate data for a subtrajectory
                M=np.array([[1,t_temp[0],t_temp[0]**2,t_temp[0]**3],\
                            [1,t_temp[-1],t_temp[-1]**2,t_temp[-1]**3],\
                            [0,1,2*t_temp[0],3*t_temp[0]**2],\
                            [0,1,2*t_temp[-1],3*t_temp[-1]**2]])

                c_x=np.array([[f_x[i]],[f_x[i+1]-f_x_dot[i+1]*Ts],[f_x_dot[i]],[f_x_dot[i+1]]])
                c_y=np.array([[f_y[i]],[f_y[i+1]-f_y_dot[i+1]*Ts],[f_y_dot[i]],[f_y_dot[i+1]]])


                a_x=np.matmul(np.linalg.inv(M),c_x)
                a_y=np.matmul(np.linalg.inv(M),c_y)

                # Compute X and Y values
                X_temp=a_x[0][0]+a_x[1][0]*t_temp+a_x[2][0]*t_temp**2+a_x[3][0]*t_temp**3
                Y_temp=a_y[0][0]+a_y[1][0]*t_temp+a_y[2][0]*t_temp**2+a_y[3][0]*t_temp**3

                # Concatenate X and Y values
                X=np.concatenate([X,X_temp])
                Y=np.concatenate([Y,Y_temp])

            # Round the numbers to avoid numerical errors
            X=np.round(X,8)
            Y=np.round(Y,8)

            # # Plot the world
            # plt.subplots_adjust(left=0.05,bottom=0.05,right=0.95,top=0.95,wspace=0.15,hspace=0.2)
            # plt.plot(X,Y,'b',linewidth=2,label='The ref trajectory')
            # plt.xlabel('X-position [m]',fontsize=15)
            # plt.ylabel('Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,x_lim)
            # plt.ylim(0,y_lim)
            # plt.xticks(np.arange(0,x_lim+1,int(x_lim/10)))
            # plt.yticks(np.arange(0,y_lim+1,int(y_lim/10)))
            # plt.show()
            #
            # plt.subplots_adjust(left=0.05,bottom=0.05,right=0.95,top=0.95,wspace=0.15,hspace=0.2)
            # plt.plot(t,X,'b',linewidth=2,label='X ref')
            # plt.plot(t,Y,'r',linewidth=2,label='Y ref')
            # plt.xlabel('t-position [s]',fontsize=15)
            # plt.ylabel('X,Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,t[-1])
            # plt.show()
            # # exit()

        # Vector of x and y changes per sample time
        dX=X[1:len(X)]-X[0:len(X)-1]
        dY=Y[1:len(Y)]-Y[0:len(Y)-1]

        X_dot=dX/Ts
        Y_dot=dY/Ts
        X_dot=np.concatenate(([X_dot[0]],X_dot),axis=0)
        Y_dot=np.concatenate(([Y_dot[0]],Y_dot),axis=0)

        # Define the reference yaw angles
        psi=np.zeros(len(X))
        psiInt=psi
        psi[0]=np.arctan2(dY[0],dX[0])
        psi[1:len(psi)]=np.arctan2(dY[0:len(dY)],dX[0:len(dX)])

        # We want the yaw angle to keep track the amount of rotations
        dpsi=psi[1:len(psi)]-psi[0:len(psi)-1]
        psiInt[0]=psi[0]
        for i in range(1,len(psiInt)):
            if dpsi[i-1]<-np.pi:
                psiInt[i]=psiInt[i-1]+(dpsi[i-1]+2*np.pi)
            elif dpsi[i-1]>np.pi:
                psiInt[i]=psiInt[i-1]+(dpsi[i-1]-2*np.pi)
            else:
                psiInt[i]=psiInt[i-1]+dpsi[i-1]


        x_dot_body=np.cos(psiInt)*X_dot+np.sin(psiInt)*Y_dot
        y_dot_body=-np.sin(psiInt)*X_dot+np.cos(psiInt)*Y_dot
        y_dot_body=np.round(y_dot_body)

        # Calculate psi_dot (rate of change of psiInt)
        dpsiInt = psiInt[1:] - psiInt[:-1]  # Differences between consecutive yaw angles
        psi_dot = dpsiInt / Ts  # Divide by the sample time to get the rate of change

        # Append the first value to keep the size consistent
        psi_dot = np.concatenate(([psi_dot[0]], psi_dot))

        # Calculate psi_dot (rate of change of psi)
  

        # # Plot the body frame velocity
        # # plt.plot(t,x_dot_body,'g',linewidth=2,label='x_dot ref')
        # plt.plot(t,X_dot,'b',linewidth=2,label='X_dot ref')
        # plt.plot(t,Y_dot,'r',linewidth=2,label='Y_dot ref')
        # plt.xlabel('t [s]',fontsize=15)
        # plt.ylabel('X_dot_ref, Y_dot_ref [m/s]',fontsize=15)
        # plt.grid(True)
        # plt.legend(loc='upper right',fontsize='small')
        # plt.show()
        
        # # Plot the reference yaw angle
        # plt.plot(t,psiInt,'g',linewidth=2,label='Psi ref')
        # plt.xlabel('t [s]',fontsize=15)
        # plt.ylabel('Psi_ref [rad]',fontsize=15)
        # plt.grid(True)
        # plt.legend(loc='upper right',fontsize='small')
        # plt.show()

        # plt.plot(t,psi_dot,'g',linewidth=2,label='Psi ref')
        # plt.xlabel('t [s]',fontsize=15)
        # plt.ylabel('Psi_dot_ref [rad]',fontsize=15)
        # plt.grid(True)
        # plt.legend(loc='upper right',fontsize='small')
        # plt.show()
        # # exit()

        return x_dot_body,y_dot_body,psiInt,X,Y , psi_dot

    def state_space(self,states,delta,a):
        '''This function forms the state space matrices and transforms them in the discrete form'''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf']
        Cr=self.constants['Cr']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        # Get the necessary states
        x_dot=states[0]
        y_dot=states[1]
        psi=states[2]

        # Get the state space matrices for the control
        A11=-mju*g/x_dot
        A12=Cf*np.sin(delta)/(m*x_dot)
        A14=Cf*lf*np.sin(delta)/(m*x_dot)+y_dot
        A22=-(Cr+Cf*np.cos(delta))/(m*x_dot)
        A24=-(Cf*lf*np.cos(delta)-Cr*lr)/(m*x_dot)-x_dot
        A34=1
        A42=-(Cf*lf*np.cos(delta)-lr*Cr)/(Iz*x_dot)
        A44=-(Cf*lf**2*np.cos(delta)+lr**2*Cr)/(Iz*x_dot)
        A51=np.cos(psi)
        A52=-np.sin(psi)
        A61=np.sin(psi)
        A62=np.cos(psi)

        B11=-1/m*np.sin(delta)*Cf
        B12=1
        B21=1/m*np.cos(delta)*Cf
        B41=1/Iz*np.cos(delta)*Cf*lf


        A=np.array([[A11, A12, 0, A14, 0, 0],[0, A22, 0, A24, 0, 0],[0, 0, 0, A34, 0, 0],\
        [0, A42, 0, A44, 0, 0],[A51, A52, 0, 0, 0, 0],[A61, A62, 0, 0, 0, 0]])
        B=np.array([[B11, B12],[B21, 0],[0, 0],[B41, 0],[0, 0],[0, 0]])
        C=np.array([[1, 0, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 1]])
        D=np.array([[0, 0],[0, 0],[0, 0],[0, 0]])

        # Discretise the system (forward Euler)
        Ad=np.identity(np.size(A,1))+Ts*A
        Bd=Ts*B
        Cd=C
        Dd=D

        return Ad, Bd, Cd, Dd

    def augmented_matrices(self, Ad, Bd, Cd, Dd):

        A_aug=np.concatenate((Ad,Bd),axis=1)
        temp1=np.zeros((np.size(Bd,1),np.size(Ad,1)))
        temp2=np.identity(np.size(Bd,1))
        temp=np.concatenate((temp1,temp2),axis=1)

        A_aug=np.concatenate((A_aug,temp),axis=0)
        B_aug=np.concatenate((Bd,np.identity(np.size(Bd,1))),axis=0)
        C_aug=np.concatenate((Cd,np.zeros((np.size(Cd,0),np.size(Bd,1)))),axis=1)
        D_aug=Dd

        return A_aug, B_aug, C_aug, D_aug

    def mpc_simplification(self, Ad, Bd, Cd, Dd, hz, x_aug_t, du):
        '''This function creates the compact matrices for Model Predictive Control'''
        # db - double bar
        # dbt - double bar transpose
        # dc - double circumflex

        A_aug, B_aug, C_aug, D_aug=self.augmented_matrices(Ad, Bd, Cd, Dd)

        Q=self.constants['Q']
        S=self.constants['S']
        R=self.constants['R']
        Cf=self.constants['Cf']
        g=self.constants['g']
        m=self.constants['m']
        mju=self.constants['mju']
        lf=self.constants['lf']
        inputs=self.constants['inputs']

        ############################### Constraints #############################
        d_delta_max=np.pi/200
        d_a_max=0.4
        d_delta_min=-np.pi/200
        d_a_min=-0.4

        ub_global=np.zeros(inputs*hz)
        lb_global=np.zeros(inputs*hz)

        # Only works for 2 inputs
        for i in range(0,inputs*hz):
            if i%2==0:
                ub_global[i]=d_delta_max
                lb_global[i]=-d_delta_min
            else:
                ub_global[i]=d_a_max
                lb_global[i]=-d_a_min

        ub_global=ub_global[0:inputs*hz]
        lb_global=lb_global[0:inputs*hz]
        ublb_global=np.concatenate((ub_global,lb_global),axis=0)

        I_global=np.eye(inputs*hz)
        I_global_negative=-I_global
        I_mega_global=np.concatenate((I_global,I_global_negative),axis=0)

        y_asterisk_max_global=[]
        y_asterisk_min_global=[]

        C_asterisk=np.matrix('1 0 0 0 0 0 0 0;\
                            0 1 0 0 0 0 0 0;\
                            0 0 0 0 0 0 1 0;\
                            0 0 0 0 0 0 0 1')

        C_asterisk_global=np.zeros((np.size(C_asterisk,0)*hz,np.size(C_asterisk,1)*hz))

        #########################################################################

        CQC=np.matmul(np.transpose(C_aug),Q)
        CQC=np.matmul(CQC,C_aug)

        CSC=np.matmul(np.transpose(C_aug),S)
        CSC=np.matmul(CSC,C_aug)

        QC=np.matmul(Q,C_aug)
        SC=np.matmul(S,C_aug)

        Qdb=np.zeros((np.size(CQC,0)*hz,np.size(CQC,1)*hz))
        Tdb=np.zeros((np.size(QC,0)*hz,np.size(QC,1)*hz))
        Rdb=np.zeros((np.size(R,0)*hz,np.size(R,1)*hz))
        Cdb=np.zeros((np.size(B_aug,0)*hz,np.size(B_aug,1)*hz))
        Adc=np.zeros((np.size(A_aug,0)*hz,np.size(A_aug,1)))

        ######################### Advanced LPV ##################################
        A_product=A_aug
        states_predicted_aug=x_aug_t
        A_aug_collection=np.zeros((hz,np.size(A_aug,0),np.size(A_aug,1)))
        B_aug_collection=np.zeros((hz,np.size(B_aug,0),np.size(B_aug,1)))
        #########################################################################

        for i in range(0,hz):
            if i == hz-1:
                Qdb[np.size(CSC,0)*i:np.size(CSC,0)*i+CSC.shape[0],np.size(CSC,1)*i:np.size(CSC,1)*i+CSC.shape[1]]=CSC
                Tdb[np.size(SC,0)*i:np.size(SC,0)*i+SC.shape[0],np.size(SC,1)*i:np.size(SC,1)*i+SC.shape[1]]=SC
            else:
                Qdb[np.size(CQC,0)*i:np.size(CQC,0)*i+CQC.shape[0],np.size(CQC,1)*i:np.size(CQC,1)*i+CQC.shape[1]]=CQC
                Tdb[np.size(QC,0)*i:np.size(QC,0)*i+QC.shape[0],np.size(QC,1)*i:np.size(QC,1)*i+QC.shape[1]]=QC

            Rdb[np.size(R,0)*i:np.size(R,0)*i+R.shape[0],np.size(R,1)*i:np.size(R,1)*i+R.shape[1]]=R

            ########################### Advanced LPV ############################
            Adc[np.size(A_aug,0)*i:np.size(A_aug,0)*i+A_aug.shape[0],0:0+A_aug.shape[1]]=A_product
            A_aug_collection[i][:][:]=A_aug
            B_aug_collection[i][:][:]=B_aug
            #####################################################################

            ######################## Constraints ################################
            x_dot_max=40
            if 0.2*states_predicted_aug[0][0] < 3.5:
                y_dot_max = 0.25*states_predicted_aug[0][0]
            else:
                y_dot_max = 5
            delta_max=np.pi/2
            Fyf=Cf*(states_predicted_aug[6][0]-states_predicted_aug[1][0]/states_predicted_aug[0][0]-lf*states_predicted_aug[3][0]/states_predicted_aug[0][0])
            a_max=2+(Fyf*np.sin(states_predicted_aug[6][0])+mju*m*g)/m-states_predicted_aug[3][0]*states_predicted_aug[1][0]
            x_dot_min=1.5
            if -0.2*states_predicted_aug[0][0] > -3.5:
                y_dot_min=-0.2*states_predicted_aug[0][0]
            else:
                y_dot_min=-4
            delta_min=-np.pi/2
            a_min=-5+(Fyf*np.sin(states_predicted_aug[6][0])+mju*m*g)/m-states_predicted_aug[3][0]*states_predicted_aug[1][0]

            y_asterisk_max=np.array([x_dot_max,y_dot_max,delta_max,a_max])
            y_asterisk_min=np.array([x_dot_min,y_dot_min,delta_min,a_min])

            y_asterisk_max_global=np.concatenate((y_asterisk_max_global,y_asterisk_max),axis=0)
            y_asterisk_min_global=np.concatenate((y_asterisk_min_global,y_asterisk_min),axis=0)

            C_asterisk_global[np.size(C_asterisk,0)*i:np.size(C_asterisk,0)*i+C_asterisk.shape[0],np.size(C_asterisk,1)*i:np.size(C_asterisk,1)*i+C_asterisk.shape[1]]=C_asterisk


            #####################################################################

            ######################### Advanced LPV ##############################
            if i<hz-1:
                du1=du[inputs*(i+1)][0]
                du2=du[inputs*(i+1)+inputs-1][0]
                states_predicted_aug=np.matmul(A_aug,states_predicted_aug)+np.matmul(B_aug,np.transpose([[du1,du2]]))
                states_predicted=np.transpose(states_predicted_aug[0:6])[0]
                delta_predicted=states_predicted_aug[6][0]
                a_predicted=states_predicted_aug[7][0]
                Ad, Bd, Cd, Dd=self.state_space(states_predicted,delta_predicted,a_predicted)
                A_aug, B_aug, C_aug, D_aug=self.augmented_matrices(Ad, Bd, Cd, Dd)
                A_product=np.matmul(A_aug,A_product)

        for i in range(0,hz):
            for j in range(0,hz):
                if j<=i:
                    AB_product=np.eye(np.shape(A_aug)[0])
                    for ii in range(i,j-1,-1):
                        if ii>j:
                            AB_product=np.matmul(AB_product,A_aug_collection[ii][:][:])
                        else:
                            AB_product=np.matmul(AB_product,B_aug_collection[ii][:][:])
                    Cdb[np.size(B_aug,0)*i:np.size(B_aug,0)*i+B_aug.shape[0],np.size(B_aug,1)*j:np.size(B_aug,1)*j+B_aug.shape[1]]=AB_product

        #########################################################################

        ####################### Constraints #####################################

        Cdb_constraints=np.matmul(C_asterisk_global,Cdb)
        Cdb_constraints_negative=-Cdb_constraints
        Cdb_constraints_global=np.concatenate((Cdb_constraints,Cdb_constraints_negative),axis=0)

        Adc_constraints=np.matmul(C_asterisk_global,Adc)
        Adc_constraints_x0=np.transpose(np.matmul(Adc_constraints,x_aug_t))[0]
        y_max_Adc_difference=y_asterisk_max_global-Adc_constraints_x0
        y_min_Adc_difference=-y_asterisk_min_global+Adc_constraints_x0
        y_Adc_difference_global=np.concatenate((y_max_Adc_difference,y_min_Adc_difference),axis=0)

        G=np.concatenate((I_mega_global,Cdb_constraints_global),axis=0)
        ht=np.concatenate((ublb_global,y_Adc_difference_global),axis=0)

        #######################################################################

        Hdb=np.matmul(np.transpose(Cdb),Qdb)
        Hdb=np.matmul(Hdb,Cdb)+Rdb

        temp=np.matmul(np.transpose(Adc),Qdb)
        temp=np.matmul(temp,Cdb)

        temp2=np.matmul(-Tdb,Cdb)
        Fdbt=np.concatenate((temp,temp2),axis=0)

        return Hdb,Fdbt,Cdb,Adc,G,ht

    def open_loop_new_states(self,states,delta,a):
        '''This function computes the new state vector for one sample time later'''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf_1']
        Cr=self.constants['Cr_1']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        current_states=states
        new_states=current_states
        x_dot=current_states[0]
        y_dot=current_states[1]
        psi=current_states[2]
        psi_dot=current_states[3]
        X=current_states[4]
        Y=current_states[5]

        sub_loop=30  #Chops Ts into 30 pieces
        for i in range(0,sub_loop):

            # Compute lateral forces
            Fyf=Cf*(delta-y_dot/x_dot - lf*psi_dot/x_dot)
            Fyr=Cr*(-y_dot/x_dot + lr*psi_dot/x_dot)

            # Compute the the derivatives of the states
            x_dot_dot=a+(-Fyf*np.sin(delta)-mju*m*g)/m+psi_dot*y_dot
            y_dot_dot=(Fyf*np.cos(delta)+Fyr)/m-psi_dot*x_dot
            psi_dot=psi_dot
            psi_dot_dot=(Fyf*lf*np.cos(delta)-Fyr*lr)/Iz
            X_dot=x_dot*np.cos(psi)-y_dot*np.sin(psi)
            Y_dot=x_dot*np.sin(psi)+y_dot*np.cos(psi)

            # Update the state values with new state derivatives
            x_dot=x_dot+x_dot_dot*Ts/sub_loop
            y_dot=y_dot+y_dot_dot*Ts/sub_loop
            psi=psi+psi_dot*Ts/sub_loop
            psi_dot=psi_dot+psi_dot_dot*Ts/sub_loop
            X=X+X_dot*Ts/sub_loop
            Y=Y+Y_dot*Ts/sub_loop

        # Take the last states
        new_states[0]=x_dot
        new_states[1]=y_dot
        new_states[2]=psi
        new_states[3]=psi_dot
        new_states[4]=X
        new_states[5]=Y

        return new_states,x_dot_dot,y_dot_dot,psi_dot_dot
    
    def open_loop_new_states_RK4(self,states,delta,a , uncertain_r ,uncertain_f):
        '''This function computes the new state vector for one sample time later'''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf_1']
        Cr=self.constants['Cr_1']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        # current_states=states
        # new_states=current_states


        def state_dot_fn(t,states):
            x_dot=states[0]
            y_dot=states[1]
            psi=states[2]
            psi_dot=states[3]
            X=states[4]
            Y=states[5]

        
            # Compute lateral forces
            Fyf=Cf*(delta-y_dot/x_dot - lf*psi_dot/x_dot) + uncertain_f
            Fyr=Cr*(-y_dot/x_dot + lr*psi_dot/x_dot) +  uncertain_r

            # Compute the the derivatives of the states
            x_dot_dot=a+(-Fyf*np.sin(delta)-mju*m*g)/m+psi_dot*y_dot
            y_dot_dot=(Fyf*np.cos(delta)+Fyr)/m-psi_dot*x_dot
            psi_dot=psi_dot
            psi_dot_dot=(Fyf*lf*np.cos(delta)-Fyr*lr)/Iz
            X_dot=x_dot*np.cos(psi)-y_dot*np.sin(psi)
            Y_dot=x_dot*np.sin(psi)+y_dot*np.cos(psi)

            return [x_dot_dot, y_dot_dot, psi_dot, psi_dot_dot, X_dot, Y_dot]


        sol =  scipy.integrate.solve_ivp(state_dot_fn, [0, Ts], states, method='RK45')

        # Take the last states from the solution
        new_states = sol.y[:, -1]  # Last column gives the final state

        # Compute accelerations using the final state values
        x_dot_dot = (new_states[0]  - sol.y[0, -2])/Ts
        y_dot_dot = (new_states[1] -  sol.y[1, -2])/Ts
        psi_dot_dot = (new_states[3]  - sol.y[3, -2])/Ts

        # Fyf = Cf * (delta - y_dot / x_dot - lf * psi_dot / x_dot)
        # Fyr = Cr * (-y_dot / x_dot + lr * psi_dot / x_dot)

        # # Recompute accelerations from final state
        # x_dot_dot = a + (-Fyf * np.sin(delta) - mju * m * g) / m + psi_dot * y_dot
        # y_dot_dot = (Fyf * np.cos(delta) + Fyr) / m - psi_dot * x_dot
        # psi_dot_dot = (Fyf * lf * np.cos(delta) - Fyr * lr) / Iz

        return new_states,x_dot_dot,y_dot_dot,psi_dot_dot

    # Disturbance model (random walk model)
    def dis_noise(self,states,delta,a):

        Cf_nonlinear=self.constants['Cf_nonlinear']
        Cr_nonlinear=self.constants['Cr_nonlinear']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        current_states=states
        new_states=current_states
        x_dot=current_states[0]
        y_dot=current_states[1]
        psi=current_states[2]
        psi_dot=current_states[3]



        # Compute lateral forces
        # noise =  10 * np.random.random()
        noise =  0
        Fyf_unknow = Cf_nonlinear*(delta-y_dot/x_dot - lf*psi_dot/x_dot) + noise
        Fyr_unknow = Cr_nonlinear*(-y_dot/x_dot + lr*psi_dot/x_dot)  + noise
        # State-dependent process noise for generating the disturbance force and troque
        #         0 is the mean of the distribution, meaning the disturbances are centered around zero.
        #          Fyf_unknow is the standard deviation (not variance) of the distribution, calculated earlier in the code as a state-dependent value.
        #           1 specifies that only one sample should be generated.

        # f_yf     = np.random.normal(0,Fyf_unknow,1) 
        # f_yr     = np.random.normal(0,Fyr_unknow,1)


        return Fyf_unknow, Fyr_unknow

    def open_loop_new_states_uncertain(self,states,delta,a,uncertain_r ,uncertain_f ):
        '''This function computes the new state vector for one sample time later'''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf']
        Cr=self.constants['Cr']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        current_states=states
        new_states=current_states
        x_dot=current_states[0]
        y_dot=current_states[1]
        psi=current_states[2]
        psi_dot=current_states[3]
        X=current_states[4]
        Y=current_states[5]

        sub_loop=30  #Chops Ts into 30 pieces
        for i in range(0,sub_loop):

            # Compute lateral forces
            Fyf=Cf*(delta-y_dot/x_dot-lf*psi_dot/x_dot) + uncertain_f
            Fyr=Cr*(-y_dot/x_dot+lr*psi_dot/x_dot)  + uncertain_r

            # Compute the the derivatives of the states
            x_dot_dot=a+(-Fyf*np.sin(delta)-mju*m*g)/m+psi_dot*y_dot
            y_dot_dot=(Fyf*np.cos(delta)+Fyr)/m-psi_dot*x_dot
            psi_dot=psi_dot
            psi_dot_dot=(Fyf*lf*np.cos(delta)-Fyr*lr)/Iz
            X_dot=x_dot*np.cos(psi)-y_dot*np.sin(psi)
            Y_dot=x_dot*np.sin(psi)+y_dot*np.cos(psi)

            # Update the state values with new state derivatives
            x_dot=x_dot+x_dot_dot*Ts/sub_loop
            y_dot=y_dot+y_dot_dot*Ts/sub_loop
            psi=psi+psi_dot*Ts/sub_loop
            psi_dot=psi_dot+psi_dot_dot*Ts/sub_loop
            X=X+X_dot*Ts/sub_loop
            Y=Y+Y_dot*Ts/sub_loop

        # Take the last states
        new_states[0]=x_dot
        new_states[1]=y_dot
        new_states[2]=psi
        new_states[3]=psi_dot
        new_states[4]=X
        new_states[5]=Y

        return new_states,x_dot_dot,y_dot_dot,psi_dot_dot

    def matrix_state_space_simple_observer(self, state, delta, a):
        ''' This function is give matrix , to the observer or controller
            This model matrix suppose to be not the true , only an aproximate of the true model
        '''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']

        
        Cf=self.constants['Cf_1']
        Cr=self.constants['Cr_1']

        
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']


        # Unpack the state vector (x_dot, y_dot, psi)
        x_dot = state[0]
        y_dot = state[1]
        psi = state[2]

        # Compute state-space matrices A and B
        A11 = -mju * g / x_dot
        A12 = Cf * np.sin(delta) / (m * x_dot)
        A14 = Cf * lf * np.sin(delta) / (m * x_dot) + y_dot
        A22 = -(Cr + Cf * np.cos(delta)) / (m * x_dot)
        A24 = -(Cf * lf * np.cos(delta) - Cr * lr) / (m * x_dot) - x_dot
        A34 = 1
        A42 = -(Cf * lf * np.cos(delta) - lr * Cr) / (Iz * x_dot)
        A44 = -(Cf * lf**2 * np.cos(delta) + lr**2 * Cr) / (Iz * x_dot)



        # State-space matrix A
        A_small = np.array([
            [A11, A12, 0, A14],
            [0  , A22, 0, A24],
            [0  , 0  , 0, A34],
            [0  , A42, 0, A44]
        ])

        # State-space matrix B
        B11 = -1 / m * np.sin(delta) * Cf
        B12 = 1
        B21 = 1 / m * np.cos(delta) * Cf
        B41 = 1 / Iz * np.cos(delta) * Cf * lf

        B = np.array([
            [B11, B12],
            [B21, 0],
            [0, 0],
            [B41, 0]
        ])

        # Output matrix C (from state to output)
        C = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
        ])



        D = np.array([
            [0,  np.sin(delta)/m],
            [1/m, np.cos(delta)/m  ],
            [0, 0],
            [-lr /Iz, np.cos(delta)*lf/Iz]
        ])




        return A_small, B, C,D
    
    def matrix_state_space_simple_observer_discret(self, state, delta, a):
        ''' This function is give matrix , to the observer or controller
            This model matrix suppose to be not the true , only an aproximate of the true model
        '''
        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        
        # Faux value
        # Cf=self.constants_faux['Cf_faux']
        # Cr=self.constants_faux['Cr_faux']

        # lf=self.constants_faux['lf_faux']
        # lr=self.constants_faux['lr_faux']

        # Real value

        Cf=self.constants['Cf_1']
        Cr=self.constants['Cr_1']


        lf=self.constants['lf']
        lr=self.constants['lr']


        Ts=self.constants['Ts']
        mju=self.constants['mju']


        # Unpack the state vector (x_dot, y_dot, psi)
        x_dot = state[0]
        y_dot = state[1]
        psi = state[2]

        # Compute state-space matrices A and B
        A11 = -mju * g / x_dot
        A12 = Cf * np.sin(delta) / (m * x_dot)
        A14 = Cf * lf * np.sin(delta) / (m * x_dot) + y_dot
        A22 = -(Cr + Cf * np.cos(delta)) / (m * x_dot)
        A24 = -(Cf * lf * np.cos(delta) - Cr * lr) / (m * x_dot) - x_dot
        A34 = 1
        A42 = -(Cf * lf * np.cos(delta) - lr * Cr) / (Iz * x_dot)
        A44 = -(Cf * lf**2 * np.cos(delta) + lr**2 * Cr) / (Iz * x_dot)



        # State-space matrix A
        A_small = np.array([
            [A11, A12, 0, A14],
            [0  , A22, 0, A24],
            [0  , 0  , 0, A34],
            [0  , A42, 0, A44]
        ])

        # State-space matrix B
        B11 = -1 / m * np.sin(delta) * Cf
        B12 = 1
        B21 = 1 / m * np.cos(delta) * Cf
        B41 = 1 / Iz * np.cos(delta) * Cf * lf


        B = np.array([
            [B11, B12],
            [B21, 0],
            [0, 0],
            [B41, 0]
        ])

        # Output matrix C (from state to output)
        C = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
        ])



        D = np.array([
            [0,  np.sin(delta)/m],
            [1/m, np.cos(delta)/m  ],
            [0, 0],
            [-lr /Iz, np.cos(delta)*lf/Iz]
        ])


        # # Discretise the system (forward Euler)
        Ad=np.identity(np.size(A_small,1)) + Ts*A_small
        Bd=Ts*B
        Dd=Ts*D
        Cd=C


        return Ad, Bd, Cd,Dd
      
    def observer(self,states_hat, delta, a, y_mesure,f_hat,f_nn ):

        Ts=self.constants['Ts']
        control_input = np.array([delta , a])
        x_hat_i = states_hat.reshape(-1, 1)
        control_input = control_input.reshape(-1, 1)
        y_mesure = y_mesure.reshape(-1, 1)
        f_hat = f_hat.reshape(-1, 1)



        # x_hat_new = x_hat_i + Ts*(self.A_small_fix_conti @ x_hat_i + self.B_small_fix_conti @ control_input + self.K_P_continue @ (self.C_small_fix_conti @ x_hat_i - y_mesure)   )
        # f_hat_new = f_hat + Ts*(self.K_I_continue @ (self.C_small_fix_conti @ x_hat_i - y_mesure))


        x_hat_new = self.A_small_fix @ x_hat_i + self.B_small_fix @ control_input + self.K_P_discret @ (self.C_small_fix @ x_hat_i - y_mesure)   +  self.D_fix@f_nn 

        # Compute the new disturbance estimate (f_hat_new)
        f_hat_new = self.K_I_discret @ (self.C_small_fix @ x_hat_i - y_mesure)


        x_hat_new = x_hat_new.flatten()
        f_hat_new = f_hat_new.reshape(4)

        return x_hat_new , f_hat_new
    
    def observer2(self,states_hat, delta, a, y_mesure,f_hat,f_nn ):

        Ts=self.constants['Ts']
        control_input = np.array([delta , a])
        x_hat_i = states_hat.reshape(-1, 1)
        control_input = control_input.reshape(-1, 1)
        y_mesure = y_mesure.reshape(-1, 1)
        f_hat = f_hat.reshape(-1, 1)


        
        # Compute the new state estimate (x_hat_new)
        # Ensure matrix multiplications
        x_hat_new = self.A_small_fix_2 @ x_hat_i + self.B_small_fix_2 @ control_input + self.K_P_discret @ (self.C_small_fix @ x_hat_i - y_mesure)  + f_hat
        

        # Compute the new disturbance estimate (f_hat_new)
        f_hat_new = self.K_I_discret @ (self.C_small_fix @ x_hat_i - y_mesure)


        x_hat_new = x_hat_new.flatten()
        f_hat_new = f_hat_new.reshape(4)

        return x_hat_new , f_hat_new
    


    


    def observer_with_D(self,states_hat, delta, a, y_mesure,f_hat,f_nn ):

        Ts=self.constants['Ts']
        control_input = np.array([delta , a])
        x_hat_i = states_hat.reshape(-1, 1)
        control_input = control_input.reshape(-1, 1)
        y_mesure = y_mesure.reshape(-1, 1)
        f_hat = f_hat.reshape(-1, 1)


        
        # Compute the new state estimate (x_hat_new)

        # A_small, B_small, C_small, D_small = self.matrix_state_space_simple_observer(states_hat , delta , a )


        # x_hat_new = x_hat_i + Ts*(self.A_small_fix_conti @ x_hat_i + self.B_small_fix_conti @ control_input + self.K_Huy_continue @ (self.C_small_fix_conti @ x_hat_i - y_mesure)  + self.D_small_fix_conti@f_nn )
        # f_hat_new = f_hat + Ts*(self.K_I_continue_w_D @ (self.C_small_fix_conti @ x_hat_i - y_mesure))


        # ------ 

        # x_hat_new = self.A_small_fix @ x_hat_i + self.B_small_fix @ control_input + self.K_P_dis_w_D @ (self.C_small_fix @ x_hat_i - y_mesure)  + self.D_small_fix@f_nn 
        # f_hat_new = self.K_I_dis_w_D @ (self.C_small_fix @ x_hat_i - y_mesure)


        A_small, B_small, C_small, D_small = self.matrix_state_space_simple_observer_discret(states_hat , delta , a )


        # x_hat_new = self.A_small_fix @ x_hat_i + self.B_small_fix @ control_input + self.K_P_dis_w_D @ (self.C_small_fix @ x_hat_i - y_mesure)  + D_small@f_nn
        # f_hat_new = self.K_I_dis_w_D @ (self.C_small_fix @ x_hat_i - y_mesure)

        x_hat_new = self.A_small_fix @ x_hat_i + self.B_small_fix @ control_input + self.K_P_dis_w_D @ (self.C_small_fix @ x_hat_i - y_mesure)  + self.D_small_fix@f_nn
        f_hat_new = self.K_I_dis_w_D @ (self.C_small_fix @ x_hat_i - y_mesure)


        x_hat_new = x_hat_new.flatten()
        f_hat_new = f_hat_new.reshape(2)

        return x_hat_new , f_hat_new
 

    def observer2_with_D(self,states_hat, delta, a, y_mesure,f_hat,f_nn ):

        Ts=self.constants['Ts']
        control_input = np.array([delta , a])
        x_hat_i = states_hat.reshape(-1, 1)
        control_input = control_input.reshape(-1, 1)
        y_mesure = y_mesure.reshape(-1, 1)
        f_hat = f_hat.reshape(-1, 1)


        # x_hat_new = x_hat_i + Ts*(self.A_small_fix_conti @ x_hat_i + self.B_small_fix_conti @ control_input + self.K_P_continue_w_D @ (self.C_small_fix_conti @ x_hat_i - y_mesure)  + self.D_small_fix_conti@f_hat )
        # f_hat_new = f_hat + Ts*(self.K_I_continue_w_D @ (self.C_small_fix_conti @ x_hat_i - y_mesure))
         


        # Compute the new state estimate (x_hat_new)
        ## Ensure matrix multiplications

        A_small, B_small, C_small, D_small = self.matrix_state_space_simple_observer_discret(states_hat , delta , a )

        # x_hat_new = A_small @ x_hat_i + self.B_small_fix_2 @ control_input + self.K_P_dis_w_D @ (C_small @ x_hat_i - y_mesure)  + D_small@f_hat
        # f_hat_new = self.K_I_dis_w_D @ (self.C_small_fix @ x_hat_i - y_mesure)

        x_hat_new = self.A_small_fix_2 @ x_hat_i + self.B_small_fix_2 @ control_input + self.K_P_dis_w_D @ (self.C_small_fix @ x_hat_i - y_mesure)  + D_small@f_hat
        f_hat_new = self.K_I_dis_w_D @ (self.C_small_fix @ x_hat_i - y_mesure)


        x_hat_new = x_hat_new.flatten()
        f_hat_new = f_hat_new.reshape(2)

        return x_hat_new , f_hat_new

    def compensate_Control(self, U1 , U2 ,f_nn):
        
        U = np.array([U1] , [U2])
        U = np.array([U1] , [U2]) + self.B_d_pseudo_inverse * f_nn
        return U1 , U2

    # def moving_average(self,output, window_size=3):
    #     return np.convolve(output, np.ones(window_size)/window_size, mode='valid')

    def moving_average_matrix(self,f_nn_new , last_columns_from_save, window_size=3 , weight_type = 'linear'):

        # Get the last window_size - 1 columns from f_nn_Save (from the saved history)
        # last_columns_from_save = f_nn_Save[:, -window_size+1:]  # Shape (n_rows, window_size-1)

        # Step 2: Stack the current f_nn as the last column with the saved columns
        combined_columns = np.hstack((last_columns_from_save, f_nn_new))  # Shape (n_rows, window_size)

        # Define the weights for the last, second-to-last, and third-to-last columns
        # Step 2: Generate dynamic weights based on window_size and weight_type
        if weight_type == 'linear':
            weights = np.arange(1, window_size + 1)  # [1, 2, 3, ..., window_size]
        elif weight_type == 'exponential':
            weights = np.exp(np.arange(window_size))  # exponential growth weights
        elif weight_type == 'uniform':
            weights = np.ones(window_size)  # uniform weights
        else:
            raise ValueError(f"Unknown weight_type: {weight_type}")

        # Normalize weights to sum to 1
        weights = weights / np.sum(weights)
        # weights = np.array([0.1, 0.2, 0.7])

        # Compute the weighted sum (row-wise)
        smoothed_last_column = np.dot(combined_columns, weights)
        
        # Overwrite the last column with the weighted smoothed values
        # f_nn_Save[:, -1] = smoothed_last_column
        return smoothed_last_column.reshape(-1,1)

    def moving_average(self,series, window_size=3, weight_type='linear'):
        """
        Computes the moving average for a 1D series of numbers with specified weighting.

        Parameters:
            series (list or np.array): The series of numbers to compute the moving average for.
            window_size (int): The number of elements to include in each moving average calculation.
            weight_type (str): The type of weighting ('linear', 'exponential', 'uniform').

        Returns:
            np.array: The moving average of the series with the specified weighting.
        """
        series = np.array(series)
        if len(series) < window_size:
            raise ValueError("Series length must be greater than or equal to the window size.")

        # Define weights based on the weight type
        if weight_type == 'linear':
            weights = np.arange(1, window_size + 1)  # [1, 2, 3, ..., window_size]
        elif weight_type == 'exponential':
            weights = np.exp(np.arange(window_size))  # exponential growth weights
        elif weight_type == 'uniform':
            weights = np.ones(window_size)  # uniform weights
        else:
            raise ValueError(f"Unknown weight_type: {weight_type}")

        # Normalize weights to sum to 1
        weights = weights / np.sum(weights)

        # Compute the weighted moving average
        moving_avg = np.convolve(series, weights, mode='valid')

        return moving_avg

        
    def state_space_step_augmented(self , state_augemented , control):

        ''' This function is like open loop , that contain the true model '''
        # Define state and control variables

        # # Unpack state
        # x_dot = state[0]
        # y_dot = state[1]
        # psi = state[2]
        # psi_dot = state[3]
        # X = state[4]
        # Y = state[5]

        state = state_augemented[:6]  # The original state vector: [x_dot, y_dot, psi, psi_dot, X, Y]


        # Control inputs
        delta = control[0]  # Steering angle
        a = control[1]  # Acceleration


        Ad , Bd , Cd , Dd = self.state_space_symbolic(state , delta , a)
       
        A_aug, B_aug, C_aug, D_aug =  self.augmented_matrices_symbolic(Ad , Bd , Cd , Dd)



        next_state_augemented = ca.mtimes(A_aug, state_augemented) + ca.mtimes(B_aug, control)
         
        return next_state_augemented
    
    
    def state_space_symbolic(self, states, delta, a):
        '''This function forms the symbolic state-space matrices and transforms them in discrete form'''

        # Define symbolic state and parameter variables
        # x_dot = ca.MX.sym('x_dot')
        # y_dot = ca.MX.sym('y_dot')
        # psi = ca.MX.sym('psi')

        x_dot=states[0]
        y_dot=states[1]
        psi=states[2]
        
        # Define constants as symbolic variables (you can replace them with actual numbers if needed)
        # g = ca.MX.sym('g')
        # m = ca.MX.sym('m')
        # Iz = ca.MX.sym('Iz')
        # Cf = ca.MX.sym('Cf')
        # Cr = ca.MX.sym('Cr')
        # lf = ca.MX.sym('lf')
        # lr = ca.MX.sym('lr')
        # Ts = ca.MX.sym('Ts')
        # mju = ca.MX.sym('mju')

        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf']
        Cr=self.constants['Cr']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        # State space matrices in symbolic form
        A11 = -mju * g / x_dot
        A12 = Cf * ca.MX.sin(delta) / (m * x_dot)
        A14 = Cf * lf * ca.MX.sin(delta) / (m * x_dot) + y_dot
        A22 = -(Cr + Cf * ca.MX.cos(delta)) / (m * x_dot)
        A24 = -(Cf * lf * ca.MX.cos(delta) - Cr * lr) / (m * x_dot) - x_dot
        A34 = 1
        A42 = -(Cf * lf * ca.MX.cos(delta) - lr * Cr) / (Iz * x_dot)
        A44 = -(Cf * lf**2 * ca.MX.cos(delta) + lr**2 * Cr) / (Iz * x_dot)
        A51 = ca.MX.cos(psi)
        A52 = -ca.MX.sin(psi)
        A61 = ca.MX.sin(psi)
        A62 = ca.MX.cos(psi)

        # Control input matrices in symbolic form
        B11 = -1 / m * ca.MX.sin(delta) * Cf
        B12 = 1
        B21 = 1 / m * ca.MX.cos(delta) * Cf
        B41 = 1 / Iz * ca.MX.cos(delta) * Cf * lf

        # Assemble symbolic state-space matrices
        A = ca.vertcat(
            ca.horzcat(A11, A12, 0, A14, 0, 0),
            ca.horzcat(0, A22, 0, A24, 0, 0),
            ca.horzcat(0, 0, 0, A34, 0, 0),
            ca.horzcat(0, A42, 0, A44, 0, 0),
            ca.horzcat(A51, A52, 0, 0, 0, 0),
            ca.horzcat(A61, A62, 0, 0, 0, 0)
        )

        B = ca.vertcat(
            ca.horzcat(B11, B12),
            ca.horzcat(B21, 0),
            ca.horzcat(0, 0),
            ca.horzcat(B41, 0),
            ca.horzcat(0, 0),
            ca.horzcat(0, 0)
        )

        C = ca.vertcat(
            ca.horzcat(1, 0, 0, 0, 0, 0),
            ca.horzcat(0, 0, 1, 0, 0, 0),
            ca.horzcat(0, 0, 0, 0, 1, 0),
            ca.horzcat(0, 0, 0, 0, 0, 1)
        )

        D = ca.MX.zeros(4, 2)

        # Discretize the system using forward Euler
        Ad = ca.MX.eye(A.size1()) + Ts * A
        Bd = Ts * B
        Cd = C
        Dd = D

        # Return the symbolic matrices
        return Ad, Bd, Cd, Dd


        # A=np.array([[A11, A12, 0, A14, 0, 0],[0, A22, 0, A24, 0, 0],[0, 0, 0, A34, 0, 0],\
        # [0, A42, 0, A44, 0, 0],[A51, A52, 0, 0, 0, 0],[A61, A62, 0, 0, 0, 0]])
        # B=np.array([[B11, B12],[B21, 0],[0, 0],[B41, 0],[0, 0],[0, 0]])
        # C=np.array([[1, 0, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 1]])
        # D=np.array([[0, 0],[0, 0],[0, 0],[0, 0]])

        # # Discretise the system (forward Euler)
        # Ad=np.identity(np.size(A,1))+Ts*A
        # Bd=Ts*B
        # Cd=C
        # Dd=D

        # return Ad, Bd, Cd, Dd

    def augmented_matrices_symbolic(self, Ad, Bd, Cd, Dd):
        '''This function forms the augmented matrices in symbolic form'''

        # Augment the A matrix by concatenating Ad and Bd
        A_aug = ca.horzcat(Ad, Bd)

        # Create a symbolic matrix of zeros for concatenation
        temp1 = ca.MX.zeros(Bd.size2(), Ad.size1())
        temp2 = ca.MX.eye(Bd.size2())

        # Augment the A matrix further with temp1 and temp2
        temp = ca.horzcat(temp1, temp2)
        A_aug = ca.vertcat(A_aug, temp)

        # Augment the B matrix by concatenating Bd and identity matrix
        B_aug = ca.vertcat(Bd, ca.MX.eye(Bd.size2()))

        # Augment the C matrix by concatenating Cd and a matrix of zeros
        C_aug = ca.horzcat(Cd, ca.MX.zeros(Cd.size1(), Bd.size2()))

        # D_aug remains the same
        D_aug = Dd

        return A_aug, B_aug, C_aug, D_aug
    
    def constrain_the_fnn(self,f_nn):
        # Define the limits
        x_dot_max = 0.3
        x_dot_min = -0.3

        y_dot_max = 0.5
        y_dot_min = -0.5

        psi_max = 0.4
        psi_min = -0.4

        # Apply constraints using np.clip
        f_nn[0] = np.clip(f_nn[0], x_dot_min, x_dot_max)  # Constrain x_dot
        f_nn[1] = np.clip(f_nn[1], y_dot_min, y_dot_max)  # Constrain y_dot
        f_nn[2] = np.clip(f_nn[2], psi_min, psi_max)      # Constrain psi
        f_nn[3] = np.clip(f_nn[3], psi_min, psi_max)      # Assuming f_nn[3] represents another psi

        return f_nn

            # if 0.18*states_predicted_aug[0][0] < 3.5:
            #     y_dot_max=0.2*states_predicted_aug[0][0]
            # else:
            #     y_dot_max=3.5
            # delta_max=np.pi/5
            # Fyf=Cf*(states_predicted_aug[6][0]-states_predicted_aug[1][0]/states_predicted_aug[0][0]-lf*states_predicted_aug[3][0]/states_predicted_aug[0][0])
            # a_max=1.5+(Fyf*np.sin(states_predicted_aug[6][0])+mju*m*g)/m-states_predicted_aug[3][0]*states_predicted_aug[1][0]
            # x_dot_min=1.5
            # if -0.18*states_predicted_aug[0][0] > -3.5:
            #     y_dot_min=-0.2*states_predicted_aug[0][0]
            # else:
            #     y_dot_min=-3.5
            # delta_min=-np.pi/5
            # a_min=-5+(Fyf*np.sin(states_predicted_aug[6][0])+mju*m*g)/m-states_predicted_aug[3][0]*states_predicted_aug[1][0]




    def UKF_setUp(self , init_state ,Ts):
        # Define the UKF process function (state transition without nonlinearity)
        def fx(x, dt,u):
            x = x.reshape(-1, 1)
            u = u.reshape(-1, 1)
            x_new =  self.A_small_fix_conti@ x + self.B_small_fix_conti @ u  # Linear dynamics for now, add f_uk(x) if needed
            return x_new.flatten()
        # Define the measurement function (linear observation)
        def hx(x):
            x = x.reshape(-1, 1)
            y_new = self.C_small_fix_conti @ x
             
            return y_new.flatten()
        # Unscented transform parameters
        points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2., kappa=1)

        # Initialize the Unscented Kalman Filter
        self.ukf = UKF(dim_x=4, dim_z=2, fx=fx, hx=hx, dt=Ts, points=points)

        # Initial state (4-dimensional)
        self.ukf.x = init_state

        # Initial covariance matrix
        self.ukf.P = np.eye(4) * 0.1

        # Process noise covariance (Q) and measurement noise covariance (R)
        self.ukf.Q = np.eye(4) * 0.01
        self.ukf.R = np.eye(2) * 0.1

        pass

    def UKF_estimate(self , delta, a, y_mesure) :
        control_u = np.array([delta , a])
        # control_u = control_u.reshape(-1, 1)
        y_mesure = y_mesure.reshape(-1, 1)
        y_mes = np.array([float(y_mesure[0]) ,float(y_mesure[1])])


        # Perform prediction step
        self.ukf.predict(u = control_u )

        self.ukf.update(y_mes)
        return self.ukf.x


    def Gain_observer_discret(self):
        self.K_P_discret=np.matrix('-0.999018999802465	1.97534519489663e-09 ; \
                                -6.05308606350771e-08	-6.05308608449148e-07; \
                                1.97534519337374e-09	-0.999999980246551;\
                                -5.30202680987599e-08	-5.30202683806855e-07')

        self.K_I_discret=np.matrix('-1.55971637850149e-12	-1.56006380491143e-11; \
                            -7.99290996763423e-10	-7.99291342020207e-09; \
                            -1.56006380848150e-11	-1.56009854788739e-10; \
                            2.06945181454930e-11	2.06941707637038e-10')
        
        self.K_P_dis_w_D=np.matrix('-0.999018999802465	1.97534519489663e-09 ; \
                                -6.05308606350771e-08	-6.05308608449148e-07; \
                                1.97534519337374e-09	-0.999999980246551;\
                                -5.30202680987599e-08	-5.30202683806855e-07')
            
        self.K_I_dis_w_D=np.matrix('-1.55971637850149e-12	-1.56006380491143e-11; \
                            -7.99290996763423e-10	-7.99291342020207e-09')



    def Gain_observer_continu(self):
        self.K_P_continue=np.matrix('-0.619153452064276	4.24731697694881e-11; \
                                    4.71226546284822e-10	-0.132669645470167; \
                                    -2.90066594078242e-11	-0.648998065902704; \
                                    -9.70263290523128e-10	-1.10472455234990')
        
        
        self.K_I_continue=np.matrix('-1.06658263994320	4.28314082248076e-12; \
                                    -8.03587082895279e-13	-0.00860825932690581; \
                                    -4.06079651491426e-12	-1.06595937952981; \
                                    -9.02639277836634e-14	-0.0187148191258271')
        



        self.K_P_continue_w_D =np.matrix('-4.80096738979851	1.57388598437361e-09; \
                            7.43849285604900e-10	0.0390539091868105; \
                            -1.56735294350963e-09	-4.84283662431894; \
                            -6.54715518915661e-10	-0.487425679558652')
        
        self.K_I_continue_w_D=np.matrix('-9.93183806524528e-15	3.71465113748601e-06; \
                                        -9.31315733090367e-15	-1.60984487297464e-06')

        self.K_Huy_continue=np.matrix('-0.403937267643435	4.24796069626324e-12; \
                                        3.34855523564156e-11	-1.52611935936806; \
                                        -2.39084243226004e-12	-0.451651407640288; \
                                        -1.05892162029760e-10	0.437485625069983')



        self.K_P_3_continue=np.matrix('  -0.613948753450052	6.42342570393282e-11	1.09243682217592e-10; \
                                        1.90783048839008e-09	-0.280967904311732	-3.75855343685081; \
                                        9.21059282620321e-12	-1	-0.0; \
                                        -5.02207690001747e-09	0.0125407614124539	26.9342102819847')
        
        self.K_I_3_continue=np.matrix(' -1.0271269576024	8.04876213174619e-12	2.62500516656090e-11;\
                                        -5.51132648594092e-12	-0.00967508829728477	-0.0523604988003886;\
                                        -7.10816827504047e-12	-1.	-0.0;\
                                        -9.67226748701908e-12	-0.0199416658147967	-0.0875386288898272')
    
    def observer_3mesure_1(self,states_hat, delta, a, y_mesure,f_hat,f_nn ):

        Ts=self.constants['Ts']
        control_input = np.array([delta , a])
        x_hat_i = states_hat.reshape(-1, 1)
        control_input = control_input.reshape(-1, 1)
        y_mesure = y_mesure.reshape(-1, 1)
        f_hat = f_hat.reshape(-1, 1)



        x_hat_new = x_hat_i + Ts*(self.A_small_fix_conti @ x_hat_i + self.B_small_fix_conti @ control_input + self.K_P_3_continue @ (self.C_3mesure_obs @ x_hat_i - y_mesure) + f_nn) 
        f_hat_new = f_hat + Ts*(self.K_I_3_continue @ (self.C_3mesure_obs @ x_hat_i - y_mesure))


        # x_hat_new = self.A_small_fix @ x_hat_i + self.B_small_fix @ control_input + self.K_P_discret @ (self.self.C_3mesure_obs @ x_hat_i - y_mesure)   +  f_nn 

        # # Compute the new disturbance estimate (f_hat_new)
        # f_hat_new = self.K_I_discret @ (self.self.C_3mesure_obs @ x_hat_i - y_mesure)


        x_hat_new = x_hat_new.flatten()
        f_hat_new = f_hat_new.reshape(4)

        return x_hat_new , f_hat_new
    
    def observer_3mesure_2(self,states_hat, delta, a, y_mesure,f_hat,f_nn ):

        Ts=self.constants['Ts']
        control_input = np.array([delta , a])
        x_hat_i = states_hat.reshape(-1, 1)
        control_input = control_input.reshape(-1, 1)
        y_mesure = y_mesure.reshape(-1, 1)
        f_hat = f_hat.reshape(-1, 1)



        x_hat_new = x_hat_i + Ts*(self.A_small_fix_conti @ x_hat_i + self.B_small_fix_conti @ control_input + self.K_P_3_continue @ (self.C_3mesure_obs @ x_hat_i - y_mesure) + f_hat )  
        f_hat_new = f_hat + Ts*(self.K_I_3_continue @ (self.C_3mesure_obs @ x_hat_i - y_mesure))
        
        # # Compute the new state estimate (x_hat_new)
        # # Ensure matrix multiplications
        # x_hat_new = self.A_small_fix_2 @ x_hat_i + self.B_small_fix_2 @ control_input + self.K_P_discret @ (self.self.C_3mesure_obs @ x_hat_i - y_mesure)  + f_hat
        

        # # Compute the new disturbance estimate (f_hat_new)
        # f_hat_new = self.K_I_discret @ (self.self.C_3mesure_obs @ x_hat_i - y_mesure)


        x_hat_new = x_hat_new.flatten()
        f_hat_new = f_hat_new.reshape(4)

        return x_hat_new , f_hat_new