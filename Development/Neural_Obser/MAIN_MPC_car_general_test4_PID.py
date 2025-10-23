
import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'
import time

import numpy as np
import matplotlib.pyplot as plt
from gradient import Gradient_solver
from NN_net import Learning_batch, Net , ModelQueue
from tes_save_excel import Save_Exel
import torch


import scipy.io


import support_files_car_general as sfc_g
import support_files_car_simple as sfc_simple

import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from qpsolvers import solve_qp
np.set_printoptions(suppress=True)

import random


import platform
print("Python " + platform.python_version())
import numpy as np
print("Numpy " + np.__version__)
import matplotlib
print("Matplotlib " + matplotlib.__version__)

# Get the directory where this script is located (Neural_Obser folder)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

trajec_number = 3
version = 1
# Create an object for the support functions.
support=sfc_g.SupportFilesCar(trajec_number , version)
support_simple = sfc_simple.SupportFilesCar()
constants=support.constants

# Load the constant values needed in the main file
Ts=constants['Ts']
outputs=constants['outputs'] # number of outputs (psi, Y)
hz = constants['hz'] # horizon prediction period
time_length=constants['time_length'] # duration of the manoeuvre
inputs=constants['inputs']
x_lim=constants['x_lim']
y_lim=constants['y_lim']
trajectory=constants['trajectory']


'''---------------- Generate the refence signals -------------------------- '''

t=np.zeros((int(time_length/Ts+1)))

for i in range(1,len(t)):
    t[i]=np.round(t[i-1]+Ts,2)

#----------- Number total iterations
sim_length=len(t)

#----------- References
x_dot_ref,y_dot_ref,psi_ref,X_ref,Y_ref , psi_dot_ref =support.trajectory_generator(t)

refSignals=np.zeros(len(X_ref)*outputs)

# Build up the reference signal vector:
# refSignal = [x_dot_ref_0, psi_ref_0, X_ref_0, Y_ref_0, x_dot_ref_1, psi_ref_1, X_ref_1, Y_ref_1, x_dot_ref_2, psi_ref_2, X_ref_2, Y_ref_2, ... etc.]
k=0
for i in range(0,len(refSignals),outputs):
    refSignals[i]=x_dot_ref[k]
    refSignals[i+1]=psi_ref[k]
    refSignals[i+2]=X_ref[k]
    refSignals[i+3]=Y_ref[k]
    k=k+1

Mode_simulation = 'train'  # 'train'   or 'evaluation' 


# D_in = 6  # input neural net
# D_h = 24  # number hidden layer
# D_out = 4 # output  layer


D_in = 6  # input neural net
D_h = 30  # number hidden layer
D_out = 4 # output  layer


def Train(x_dot_ref,y_dot_ref,psi_ref,X_ref,Y_ref , psi_dot_ref  , refSignals , sim_length):

    
    ######################### Gradient and Neural net parameter ########################################

    Type_state_Feedback_map =  {
        1: 'real_state',
        2 : 'state_hat_obs1',
        3 : 'state_hat_obs2'
    }
    Type_state_Feedback = Type_state_Feedback_map.get(1) 

    Type_learning_map =  {
        1: 'normal_Basic',
        2 : 'continous_learning',
        3 : 'learningby_Dict'
    }

    ''' Parameter to tune '''

    new_model = True
    first_model = False

    # --------- Layer model


    # --- continous_learning
    duplicate = False
    # --- continous_learning


    Type_learning = Type_learning_map.get(1)
    LossType = 'mesurement_full' #   'refs' , 'mesurement_full' or  'mesurement_small'
    size_dict = 10
    have_f_tire = True
    have_y_noise = False


    # D_in = 6  # input neural net
    # D_h = 30  # number hidden layer
    # D_out = 4 # output  layer

    # --------- Learning rate
    lr_nn = 0.005
    weight_decay_nn = 0     # 1e-4

    lamda = 0


    # --------- Define batch size
    # Initialize variables for batch training
    batch_size = 5 # Number of ccummulation the loss ,  Number of samples before each training step 


    max_epochs = 1  # Set an upper limit on the number of epochs , training many simulation 

    window_size = 0# smothing the output of the neural


    # --------- Tuning Weight gradient ---------------
    if (LossType == 'refs'):
        Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 2, 10, 10, 50
        # Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 1, 0.4, 1 , 0.4
        weight      = np.array([Weight_v_x , Weight_v_y, Weight_psi,Weight_psi_dot])
    elif (LossType == 'mesurement_small'):
        Weight_v_x,  Weight_psi    = 1,  1
        weight      = np.array([Weight_v_x , Weight_psi])
    else: # mesurement_full
        Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 1, 200, 1, 200
        # Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 0, 0, 0, 0

        weight      = np.array([Weight_v_x , Weight_v_y, Weight_psi,Weight_psi_dot])



    weight_error_traj = np.diag(weight)

    # Weight diff (f_hat - f_nn)


    # print('LossType:',LossType,';Type_learning:',Type_learning,';Type_state_Feedback: ',Type_state_Feedback , ': with_f_tire:' , have_f_tire ,';learn_rate:',lr_nn)
    save_Exel = Save_Exel(LossType,Type_learning,Type_state_Feedback  , weight ,lr_nn , lamda , size_dict , max_epochs)


    ''' End Parameter to tune '''

    carGrad = Gradient_solver(support)



    # Path where the model and optimizer are saved
    model_save_path = os.path.join(SCRIPT_DIR, "trained_data", "trained_model.pt")
    optimizer_save_path = os.path.join(SCRIPT_DIR, "trained_data", "optimizer_state.pt")

    # Ensure the trained_data directory exists
    os.makedirs(os.path.join(SCRIPT_DIR, "trained_data"), exist_ok=True)


    if new_model :
        model_QR = Net(D_in, D_h, D_out)  # Define your neural network model
        optimizer_p = torch.optim.Adam(model_QR.parameters(), lr=lr_nn , weight_decay=weight_decay_nn)
        # optimizer_p = torch.optim.SGD(model_QR.parameters(), lr=lr_nn , weight_decay=weight_decay_nn)


    elif (not new_model and first_model ) :# not new ,but just the first
        PATH0 = os.path.join(SCRIPT_DIR, "trained_data", "initial_nn_model_2.pt")
        model_QR = torch.load(PATH0)
        optimizer_p = torch.optim.Adam(model_QR.parameters(), lr=lr_nn, weight_decay=weight_decay_nn)
    else :
        model_QR = Net(D_in, D_h, D_out)
        optimizer_p = torch.optim.Adam(model_QR.parameters(), lr=lr_nn, weight_decay=weight_decay_nn)
        # Check if there is a saved model to continue training from
        if os.path.exists(model_save_path):
            # Load the model's state_dict
            model_QR.load_state_dict(torch.load(model_save_path))
            print("Loaded saved model for further training.")

            # If optimizer state is also saved, load it to continue training with the same optimizer
            if os.path.exists(optimizer_save_path):
                optimizer_p.load_state_dict(torch.load(optimizer_save_path))
                print("Loaded saved optimizer state.")
        else:
            # Initialize model and optimizer as usual
            PATH0 = os.path.join(SCRIPT_DIR, "trained_data", "initial_nn_model_2.pt")
            model_QR = torch.load(PATH0)
            # # model_QR = Net(D_in, D_h, D_out)
            # # torch.save(model_QR,PATH0)



    # Initialize a queue for neural network models
    model_queue = ModelQueue(D_in =D_in, D_out =D_out , queue_size=3)

    # Add an initial neural network model to the queue


    #----------------- Map LossType to the corresponding ChainRule functions
    loss_function_mapping = {
        'refs': carGrad.ChainRule,
        'mesurement_small': carGrad.ChainRule2,
        'mesurement_full': carGrad.ChainRule3_test
    }

    # Fetch the appropriate loss function and target input
    chainRule_func = loss_function_mapping.get(LossType)



    ######### 'Loop each epoc to better model'

    # Define threshold for error tolerance
    error_threshold = 0.01  # This can be tuned based on your application

    # Initialize variables for error tracking
    error_reached = False


    ###### Params for saturated gradient  
    epsilon0, gmin0 = 1e-4, 1e-4 # smaller the epsilon is, larger the gradient will be. This demonstrates that calculating the gradient requires the inverse of the weightings.


    ################## Lists for storing training data

    Loss        = []
    Epochs      = []
        # Training loop with saving
    for epoch in range(max_epochs):
        running_loss = 0.0

        batch_loss = 0.0  # To accumulate the loss
        batch_count = 0  # To track how many samples have been processed

        # Sum of loss : To accumulate the error over the entire epoch
        sum_loss = 0.0  # only x_hat - x_ref 
        total_error = 0.0  # norm 2  of (x_hat - x_ref)

        error_reached = True  # Assume error is within threshold


        '''---------------- Inittial step of system-------------------------- '''
        #region Inittial_step

        # Load the initial states , and reset for next Epoch
        
        x_dot=x_dot_ref[0]
        y_dot=y_dot_ref[0]
        psi=psi_ref[0]
        psi_dot=0.
        X=X_ref[0]
        Y=Y_ref[0]

        ######################### State System  Inite ########################################

        states = np.array([x_dot,y_dot,psi,psi_dot,X,Y])
        statesTotal = np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
        statesTotal[0][0:len(states)] = states

        ######################### State Observer Inite ########################################
        states_obs=np.array([x_dot,y_dot,psi,psi_dot])

        statesTotal_obs=np.zeros((len(t),len(states_obs))) # It will keep track of all your states during the entire manoeuvre
        statesTotal_obs[0][0:len(states_obs)]=states_obs

        f_hat = np.array([0,0,0,0])

        f_hatTotal_obs=np.zeros((len(t),len(f_hat))) 
        f_hatTotal_obs[0][0:len(f_hat)]=f_hat

        # ------------------ Observer 2 for compare
        states_obs2=np.array([x_dot,y_dot,psi,psi_dot])

        statesTotal_obs2=np.zeros((len(t),len(states_obs2))) # It will keep track of all your states during the entire manoeuvre
        statesTotal_obs2[0][0:len(states_obs2)]=states_obs2

        f_hat2 = np.array([0,0,0,0])

        f_hatTotal_obs2=np.zeros((len(t),len(f_hat2))) 
        f_hatTotal_obs2[0][0:len(f_hat2)]=f_hat2


        # ------------------ Kalmal Observer 

        support.UKF_setUp(states_obs , Ts)

        statesTotal_Kalma_obs=np.zeros((len(t),len(states_obs))) # It will keep track of all your states during the entire manoeuvre
        statesTotal_Kalma_obs[0][0:len(states_obs)]=states_obs

        ######################### End State Observer  Inite ########################################






        ######################### Inite Accelerations ########################################
        x_dot_dot=0.
        y_dot_dot=0.
        psi_dot_dot=0.

        accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
        accelerations_total=np.zeros((len(t),len(accelerations)))

        ######################### Inite Control input ########################################


        # Load the initial input
        U1=0 # Input at t = -0.02 s (steering wheel angle in rad (delta))
        U2=0 # Input at t = -0.02 s (acceleration in m/s^2 (a))
        UTotal=np.zeros((len(t),2)) # To keep track all your inputs over time
        UTotal[0][0]=U1
        UTotal[0][1]=U2

        # Initiate the controller - simulation loops
        hz = constants['hz'] # horizon prediction period

        k=0
        du=np.zeros((inputs*hz,1))





        ######################### Inite for output of Neural Net  ########################################
        f_uk=np.array([0,0,0,0])
        f_uk_2=np.array([0,0,0,0])
        f_uk_3=np.array([0,0,0,0])



        f_uk_Save=np.zeros((len(t),len(f_uk))) # It will keep track of all your states during the entire manoeuvre
        f_uk_Save[0][0:len(f_uk)]=f_uk
        
        f_uk_2_Save=np.zeros((len(t),len(f_uk_2))) # It will keep track of all your states during the entire manoeuvre
        f_uk_2_Save[0][0:len(f_uk_2)]=f_uk_2

        f_uk_3_Save=np.zeros((len(t),len(f_uk_3))) # It will keep track of all your states during the entire manoeuvre
        f_uk_3_Save[0][0:len(f_uk_3)]=f_uk_3


        # first - ouput value of NN 

        f_nn = np.zeros((D_out, 1))
        f_nn_Save = np.zeros((D_out, len(t)))

        # gradient of x  wrt  f  == sensitivity variable , Shape = ( dim X , dim f  ) = (4 , 4)
        dx_df = np.zeros((len(states_obs) , D_out))
        # dx_df_Total =  np.zeros((len(t),D_out))


        ######################### Save for gradient  ########################################

        # dldf = np.array([0,0,0,0])  # Shape ( 1  ,  D_out)

        dldf_Total = np.zeros((len(t),D_out)) 

        # dldf_hatTotal_obs[0][0:len(dldf)]=dldf

        loss_nn_Total = []

        #endregion Inittial_step

        # Rest optimizer
        optimizer_p = torch.optim.Adam(model_QR.parameters(), lr=lr_nn, weight_decay=weight_decay_nn)

        learning_batch = Learning_batch(size_dict)



        ######################### Uncertainty  ########################################

        w_uncertain_f_save=np.zeros((len(t),1)) 

        w_uncertain_r_save=np.zeros((len(t),1)) 

        # w_uncertain_f_save[0][0] = 0
        # w_uncertain_r_save[0][0] = 0

        # Initialize previous time
        previous_time = time.time()

        gradient_dx_df_storage = []

        # Initialize an empty list to store loss values for moving average
        loss_history = []


        '''---------------- loop of system-------------------------- '''
        for i in range(0,sim_length-1):


            # Measure current time and compute time difference
            current_time = time.time()
            time_diff = current_time - previous_time


            ######################### Generate Trajectory ########################################

            # From the refSignals vector, only extract the reference values from your [current sample (NOW) + Ts] to [NOW+horizon period (hz)]
            # Example: Ts=0.1 seconds, t_now is 3 seconds, hz = 15 samples, so from refSignals vectors, you move the elements to vector r:
            # r=[x_dot_ref_3.1, psi_ref_3.1, X_ref_3.1, Y_ref_3.1, x_dot_ref_3.2, psi_ref_3.2, X_ref_3.2, Y_ref_3.2, ... , x_dot_ref_4.5, psi_ref_4.5, X_ref_4.5, Y_ref_4.5]
            # With each loop, it all shifts by 0.1 second because Ts=0.1 s
            k=k+outputs
            if k+outputs*hz<=len(refSignals):
                r=refSignals[k:k+outputs*hz]
            else:
                r=refSignals[k:len(refSignals)]
                hz=hz-1


            ######################### Simulate noisy measurements ########################################

            # measurement_noise =0  # Simulated noise for measurements (psi, Y)
            if have_y_noise : 
                measurement_noise = np.random.randn(2) * 0.05  # Simulated noise for measurements (psi, Y)
            else:
                measurement_noise =0  # Simulated noise for measurements (psi, Y)
            measurements = support.Cd_small @ states 
            measurements_2 = support.Cd_small @ states + measurement_noise


            # measurements = support.Cd_small_3mesure @ states + measurement_noise

            # measurements = measurements.reshape(-1, 1)


            f_uk = states[0:4] -  states_obs   
            f_uk_Save[i][0:len(f_uk)] = f_uk # np.squeeze(np.asarray(f_uk_tempo))

            
            f_uk_2 = states[0:4] -  states_obs2  
            f_uk_2_Save[i][0:len(f_uk_2)] = f_uk_2 

            ######################### Neural input########################################
            # ------- Neural input
            nn_input= np.reshape(np.array([states_obs[0], states_obs[1],states_obs[2], states_obs[3] ,U1,U2 ]),(D_in,1))

            # ------- Predict to get f_nn based on the Neural input 
            if (Type_learning == 'continous_learning'):
                ############# New thing Test Huy

                # After some time , we add new model to the Queue , and use the newest to train
                if i%20==0 :
                    model_queue.add_queue( duplicate, D_in, D_h, D_out)


                # Predict using the weighted sum of models in the queue
                f_nn = model_queue.predict(nn_input).detach().numpy()



            ############# New thing Test Huy
            else:
                # ------- Put the input into the model , and get the output , and chose Smothing the output or not
                if window_size > 1 and i > window_size :

                    f_nn = model_QR(nn_input).detach().numpy()
                    f_nn = support.moving_average_matrix(f_nn ,  f_nn_Save[:,i-window_size+1:i] , window_size )
                else:
                    f_nn = model_QR(nn_input).detach().numpy()
                    # f_nn = np.zeros((D_out, 1))
            


            # f_nn = support.constrain_the_fnn(f_nn)
            f_nn_Save[:,i:i+1] = f_nn




            ######################### Update Observer ########################################

            states_obs_new , f_hat_new=  support.observer(states_obs,U1,U2 , measurements, f_hat , f_nn  )

            states_obs2_new , f_hat2_new=  support.observer2(states_obs2,U1,U2 , measurements_2, f_hat2 ,  0 )


            # states_obs_new , f_hat_new=  support.observer_3mesure_1(states_obs,U1,U2 , measurements, f_hat , f_nn  )

            # states_obs2_new , f_hat2_new=  support.observer_3mesure_2(states_obs2,U1,U2 , measurements, f_hat2 ,  0 )
            
            

            # states_Kalma_obs =  support.UKF_estimate(U1,U2 , measurements)
            # statesTotal_Kalma_obs[i+1][0:len(states_Kalma_obs)]  = states_Kalma_obs 


            
            # -------- Save observer 1
            # states_obs_new have shape (1,4) so not fit to shape (4,)
            # need to reshape that
            states_obs = np.squeeze(np.asarray(states_obs_new)) # rember that pain, so hurt
            f_hat = np.squeeze(np.asarray(f_hat_new))


            statesTotal_obs[i+1][0:len(states_obs)]  = states_obs
            f_hatTotal_obs[i+1][0:len(f_hat)] = f_hat

            # -------- Save observer 2 
            states_obs2 = np.squeeze(np.asarray(states_obs2_new)) # rember that pain, so hurt
            f_hat2 = np.squeeze(np.asarray(f_hat2_new))


            statesTotal_obs2[i+1][0:len(states_obs)]  = states_obs2
            f_hatTotal_obs2[i+1][0:len(f_hat2)] = f_hat2

            ######################### Get State for Controller #############################################


            if i > 200:
                # new_state_obs = np.concatenate((states_obs2, states[-2:]))
                new_state_obs = states

            else:
                new_state_obs = states


            ######################### PID #############################################

            PID_switch = 0

            if PID_switch == 1:

                Kp_yaw=7
                Kd_yaw=3
                Ki_yaw=5

                Kp_Y=7
                Kd_Y=3
                Ki_Y=5

                Kp_X = 5
                Kd_X = 2
                Ki_X = 2

                Kp_vy = 1
                Kd_vy = 0.02
                Ki_vy = 0.02

                Kp_vx = 5
                Kd_vx = 1
                Ki_vx = 1
                if i == 0:
                    e_int_pid_yaw = 0
                    e_int_pid_Y = 0
                    e_int_pid_accel_x = 0
                    e_int_pid_accel_y = 0
                    e_int_pid_X = 0  # Initialize for X position control
                if i > 0:
                    # Yaw control
                    e_pid_yaw_im1 = psi_ref[i-1] - old_states[2]
                    e_pid_yaw_i = psi_ref[i] - states[2]
                    e_dot_pid_yaw = (e_pid_yaw_i - e_pid_yaw_im1) / Ts
                    e_int_pid_yaw = e_int_pid_yaw + (e_pid_yaw_im1 + e_pid_yaw_i) / 2 * Ts

                    U1_yaw = Kp_yaw * e_pid_yaw_i + Kd_yaw * e_dot_pid_yaw + Ki_yaw * e_int_pid_yaw

                    # Lateral motion control (Y-axis)
                    e_pid_Y_im1 = Y_ref[i-1] - old_states[5]
                    e_pid_Y_i = Y_ref[i] - states[5]
                    e_dot_pid_Y = (e_pid_Y_i - e_pid_Y_im1) / Ts
                    e_int_pid_Y = e_int_pid_Y + (e_pid_Y_im1 + e_pid_Y_i) / 2 * Ts

                    U1_Y = Kp_Y * e_pid_Y_i + Kd_Y * e_dot_pid_Y + Ki_Y * e_int_pid_Y

                    # U1 combined control (Yaw + Y-axis)
                    U1 = U1_yaw + U1_Y

                    ### X position control (generate desired velocity)
                    e_pid_X_im1 = X_ref[i-1] - old_states[4]  # Error in position
                    e_pid_X_i = X_ref[i] - states[4]          # Current position error
                    e_dot_pid_X = (e_pid_X_i - e_pid_X_im1) / Ts
                    e_int_pid_X = e_int_pid_X + (e_pid_X_im1 + e_pid_X_i) / 2 * Ts

                    v_desired_X = Kp_X * e_pid_X_i + Kd_X * e_dot_pid_X + Ki_X * e_int_pid_X

                    ### Velocity control (track the desired velocity)
                    e_pid_vx_im1 = v_desired_X - old_states[0]  # Error in desired velocity
                    e_pid_vx_i = v_desired_X - states[0]        # Current velocity error
                    e_dot_pid_vx = (e_pid_vx_i - e_pid_vx_im1) / Ts
                    e_int_pid_accel_x = e_int_pid_accel_x + (e_pid_vx_im1 + e_pid_vx_i) / 2 * Ts

                    U2_accel_x = Kp_vx * e_pid_vx_i + Kd_vx * e_dot_pid_vx + Ki_vx * e_int_pid_accel_x

                    # ### Lateral velocity control (Y-axis for velocity)
                    # e_pid_vy_im1 = y_dot_ref[i-1] - old_states[1]
                    # e_pid_vy_i = y_dot_ref[i] - states[1]
                    # e_dot_pid_vy = (e_pid_vy_i - e_pid_vy_im1) / Ts
                    # e_int_pid_accel_y = e_int_pid_accel_y + (e_pid_vy_im1 + e_pid_vy_i) / 2 * Ts

                    # U2_accel_y = Kp_vy * e_pid_vy_i + Kd_vy * e_dot_pid_vy + Ki_vy * e_int_pid_accel_y

                    # U2 final control (acceleration only for longitudinal + lateral control)
                    U2 = U2_accel_x 


                old_states = states
            else :

                ######################### Update Controller MPC ########################################
                # State put into the control 
                # Create new state by combining states_obs and the last two elements of states


                # Generate the discrete state space matrices
                Ad,Bd,Cd,Dd=support.state_space(new_state_obs,U1,U2)


                # Generate the augmented current state and the reference vector
                x_aug_t=np.transpose([np.concatenate((new_state_obs,[U1,U2]),axis=0)])


                # Generate the compact simplification matrices for the cost function
                Hdb,Fdbt,Cdb,Adc,G,ht=support.mpc_simplification(Ad,Bd,Cd,Dd,hz,x_aug_t,du)
                ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)
                
                # Hdb=np.array([[10.,1.],[1.,10.]])
                # ft=np.array([0.,0.])
                # G=np.array([[-1.,0.],[0.,-1.],[-1.4,-1.]])
                # ht=np.array([-0.05,-0.07,-0.15])
                ################# Constraints #####################
                try:
                    du=solve_qp(Hdb,ft,G,ht,solver="cvxopt")
                    du=np.transpose([du])
                    # print(du)
                    # exit()
                except ValueError as ve:
                    print(Hdb)
                    print(ft)
                    print(G)
                    print(ht)
                    print(Adc)
                    print(x_aug_t)
                    print(du)
                    print(i)
                    break;


            #############################################################################



            # Update the real inputs

            U1=U1+du[0][0]
            U2=U2+du[1][0]

            # if compensate:


            UTotal[i+1][0]=U1
            UTotal[i+1][1]=U2



            ######################### Update the real system ####################################

            # w_f = 10 * random.random() * np.sin(i * 10)
            # w_r = 10 * random.random() * np.cos(i * 10)

            if have_f_tire :
                w_f , w_r = support.dis_noise(states,U1,U2)
                    
                # w_f += 5 * random.random() * np.sin(i * 10)
                # w_r += 5 * random.random() * np.cos(i * 10)
            else : 
                w_f = 0
                w_r = 0






            w_uncertain_f_save[i][0] = w_f
            w_uncertain_r_save[i][0] = w_r





            states,x_dot_dot,y_dot_dot,psi_dot_dot=support.open_loop_new_states_RK4(states,U1,U2 , w_r, w_f)
            statesTotal[i+1][0:len(states)] = states


            # ------------ F_uk here is D@[w_f] , [w_r]
            # w_f_obs , w_r_obs = support.dis_noise(states_obs,U1,U2)

            A_conti, B_conti, C_conti, D_conti = support.matrix_state_space_simple_observer(states , U1 , U2 )
            f_uk_tempo = (D_conti @ np.array([ [w_f] , [w_r] ]))
            f_uk_3_Save[i+1][0:len(f_uk)] = np.squeeze(np.asarray(f_uk_tempo)) # 


            # ------------ New f_uk is state - obs2

            ######################### Accelerations ####################################
            accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
            accelerations_total[i+1][0:len(accelerations)]=accelerations




            ######################### Gradient and TRain network ########################################
            # # New ref for calculate the gradient


            #-----------Policy update based on gradient descent------------#
            dx_df  = carGrad.GradientSolver_general(dx_df) # expected shape (4 , D_out ) = nominateur , denominateur

            # dx_df  = carGrad.GradientSolver_general_3mesure(dx_df) # expected shape (4 , D_out ) = nominateur , denominateur
            gradient_dx_df_storage.append(dx_df)


            # TODO : states_obs is a head one step with measurement = C*x , so need to use statesTotal_obs to get the last state 

            statesTotal_obs_k = states_obs  #  statesTotal_obs[i]   or states_obs
            # nn_input= np.reshape(np.array([states_obs[0], states_obs[1],states_obs[2], states_obs[3] ,U1,U2 ]),(D_in,1))  
            # f_nn = model_QR(nn_input).detach().numpy()

            if (LossType == 'refs'):
                # New ref for calculate the gradient
                input_target = np.array([[x_dot_ref[i]],
                                [y_dot_ref[i]],
                                [psi_ref[i]],
                                [psi_dot_ref[i]]])
            elif (LossType == 'mesurement_small'):
                input_target = measurements
            else: #LossType == 'mesurement_full':
                input_target = support.Cd_Full @ states


            if (Type_learning == 'learningby_Dict'):
                ##--------------------------- Test_Huy

                # Add new feature to dictionary , 
                # Giả sử là case LossType == 'mesurement_full' => taget1 = measurement , target2 = f_hat , save also the time  

                #TODO : Test 
                learning_batch.add_feature(nn_input, f_nn ,statesTotal_obs_k, input_target,f_uk , dx_df ,i)

                # learning_batch.add_feature(nn_input,statesTotal_obs_k, input_target,f_hat,i)
                
                total_loss_track = 0.0
                total_dldf = 0.0
                count = 0            

                # dldf = np.zeros((1, D_out))

                # Loop through each stored feature and calculate losses
                for feature,f_nn_tempo,states_hat, target1, target2_f_uk,dx_df_tempo ,times_data in learning_batch.feature_dict:
                    # f_nn_tempo = model_QR(feature).detach().numpy()

                    # states_hat_new_tempo , f_hat_new_tempo=  support.observer(states_hat,U1,U2 , measurements, f_hat , f_nn_tempo  )
                    
                    # states_hat_new_tempo = np.squeeze(np.asarray(states_hat_new_tempo)) # rember that pain, so hurt
                    # f_hat_new_tempo = np.squeeze(np.asarray(f_hat_new_tempo))

                    # dldf, loss_track = chainRule_func(target1, states_hat_new_tempo, dx_df, f_hat_new_tempo, f_nn_tempo, weight_error_traj, lamda)

                    dldf_tempo, loss_track_tempo = chainRule_func(target1, states_hat, dx_df_tempo, target2_f_uk, f_nn_tempo, weight_error_traj, lamda)

                    # Loss calculation
                    total_loss_track += loss_track_tempo
                    total_dldf += dldf_tempo
                    count += 1

                #    Calculate the mean if there are losses; otherwise return zero
                loss_track = total_loss_track / count if count > 0 else 0
                dldf = total_dldf / count
                # dldf = total_dldf 
                loss_nn_pytorch = model_QR.myloss(model_QR(nn_input), dldf)

                # Backward pass: compute gradients and update network parameters
                optimizer_p.zero_grad()
                loss_nn_pytorch.backward()
                optimizer_p.step()

                dldf_Total[i+1][:] = dldf 
                loss_nn_Total.append(loss_nn_pytorch.detach().numpy()) 

            elif (Type_learning == 'normal_Basic'):
                
                dldf, loss_track = chainRule_func(input_target, statesTotal_obs_k, dx_df, f_uk_tempo, f_nn, weight_error_traj, lamda)
                
                # in paper
                # dldf, loss_track = chainRule_func(input_target, statesTotal_obs_k, dx_df, f_hat, f_nn, weight_error_traj, lamda)

                loss_nn_pytorch   = model_QR.myloss(model_QR(nn_input), dldf)


                # ------------- TEST 
                # Append the current loss to the history
                # loss_history.append(loss_nn_pytorch.item())  # Convert tensor to scalar if using PyTorch

                # # Apply the moving average with however many losses are available
                # window_size = 2  # Define the window size for smoothing
                # available_losses = loss_history[-window_size:]  # Get the available losses (could be < window_size)
                # smoothed_loss = support.moving_average(available_losses, len(available_losses), weight_type='exponential')[-1]  # Latest smoothed value

                # # Train with the smoothed loss
                # loss_for_training = torch.tensor(smoothed_loss, requires_grad=True)  # Convert to tensor if using PyTorch
                # loss_for_training.backward()
                # optimizer_p.step()
                # # Clear gradients if using PyTorch
                # optimizer_p.zero_grad()
                # ------------- TEST 


                                
                # # Accumulate the loss for batch training
                batch_loss += loss_nn_pytorch
                batch_count += 1

                # If we've accumulated enough samples for one batch, perform the training step
                if batch_count == batch_size:
                    # Compute the mean loss over the batch
                    batch_loss = batch_loss / batch_size


                    # Backward pass: compute gradients and update network parameters
                    optimizer_p.zero_grad()
                    batch_loss.backward()
                    optimizer_p.step()

                    # Reset the batch loss and count for the next batch
                    batch_loss = 0.0
                    batch_count = 0
        
                # # Save for plot
                
                dldf_Total[i+1][:] = dldf 
                loss_nn_Total += [loss_nn_pytorch.detach().numpy()] 

            else :
                ##--------------------------- Test_Huy
                dldf, loss_track = chainRule_func(input_target, statesTotal_obs_k, dx_df, f_uk_2, f_nn, weight_error_traj, lamda)

                loss_nn_pytorch = model_queue.train_latest_model(dldf , nn_input, epochs = batch_size, lr=lr_nn  )

                dldf_Total[i+1][:] = dldf 
                loss_nn_Total += [loss_nn_pytorch.detach().numpy()] 





            ######################### Check for Convergence ####################################

            # Simple loss_track =  x_hat - x_ref   
            loss_track = np.reshape(loss_track,(1))
            sum_loss += loss_track


            # Update the previous time for the next iteration
            previous_time = current_time


            # This is to monitor the progress of the simulation
            if i%500==0:
                print('Learning_iteration:',epoch,'Progress: '+str(round(i/sim_length*100,2))+'%')


        


        '''---------------- Save the model after each Epoch-------------------------- '''

        mean_loss = sum_loss/sim_length
        # Average error over the entire epoch
        # avg_error = total_error / sim_length

        Loss += [mean_loss]
        Epochs += [epoch] 


        # Save the model and optimizer after each epoch
        torch.save(model_QR.state_dict(), model_save_path)
        torch.save(optimizer_p.state_dict(), optimizer_save_path)
        # print(f"Model saved at epoch {epoch}.")

        print('Finish:',epoch , 'epoch : ','mean_loss=',mean_loss)

        save_Exel.Add("Epoch",epoch)
        save_Exel.Add("mean_loss", mean_loss)
        save_Exel.Save()

        gradient_dx_df_storage_array = np.array(gradient_dx_df_storage)

        # Define your desired folder path
        folder_save_matlab_data = os.path.join(SCRIPT_DIR, "data_test1")

        # Ensure the folder exists (creates it if it doesn't exist)
        os.makedirs(folder_save_matlab_data, exist_ok=True)

        # Save the final data after training is complete
        scipy.io.savemat(os.path.join(folder_save_matlab_data,f'test2_training_data_{epoch}.mat'), {
        'statesTotal': statesTotal,
        'statesTotal_obs': statesTotal_obs,
        'statesTotal_obs2': statesTotal_obs2,
        'statesTotal_Kalma_obs': statesTotal_Kalma_obs,
        'f_hatTotal_obs': f_hatTotal_obs,
        'f_hatTotal_obs2': f_hatTotal_obs2,
        'f_nn_Save': f_nn_Save,
        'f_uk_Save': f_uk_Save,
        'f_uk_2_Save': f_uk_2_Save,
        'f_uk_3_Save': f_uk_3_Save,
        'UTotal': UTotal,
        'x_dot_ref': x_dot_ref,
        'y_dot_ref': y_dot_ref,
        'psi_ref': psi_ref,
        'X_ref': X_ref,
        'Y_ref': Y_ref ,
        'psi_dot_ref': psi_dot_ref,
        'accelerations_total': accelerations_total,
        't':t,
        'dldf_Total': dldf_Total,
        'loss_nn_Total' : loss_nn_Total,
        'w_uncertain_f_save':w_uncertain_f_save,
        'w_uncertain_r_save':w_uncertain_r_save,
        'gradient_dx_df_storage' : gradient_dx_df_storage_array
        })

    print('Finsh !!! ')

def Evaluation(x_dot_ref,y_dot_ref,psi_ref,X_ref,Y_ref , psi_dot_ref  , refSignals , sim_length):

    Type_state_Feedback_map =  {
        1: 'real_state',
        2 : 'state_hat_obs1',
        3 : 'state_hat_obs2'
    }
    Type_state_Feedback = Type_state_Feedback_map.get(1) 

    Type_learning_map =  {
        1: 'normal_Basic',
        2 : 'continous_learning',
        3 : 'learningby_Dict'
    }

    ''' Parameter to tune '''



    # --------- Layer model


    # --- continous_learning
    duplicate = False
    # --- continous_learning


    Type_learning = Type_learning_map.get(1)
    LossType = 'mesurement_full' #   'refs' , 'mesurement_full' or  'mesurement_small'
    size_dict = 10
    have_f_tire = True
    have_y_noise = False


    # D_in = 6  # input neural net
    # D_h = 24  # number hidden layer
    # D_out = 4 # output  layer

    # --------- Learning rate
    lr_nn = 0.004
    weight_decay_nn = 0     # 1e-4

    lamda = 0


    # --------- Define batch size
    # Initialize variables for batch training
    batch_size = 5  # Number of ccummulation the loss ,  Number of samples before each training step 


    max_epochs = 1  # Set an upper limit on the number of epochs , training many simulation 

    window_size = 0 # smothing the output of the neural


    # --------- Tuning Weight gradient ---------------
    if (LossType == 'refs'):
        Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 2, 10, 10, 50
        # Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 1, 0.4, 1 , 0.4
        weight      = np.array([Weight_v_x , Weight_v_y, Weight_psi,Weight_psi_dot])
    elif (LossType == 'mesurement_small'):
        Weight_v_x,  Weight_psi    = 1,  1
        weight      = np.array([Weight_v_x , Weight_psi])
    else: # mesurement_full
        Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 1, 200, 10, 200
        # Weight_v_x, Weight_v_y , Weight_psi , Weight_psi_dot   = 0, 0, 0, 0

        weight      = np.array([Weight_v_x , Weight_v_y, Weight_psi,Weight_psi_dot])



    weight_error_traj = np.diag(weight)



    # print('LossType:',LossType,';Type_learning:',Type_learning,';Type_state_Feedback: ',Type_state_Feedback , ': with_f_tire:' , have_f_tire ,';learn_rate:',lr_nn)
    save_Exel = Save_Exel(LossType,Type_learning,Type_state_Feedback  , weight ,lr_nn , lamda , size_dict , max_epochs)


    ''' End Parameter to tune '''

    carGrad = Gradient_solver(support)



    # Path where the model and optimizer are saved
    model_save_path = os.path.join(SCRIPT_DIR, "trained_data", "trained_model.pt")
    optimizer_save_path = os.path.join(SCRIPT_DIR, "trained_data", "optimizer_state.pt")

    # Ensure the trained_data directory exists
    os.makedirs(os.path.join(SCRIPT_DIR, "trained_data"), exist_ok=True)

    model_QR = Net(D_in, D_h, D_out)
    model_QR.load_state_dict(torch.load(model_save_path))
    # model_QR = torch.load(model_save_path)



    # Initialize a queue for neural network models
    model_queue = ModelQueue(D_in =D_in, D_out =D_out , queue_size=3)

    # Add an initial neural network model to the queue


    #----------------- Map LossType to the corresponding ChainRule functions
    loss_function_mapping = {
        'refs': carGrad.ChainRule,
        'mesurement_small': carGrad.ChainRule2,
        'mesurement_full': carGrad.ChainRule3_test
    }

    # Fetch the appropriate loss function and target input
    chainRule_func = loss_function_mapping.get(LossType)



    ######### 'Loop each epoc to better model'

    # Define threshold for error tolerance
    error_threshold = 0.01  # This can be tuned based on your application

    # Initialize variables for error tracking
    error_reached = False


    ###### Params for saturated gradient  
    epsilon0, gmin0 = 1e-4, 1e-4 # smaller the epsilon is, larger the gradient will be. This demonstrates that calculating the gradient requires the inverse of the weightings.


    ################## Lists for storing training data

    Loss        = []
        # Training loop with saving

    sum_loss = 0.0  # only x_hat - x_ref 



    '''---------------- Inittial step of system-------------------------- '''
    #region Inittial_step

    # Load the initial states , and reset for next Epoch
    
    x_dot=x_dot_ref[0]
    y_dot=y_dot_ref[0]
    psi=psi_ref[0]
    psi_dot=0.
    X=X_ref[0]
    Y=Y_ref[0]

    ######################### State System  Inite ########################################

    states = np.array([x_dot,y_dot,psi,psi_dot,X,Y])
    statesTotal = np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
    statesTotal[0][0:len(states)] = states

    ######################### State Observer Inite ########################################
    states_obs=np.array([x_dot,y_dot,psi,psi_dot])

    statesTotal_obs=np.zeros((len(t),len(states_obs))) # It will keep track of all your states during the entire manoeuvre
    statesTotal_obs[0][0:len(states_obs)]=states_obs

    f_hat = np.array([0,0,0,0])

    f_hatTotal_obs=np.zeros((len(t),len(f_hat))) 
    f_hatTotal_obs[0][0:len(f_hat)]=f_hat

    # ------------------ Observer 2 for compare
    states_obs2=np.array([x_dot,y_dot,psi,psi_dot])

    statesTotal_obs2=np.zeros((len(t),len(states_obs2))) # It will keep track of all your states during the entire manoeuvre
    statesTotal_obs2[0][0:len(states_obs2)]=states_obs2

    f_hat2 = np.array([0,0,0,0])

    f_hatTotal_obs2=np.zeros((len(t),len(f_hat2))) 
    f_hatTotal_obs2[0][0:len(f_hat2)]=f_hat2


    # ------------------ Kalmal Observer 

    support.UKF_setUp(states_obs , Ts)

    statesTotal_Kalma_obs=np.zeros((len(t),len(states_obs))) # It will keep track of all your states during the entire manoeuvre
    statesTotal_Kalma_obs[0][0:len(states_obs)]=states_obs

    ######################### End State Observer  Inite ########################################






    ######################### Inite Accelerations ########################################
    x_dot_dot=0.
    y_dot_dot=0.
    psi_dot_dot=0.

    accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
    accelerations_total=np.zeros((len(t),len(accelerations)))

    ######################### Inite Control input ########################################


    # Load the initial input
    U1=0 # Input at t = -0.02 s (steering wheel angle in rad (delta))
    U2=0 # Input at t = -0.02 s (acceleration in m/s^2 (a))
    UTotal=np.zeros((len(t),2)) # To keep track all your inputs over time
    UTotal[0][0]=U1
    UTotal[0][1]=U2

    # Initiate the controller - simulation loops
    hz = constants['hz'] # horizon prediction period

    k=0
    du=np.zeros((inputs*hz,1))


    ######################### Save for animation  ########################################

    # Arrays for the animation - every 5th state goes in there (once in 0.1 seconds, because Ts=0.02 seconds)
    t_ani=[]
    x_dot_ani=[]
    psi_ani=[]
    X_ani=[]
    Y_ani=[]
    delta_ani=[]

    ######################### Inite for output of Neural Net  ########################################
    f_uk=np.array([0,0,0,0])
    f_uk_2=np.array([0,0,0,0])


    f_uk_Save=np.zeros((len(t),len(f_uk))) # It will keep track of all your states during the entire manoeuvre
    f_uk_Save[0][0:len(f_uk)]=f_uk
    
    f_uk_2_Save=np.zeros((len(t),len(f_uk_2))) # It will keep track of all your states during the entire manoeuvre
    f_uk_2_Save[0][0:len(f_uk_2)]=f_uk_2


    # first - ouput value of NN 

    f_nn = np.zeros((D_out, 1))
    f_nn_Save = np.zeros((D_out, len(t)))

    # gradient of x  wrt  f  == sensitivity variable , Shape = ( dim X , dim f  ) = (4 , 4)
    dx_df = np.zeros((len(states_obs) , D_out))
    # dx_df_Total =  np.zeros((len(t),D_out))


    ######################### Save for gradient  ########################################

    # dldf = np.array([0,0,0,0])  # Shape ( 1  ,  D_out)

    dldf_Total = np.zeros((len(t),D_out)) 

    # dldf_hatTotal_obs[0][0:len(dldf)]=dldf


    #endregion Inittial_step

    # Rest optimizer

    learning_batch = Learning_batch(size_dict)



    ######################### Uncertainty  ########################################

    w_uncertain_f_save=np.zeros((len(t),1)) 

    w_uncertain_r_save=np.zeros((len(t),1)) 

    w_uncertain_f_save[0][0] = 0
    w_uncertain_r_save[0][0] = 0

    # Initialize previous time
    previous_time = time.time()

    gradient_dx_df_storage = []


    '''---------------- loop of system-------------------------- '''
    for i in range(0,sim_length-1):


        # Measure current time and compute time difference
        current_time = time.time()
        time_diff = current_time - previous_time


        ######################### Generate Trajectory ########################################

        # From the refSignals vector, only extract the reference values from your [current sample (NOW) + Ts] to [NOW+horizon period (hz)]
        # Example: Ts=0.1 seconds, t_now is 3 seconds, hz = 15 samples, so from refSignals vectors, you move the elements to vector r:
        # r=[x_dot_ref_3.1, psi_ref_3.1, X_ref_3.1, Y_ref_3.1, x_dot_ref_3.2, psi_ref_3.2, X_ref_3.2, Y_ref_3.2, ... , x_dot_ref_4.5, psi_ref_4.5, X_ref_4.5, Y_ref_4.5]
        # With each loop, it all shifts by 0.1 second because Ts=0.1 s
        k=k+outputs
        if k+outputs*hz<=len(refSignals):
            r=refSignals[k:k+outputs*hz]
        else:
            r=refSignals[k:len(refSignals)]
            hz=hz-1


        ######################### Simulate noisy measurements ########################################

        # measurement_noise =0  # Simulated noise for measurements (psi, Y)
        if have_y_noise : 
            measurement_noise = np.random.randn(2) * 0.005  # Simulated noise for measurements (psi, Y)
        else:
            measurement_noise =0  # Simulated noise for measurements (psi, Y)
        measurements = support.Cd_small @ states + measurement_noise
        # measurements = measurements.reshape(-1, 1)


        # f_uk = states_obs -  states[0:4] 
        # f_uk_Save[i][0:len(f_uk)] = f_uk # np.squeeze(np.asarray(f_uk_tempo))

        
        # f_uk_2 = states_obs2 -  states[0:4] 
        # f_uk_2_Save[i][0:len(f_uk_2)] = f_uk_2 


        f_uk = states[0:4] -  states_obs   
        f_uk_Save[i][0:len(f_uk)] = f_uk # np.squeeze(np.asarray(f_uk_tempo))

        
        f_uk_2 = states[0:4] -  states_obs2  
        f_uk_2_Save[i][0:len(f_uk_2)] = f_uk_2 

        ######################### Neural input########################################
        # ------- Neural input
        nn_input= np.reshape(np.array([states_obs[0], states_obs[1],states_obs[2], states_obs[3] ,U1,U2 ]),(D_in,1))

        # ------- Predict to get f_nn based on the Neural input 
        if (Type_learning == 'continous_learning'):
            ############# New thing Test Huy

            # After some time , we add new model to the Queue , and use the newest to train
            if i%20==0 :
                model_queue.add_queue( duplicate, D_in, D_h, D_out)


            # Predict using the weighted sum of models in the queue
            f_nn = model_queue.predict(nn_input).detach().numpy()
        ############# New thing Test Huy
        else:
            # ------- Put the input into the model , and get the output , and chose Smothing the output or not
            if window_size > 1 and i > window_size :

                f_nn = model_QR(nn_input).detach().numpy()
                f_nn = support.moving_average_matrix(f_nn ,  f_nn_Save[:,i-window_size+1:i] )
            else:
                f_nn = (model_QR(nn_input)).detach().numpy()
                # f_nn = np.zeros((D_out, 1))
        


        # f_nn = support.constrain_the_fnn(f_nn)
        f_nn_Save[:,i:i+1] = f_nn




        ######################### Update Observer ########################################

        states_obs_new , f_hat_new=  support.observer(states_obs,U1,U2 , measurements, f_uk , f_nn  )

        states_obs2_new , f_hat2_new=  support.observer2(states_obs2,U1,U2 , measurements, f_hat2 ,  0 )
        

        # states_Kalma_obs =  support.UKF_estimate(U1,U2 , measurements)
        # statesTotal_Kalma_obs[i+1][0:len(states_Kalma_obs)]  = states_Kalma_obs 


        
        # -------- Save observer 1
        # states_obs_new have shape (1,4) so not fit to shape (4,)
        # need to reshape that
        states_obs = np.squeeze(np.asarray(states_obs_new)) # rember that pain, so hurt
        f_hat = np.squeeze(np.asarray(f_hat_new))


        statesTotal_obs[i+1][0:len(states_obs)]  = states_obs
        f_hatTotal_obs[i+1][0:len(f_hat)] = f_hat

        # -------- Save observer 2 
        states_obs2 = np.squeeze(np.asarray(states_obs2_new)) # rember that pain, so hurt
        f_hat2 = np.squeeze(np.asarray(f_hat2_new))


        statesTotal_obs2[i+1][0:len(states_obs)]  = states_obs2
        f_hatTotal_obs2[i+1][0:len(f_hat2)] = f_hat2

        ######################### Get State for Controller #############################################


        if i > 200:
            # new_state_obs = np.concatenate((states_obs2, states[-2:]))
            new_state_obs = states

        else:
            new_state_obs = states


        ######################### PID #############################################

        PID_switch = 0

        if PID_switch == 1:

            Kp_yaw=7
            Kd_yaw=3
            Ki_yaw=5

            Kp_Y=7
            Kd_Y=3
            Ki_Y=5

            Kp_X = 5
            Kd_X = 2
            Ki_X = 2

            Kp_vy = 1
            Kd_vy = 0.02
            Ki_vy = 0.02

            Kp_vx = 5
            Kd_vx = 1
            Ki_vx = 1
            if i == 0:
                e_int_pid_yaw = 0
                e_int_pid_Y = 0
                e_int_pid_accel_x = 0
                e_int_pid_accel_y = 0
                e_int_pid_X = 0  # Initialize for X position control
            if i > 0:
                # Yaw control
                e_pid_yaw_im1 = psi_ref[i-1] - old_states[2]
                e_pid_yaw_i = psi_ref[i] - states[2]
                e_dot_pid_yaw = (e_pid_yaw_i - e_pid_yaw_im1) / Ts
                e_int_pid_yaw = e_int_pid_yaw + (e_pid_yaw_im1 + e_pid_yaw_i) / 2 * Ts

                U1_yaw = Kp_yaw * e_pid_yaw_i + Kd_yaw * e_dot_pid_yaw + Ki_yaw * e_int_pid_yaw

                # Lateral motion control (Y-axis)
                e_pid_Y_im1 = Y_ref[i-1] - old_states[5]
                e_pid_Y_i = Y_ref[i] - states[5]
                e_dot_pid_Y = (e_pid_Y_i - e_pid_Y_im1) / Ts
                e_int_pid_Y = e_int_pid_Y + (e_pid_Y_im1 + e_pid_Y_i) / 2 * Ts

                U1_Y = Kp_Y * e_pid_Y_i + Kd_Y * e_dot_pid_Y + Ki_Y * e_int_pid_Y

                # U1 combined control (Yaw + Y-axis)
                U1 = U1_yaw + U1_Y

                ### X position control (generate desired velocity)
                e_pid_X_im1 = X_ref[i-1] - old_states[4]  # Error in position
                e_pid_X_i = X_ref[i] - states[4]          # Current position error
                e_dot_pid_X = (e_pid_X_i - e_pid_X_im1) / Ts
                e_int_pid_X = e_int_pid_X + (e_pid_X_im1 + e_pid_X_i) / 2 * Ts

                v_desired_X = Kp_X * e_pid_X_i + Kd_X * e_dot_pid_X + Ki_X * e_int_pid_X

                ### Velocity control (track the desired velocity)
                e_pid_vx_im1 = v_desired_X - old_states[0]  # Error in desired velocity
                e_pid_vx_i = v_desired_X - states[0]        # Current velocity error
                e_dot_pid_vx = (e_pid_vx_i - e_pid_vx_im1) / Ts
                e_int_pid_accel_x = e_int_pid_accel_x + (e_pid_vx_im1 + e_pid_vx_i) / 2 * Ts

                U2_accel_x = Kp_vx * e_pid_vx_i + Kd_vx * e_dot_pid_vx + Ki_vx * e_int_pid_accel_x

                # ### Lateral velocity control (Y-axis for velocity)
                # e_pid_vy_im1 = y_dot_ref[i-1] - old_states[1]
                # e_pid_vy_i = y_dot_ref[i] - states[1]
                # e_dot_pid_vy = (e_pid_vy_i - e_pid_vy_im1) / Ts
                # e_int_pid_accel_y = e_int_pid_accel_y + (e_pid_vy_im1 + e_pid_vy_i) / 2 * Ts

                # U2_accel_y = Kp_vy * e_pid_vy_i + Kd_vy * e_dot_pid_vy + Ki_vy * e_int_pid_accel_y

                # U2 final control (acceleration only for longitudinal + lateral control)
                U2 = U2_accel_x 


            old_states = states
        else :

            ######################### Update Controller MPC ########################################
            # State put into the control 
            # Create new state by combining states_obs and the last two elements of states


            # Generate the discrete state space matrices
            Ad,Bd,Cd,Dd=support.state_space(new_state_obs,U1,U2)


            # Generate the augmented current state and the reference vector
            x_aug_t=np.transpose([np.concatenate((new_state_obs,[U1,U2]),axis=0)])


            # Generate the compact simplification matrices for the cost function
            Hdb,Fdbt,Cdb,Adc,G,ht=support.mpc_simplification(Ad,Bd,Cd,Dd,hz,x_aug_t,du)
            ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)
            
            # Hdb=np.array([[10.,1.],[1.,10.]])
            # ft=np.array([0.,0.])
            # G=np.array([[-1.,0.],[0.,-1.],[-1.4,-1.]])
            # ht=np.array([-0.05,-0.07,-0.15])
            ################# Constraints #####################
            try:
                du=solve_qp(Hdb,ft,G,ht,solver="cvxopt")
                du=np.transpose([du])
                # print(du)
                # exit()
            except ValueError as ve:
                print(Hdb)
                print(ft)
                print(G)
                print(ht)
                print(Adc)
                print(x_aug_t)
                print(du)
                print(i)
                break;


        #############################################################################



        # Update the real inputs

        U1=U1+du[0][0]
        U2=U2+du[1][0]

        # if compensate:


        UTotal[i+1][0]=U1
        UTotal[i+1][1]=U2


        ######################### Gradient and TRain network ########################################
        # # New ref for calculate the gradient


        #-----------Policy update based on gradient descent------------#

        dx_df  = carGrad.GradientSolver_general(dx_df) # expected shape (4 , D_out ) = nominateur , denominateur
        gradient_dx_df_storage.append(dx_df)


        # TODO : states_obs is a head one step with measurement = C*x , so need to use statesTotal_obs to get the last state 

        statesTotal_obs_k =statesTotal_obs[i]   #  statesTotal_obs[i]   or states_obs
        # nn_input= np.reshape(np.array([states_obs[0], states_obs[1],states_obs[2], states_obs[3] ,U1,U2 ]),(D_in,1))  
        # f_nn = model_QR(nn_input).detach().numpy()

        if (LossType == 'refs'):
            # New ref for calculate the gradient
            input_target = np.array([[x_dot_ref[i]],
                            [y_dot_ref[i]],
                            [psi_ref[i]],
                            [psi_dot_ref[i]]])
        elif (LossType == 'mesurement_small'):
            input_target = measurements
        else: #LossType == 'mesurement_full':
            input_target = support.Cd_Full @ states


        if (Type_learning == 'learningby_Dict'):
            ##--------------------------- Test_Huy

            # Add new feature to dictionary , 
            # Giả sử là case LossType == 'mesurement_full' => taget1 = measurement , target2 = f_hat , save also the time  

            #TODO : Test 
            learning_batch.add_feature(nn_input, f_nn ,statesTotal_obs_k, input_target,f_uk , dx_df ,i)

            # learning_batch.add_feature(nn_input,statesTotal_obs_k, input_target,f_hat,i)
            
            total_loss_track = 0.0
            total_dldf = 0.0
            count = 0            

            # dldf = np.zeros((1, D_out))

            # Loop through each stored feature and calculate losses
            for feature,f_nn_tempo,states_hat, target1, target2_f_uk,dx_df_tempo ,times_data in learning_batch.feature_dict:
                # f_nn_tempo = model_QR(feature).detach().numpy()

                # states_hat_new_tempo , f_hat_new_tempo=  support.observer(states_hat,U1,U2 , measurements, f_hat , f_nn_tempo  )
                
                # states_hat_new_tempo = np.squeeze(np.asarray(states_hat_new_tempo)) # rember that pain, so hurt
                # f_hat_new_tempo = np.squeeze(np.asarray(f_hat_new_tempo))

                # dldf, loss_track = chainRule_func(target1, states_hat_new_tempo, dx_df, f_hat_new_tempo, f_nn_tempo, weight_error_traj, lamda)

                dldf_tempo, loss_track_tempo = chainRule_func(target1, states_hat, dx_df_tempo, target2_f_uk, f_nn_tempo, weight_error_traj, lamda)

                # Loss calculation
                total_loss_track += loss_track_tempo
                total_dldf += dldf_tempo
                count += 1

            #    Calculate the mean if there are losses; otherwise return zero
            loss_track = total_loss_track / count if count > 0 else 0
            dldf = total_dldf / count
            # dldf = total_dldf 


        elif (Type_learning == 'normal_Basic'):
            
            dldf, loss_track = chainRule_func(input_target, statesTotal_obs_k, dx_df, f_uk, f_nn, weight_error_traj, lamda)
            
            # in paper
            # dldf, loss_track = chainRule_func(input_target, statesTotal_obs_k, dx_df, f_hat, f_nn, weight_error_traj, lamda)



            


        else :
            ##--------------------------- Test_Huy

            loss_nn_pytorch = model_queue.train_latest_model(dldf , nn_input, epochs = batch_size, lr=lr_nn  )




        ######################### Update the real system ####################################

        # w_f = 10 * random.random() * np.sin(i * 10)
        # w_r = 10 * random.random() * np.cos(i * 10)

        if have_f_tire :
            w_f , w_r = support.dis_noise(states,U1,U2)
                
            # w_f += 5 * random.random() * np.sin(i * 10)
            # w_r += 5 * random.random() * np.cos(i * 10)
        else : 
            w_f = 0
            w_r = 0


        # ------------ F_uk here is D@[w_f] , [w_r]
        # A_conti, B_conti, C_conti, D_conti = support.matrix_state_space_simple_observer(states , U1 , U2 )
        # f_uk_tempo = (D_conti @ np.array([ [w_f] , [w_r] ]))
        # f_uk_Save[:,i:i+1] = f_uk_tempo # np.squeeze(np.asarray(f_uk_tempo))

        # ------------ New f_uk is state - obs2



        w_uncertain_f_save[i][0] = w_f
        w_uncertain_r_save[i][0] = w_r


        states,x_dot_dot,y_dot_dot,psi_dot_dot=support.open_loop_new_states_RK4(states,U1,U2 , w_r, w_f)
        statesTotal[i+1][0:len(states)] = states


        ######################### Accelerations ####################################
        accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
        accelerations_total[i+1][0:len(accelerations)]=accelerations

        ######################### Check for Convergence ####################################

        # Simple loss_track =  x_hat - x_ref   
        loss_track = np.reshape(loss_track,(1))
        sum_loss += loss_track


        # Update the previous time for the next iteration
        previous_time = current_time


        # This is to monitor the progress of the simulation
        if i%500==0:
            print('Progress: '+str(round(i/sim_length*100,2))+'%')


    


    '''---------------- Save the model after each Epoch-------------------------- '''

    mean_loss = sum_loss/sim_length
    # Average error over the entire epoch
    # avg_error = total_error / sim_length

    Loss += [mean_loss]


    # print(f"Model saved at epoch {epoch}.")

    print('Finish:mean_loss=',mean_loss)

    save_Exel.Add("mean_loss", mean_loss)
    save_Exel.Save()

    # Define your desired folder path
    folder_save_matlab_data = os.path.join(SCRIPT_DIR, "data_evaluation")

    # Ensure the folder exists (creates it if it doesn't exist)
    os.makedirs(folder_save_matlab_data, exist_ok=True)

    # Save the final data after training is complete
    scipy.io.savemat(os.path.join(folder_save_matlab_data,f'evaluation2_training_data.mat'), {
    'statesTotal': statesTotal,
    'statesTotal_obs': statesTotal_obs,
    'statesTotal_obs2': statesTotal_obs2,
    'statesTotal_Kalma_obs': statesTotal_Kalma_obs,
    'f_hatTotal_obs': f_hatTotal_obs,
    'f_hatTotal_obs2': f_hatTotal_obs2,
    'f_nn_Save': f_nn_Save,
    'f_uk_Save': f_uk_Save,
    'f_uk_2_Save': f_uk_2_Save,
    'UTotal': UTotal,
    'x_dot_ref': x_dot_ref,
    'y_dot_ref': y_dot_ref,
    'psi_ref': psi_ref,
    'X_ref': X_ref,
    'Y_ref': Y_ref ,
    'psi_dot_ref': psi_dot_ref,
    'accelerations_total': accelerations_total,
    't':t,
    'dldf_Total': dldf_Total,
    'w_uncertain_f_save':w_uncertain_f_save,
    'w_uncertain_r_save':w_uncertain_r_save,
    })




    print('Finsh !!! ')


    ######################### Check for Convergence After Epoch ###########################
    # if avg_error < error_threshold:
    #     print(f"Converged after {epoch} epochs, error within tolerance.")
    #     break

if (Mode_simulation == 'train'):
    Train(x_dot_ref,y_dot_ref,psi_ref,X_ref,Y_ref , psi_dot_ref  , refSignals , sim_length)    
else:
    Evaluation(x_dot_ref,y_dot_ref,psi_ref,X_ref,Y_ref , psi_dot_ref  , refSignals , sim_length)

# plt.figure(1)
# plt.plot(Epochs, Loss, linewidth=1.5, marker='o')
# plt.xlabel('Training episodes')
# plt.ylabel('Mean loss')
# plt.grid()
# # plt.savefig('plots_in_paper/mean_loss_train_reproduction_dmhe.png')
# plt.show()


    ######################### Save and FIN   ####################################

    # # This is to monitor the progress of the simulation
    # if i%500==0:
    #     print("Progress: "+str(round(i/sim_length*100,2))+"%")

    # # To make the animations 5x faster
    # if i%5==1:
    #     t_ani=np.concatenate([t_ani,[t[i]]])
    #     x_dot_ani=np.concatenate([x_dot_ani,[states[0]]])
    #     psi_ani=np.concatenate([psi_ani,[states[2]]])
    #     X_ani=np.concatenate([X_ani,[states[4]]])
    #     Y_ani=np.concatenate([Y_ani,[states[5]]])
    #     delta_ani=np.concatenate([delta_ani,[U1]])

# ################################ ANIMATION LOOP ###############################
# region: ANIMATION

# frame_amount=len(X_ani)
# lf=constants['lf']
# lr=constants['lr']

# def update_plot(num):
#     hz=constants['hz']
#     car_1.set_data([X_ani[num]-lr*np.cos(psi_ani[num]),X_ani[num]+lf*np.cos(psi_ani[num])],
#         [Y_ani[num]-lr*np.sin(psi_ani[num]),Y_ani[num]+lf*np.sin(psi_ani[num])])

#     car_determined.set_data(X_ani[0:num],Y_ani[0:num])
#     x_dot.set_data(t_ani[0:num],x_dot_ani[0:num])
#     yaw_angle.set_data(t_ani[0:num],psi_ani[0:num])
#     X_position.set_data(t_ani[0:num],X_ani[0:num])
#     Y_position.set_data(t_ani[0:num],Y_ani[0:num])
#     # if num<len(X_ani)-5:
#     #     car_predicted.set_data(X_opt_total[(num+1)+4*num][0:hz],Y_opt_total[(num+1)+4*num][0:hz])
#     # else:
#     #     car_predicted.set_data(X_opt_total[(num+1)+4*num][0:0],Y_opt_total[(num+1)+4*num][0:0])

#     car_1_body.set_data([-lr*np.cos(psi_ani[num]),lf*np.cos(psi_ani[num])],
#         [-lr*np.sin(psi_ani[num]),lf*np.sin(psi_ani[num])])

#     car_1_body_extension.set_data([0,(lf+40)*np.cos(psi_ani[num])],
#         [0,(lf+40)*np.sin(psi_ani[num])])

#     car_1_back_wheel.set_data([-(lr+0.5)*np.cos(psi_ani[num]),-(lr-0.5)*np.cos(psi_ani[num])],
#         [-(lr+0.5)*np.sin(psi_ani[num]),-(lr-0.5)*np.sin(psi_ani[num])])

#     car_1_front_wheel.set_data([lf*np.cos(psi_ani[num])-0.5*np.cos(psi_ani[num]+delta_ani[num]),lf*np.cos(psi_ani[num])+0.5*np.cos(psi_ani[num]+delta_ani[num])],
#         [lf*np.sin(psi_ani[num])-0.5*np.sin(psi_ani[num]+delta_ani[num]),lf*np.sin(psi_ani[num])+0.5*np.sin(psi_ani[num]+delta_ani[num])])

#     car_1_front_wheel_extension.set_data([lf*np.cos(psi_ani[num]),lf*np.cos(psi_ani[num])+(0.5+40)*np.cos(psi_ani[num]+delta_ani[num])],
#         [lf*np.sin(psi_ani[num]),lf*np.sin(psi_ani[num])+(0.5+40)*np.sin(psi_ani[num]+delta_ani[num])])

#     yaw_angle_text.set_text(str(round(psi_ani[num],2))+' rad')
#     steer_angle.set_text(str(round(delta_ani[num],2))+' rad')
#     body_x_velocity.set_text(str(round(x_dot_ani[num],2))+' m/s')

#     return car_determined,car_1,x_dot,yaw_angle,X_position,Y_position,\
#     car_1_body,car_1_body_extension,car_1_back_wheel,car_1_front_wheel,car_1_front_wheel_extension,\
#     yaw_angle_text,steer_angle,body_x_velocity#,car_predicted


# # Set up your figure properties
# fig_x=16
# fig_y=9
# fig=plt.figure(figsize=(fig_x,fig_y),dpi=120,facecolor=(0.8,0.8,0.8))
# n=12
# m=12
# gs=gridspec.GridSpec(n,m)

# # Main trajectory
# plt.subplots_adjust(left=0.05,bottom=0.08,right=0.95,top=0.95,wspace=0.15,hspace=0)

# ax0=fig.add_subplot(gs[:,0:9],facecolor=(0.9,0.9,0.9))
# ax0.grid(True)
# plt.title('Autonomous vehicle animation (5x faster than the reality)',size=15)
# plt.xlabel('X-position [m]',fontsize=15)
# plt.ylabel('Y-position [m]',fontsize=15)

# # Plot the reference trajectory
# ref_trajectory=ax0.plot(X_ref,Y_ref,'b',linewidth=1)

# # Draw a motorcycle
# car_1,=ax0.plot([],[],'k',linewidth=3)
# # car_predicted,=ax0.plot([],[],'-m',linewidth=2)
# car_determined,=ax0.plot([],[],'-r',linewidth=1)

# # Zoomed vehicle
# if trajectory==1:
#     ax1=fig.add_subplot(gs[0:6,0:5],facecolor=(0.9,0.9,0.9))
# elif trajectory==2:
#     ax1=fig.add_subplot(gs[3:9,2:7],facecolor=(0.9,0.9,0.9))
# else:
#     ax1=fig.add_subplot(gs[2:6,2:5],facecolor=(0.9,0.9,0.9))
# ax1.axes.get_xaxis().set_visible(False)
# ax1.axes.get_yaxis().set_visible(False)

# bbox_props_x_dot=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='b',lw=1.0)
# bbox_props_steer_angle=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='r',lw=1.0)
# bbox_props_angle=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='k',lw=1.0)

# neutral_line=ax1.plot([-50,50],[0,0],'k',linewidth=1)
# car_1_body,=ax1.plot([],[],'k',linewidth=3)
# car_1_body_extension,=ax1.plot([],[],'--k',linewidth=1)
# car_1_back_wheel,=ax1.plot([],[],'r',linewidth=4)
# car_1_front_wheel,=ax1.plot([],[],'r',linewidth=4)
# car_1_front_wheel_extension,=ax1.plot([],[],'--r',linewidth=1)


# plt.xlim(-5,5)
# plt.ylim(-4,4)

# body_x_velocity=ax1.text(3,-1.5,'',size='10',color='b',bbox=bbox_props_x_dot)
# steer_angle=ax1.text(3,-2.5,'',size='10',color='r',bbox=bbox_props_steer_angle)
# yaw_angle_text=ax1.text(3,-3.5,'',size='10',color='k',bbox=bbox_props_angle)

# body_x_velocity_word=ax1.text(3.7,3.4,'x_dot',size='10',color='b',bbox=bbox_props_x_dot)
# steer_angle_word=ax1.text(3.8,2.5,'delta',size='10',color='r',bbox=bbox_props_steer_angle)
# yaw_angle_word=ax1.text(4.2,1.6,'Psi',size='10',color='k',bbox=bbox_props_angle)


# # x_dot function
# ax2=fig.add_subplot(gs[0:3,9:12],facecolor=(0.9,0.9,0.9))
# x_dot_reference=ax2.plot(t,x_dot_ref,'-b',linewidth=1)
# x_dot,=ax2.plot([],[],'-r',linewidth=1)
# plt.title('© Mark Misin Engineering')
# ax2.spines['bottom'].set_position(('data',-9999999))
# ax2.yaxis.tick_right()
# ax2.grid(True)
# plt.xlabel('time [s]',fontsize=15)
# plt.ylabel('x_dot [m/s]',fontsize=15)
# ax2.yaxis.set_label_position("right")

# # Psi function
# ax3=fig.add_subplot(gs[3:6,9:12],facecolor=(0.9,0.9,0.9))
# yaw_angle_reference=ax3.plot(t,psi_ref,'-b',linewidth=1)
# yaw_angle,=ax3.plot([],[],'-r',linewidth=1)
# ax3.spines['bottom'].set_position(('data',-9999999))
# ax3.yaxis.tick_right()
# ax3.grid(True)
# plt.xlabel('time [s]',fontsize=15)
# plt.ylabel('Psi [rad]',fontsize=15)
# ax3.yaxis.set_label_position("right")

# # X function
# ax4=fig.add_subplot(gs[6:9,9:12],facecolor=(0.9,0.9,0.9))
# X_position_reference=ax4.plot(t,X_ref,'-b',linewidth=1)
# X_position,=ax4.plot([],[],'-r',linewidth=1)
# ax4.spines['bottom'].set_position(('data',-9999999))
# ax4.yaxis.tick_right()
# ax4.grid(True)
# plt.xlabel('time [s]',fontsize=15)
# plt.ylabel('X-position [m]',fontsize=15)
# ax4.yaxis.set_label_position("right")

# # Y function
# ax5=fig.add_subplot(gs[9:12,9:12],facecolor=(0.9,0.9,0.9))
# Y_position_reference=ax5.plot(t,Y_ref,'-b',linewidth=1)
# Y_position,=ax5.plot([],[],'-r',linewidth=1)
# ax5.yaxis.tick_right()
# ax5.grid(True)
# plt.xlabel('time [s]',fontsize=15)
# plt.ylabel('Y-position [m]',fontsize=15)
# ax5.yaxis.set_label_position("right")


# car_ani=animation.FuncAnimation(fig, update_plot,
#     frames=frame_amount,interval=20,repeat=True,blit=True)
# plt.show()

# # This is to record a video on the animation
# # Matplotlib 3.3.3 needed - close the animation itself to start the recording process
# Writer=animation.writers['ffmpeg']
# writer=Writer(fps=30,metadata={'artist': 'Me'},bitrate=1800)
# car_ani.save('car_mpc_demo_traj3_v2.mp4',writer)

# endregion: ANIMATION

##################### END OF THE ANIMATION ############################


# c_fontsize = 8
# c_linewidth=1


'''---------- Plot f_nn with f_hay observer ---------------------- '''


# plt.subplot(4,1,1)
# plt.plot(t,f_nn_Save[0,:],'r',linewidth=1,label='x_dot')
# plt.plot(t,f_hatTotal_obs[:,0],'m',linewidth=1,label='x_hat_dot')

# plt.xlabel('t-time [s]',fontsize=c_fontsize)
# plt.ylabel('x_dot [m/s]',fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='center right',fontsize='small')

# plt.subplot(4,1,2)
# # plt.plot(t,y_dot_ref,'--b',linewidth=c_linewidth,label='y_dot_ref')
# plt.plot(t,f_nn_Save[1,:],'r',linewidth=1,label='y_dot')
# plt.plot(t,f_hatTotal_obs[:,1],'m',linewidth=1,label='y_hat_dot')


# plt.xlabel('t-time [s]',fontsize=c_fontsize)
# plt.ylabel('y_dot [m/s]',fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='center right',fontsize='small')

# plt.subplot(4,1,3)
# plt.plot(t,f_nn_Save[2,:],'r',linewidth=1,label='psi')
# plt.plot(t,f_hatTotal_obs[:,2],'m',linewidth=1,label='psi_hat')
# plt.grid(True)

# plt.xlabel('t-time [s]',fontsize=c_fontsize)
# plt.ylabel('psi [rad/s]',fontsize=c_fontsize)
# plt.legend(loc='upper right',fontsize='small')

# plt.subplot(4,1,4)
# plt.plot(t,f_nn_Save[3,:],'r',linewidth=1,label='psi_dot')
# plt.plot(t,f_hatTotal_obs[:,3],'m',linewidth=1,label='psi_hat_dot')

# plt.xlabel('t-time [s]',fontsize=c_fontsize)
# plt.ylabel('psi_dot [rad/s]',fontsize=c_fontsize)

# plt.grid(True)
# plt.legend(loc='upper right',fontsize='small')
# plt.show()


'''---------- Plot the world---------------------- '''
# plt.plot(X_ref,Y_ref,'--b',linewidth=c_linewidth,label='The trajectory')
# plt.plot(statesTotal[:,4],statesTotal[:,5],'r',linewidth=1,label='Car position')
# plt.xlabel('X-position [m]',fontsize=c_fontsize)
# plt.ylabel('Y-position [m]',fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='upper right',fontsize='small')
# plt.xlim(0,x_lim)
# plt.ylim(0,y_lim)
# plt.xticks(np.arange(0,x_lim+1,int(x_lim/10)))
# plt.yticks(np.arange(0,y_lim+1,int(y_lim/10)))
# plt.show()



# '''---------- Plot Controller ---------------------- '''
# plt.figure(2)

# # First subplot (steering wheel angle)
# ax1 = plt.subplot(2, 1, 1)
# plt.plot(t, UTotal[:, 0], 'r', linewidth=1, label='steering wheel angle')
# plt.xlabel('t-time [s]', fontsize=c_fontsize)
# plt.ylabel('steering wheel angle [rad]', fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='lower right', fontsize='small')

# # Second subplot (applied acceleration)
# ax2 = plt.subplot(2, 1, 2)
# plt.plot(t, UTotal[:, 1], 'r', linewidth=1, label='applied acceleration')
# plt.xlabel('t-time [s]', fontsize=c_fontsize)
# plt.ylabel('applied acceleration [m/s^2]', fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='lower right', fontsize='small')

# plt.show()

# '''---------- Plot How good tracking ---------------------- '''
# plt.figure(3)

# # First subplot (X_ref position)
# ax3 = plt.subplot(2, 1, 1)
# plt.plot(t, X_ref, '--b', linewidth=c_linewidth, label='X_ref position')
# plt.plot(t, statesTotal[:, 4], 'r', linewidth=1, label='Car X position')
# plt.xlabel('t-time [s]', fontsize=c_fontsize)
# plt.ylabel('X-position [m]', fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='lower right', fontsize='small')

# # Second subplot (Y_ref position)
# ax4 = plt.subplot(2, 1, 2)
# plt.plot(t, Y_ref, '--b', linewidth=c_linewidth, label='Y_ref position')
# plt.plot(t, statesTotal[:, 5], 'r', linewidth=1, label='Car Y position')
# plt.xlabel('t-time [s]', fontsize=c_fontsize)
# plt.ylabel('Y-position [m]', fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='lower right', fontsize='small')

# plt.show()

# '''---------- Plot Refs, State, State hat ---------------------- '''
# plt.figure(4)

# # First subplot (x_dot_ref, x_dot, x_hat_dot)
# ax5 = plt.subplot(4, 1, 1)
# plt.plot(t, x_dot_ref, '--b', linewidth=c_linewidth, label='x_dot_ref')
# plt.plot(t, statesTotal[:, 0], 'r', linewidth=c_linewidth, label='x_dot')
# plt.plot(t, statesTotal_obs[:, 0], 'm', linewidth=c_linewidth, label='x_hat_dot')
# plt.xlabel('t-time [s]', fontsize=c_fontsize)
# plt.ylabel('x_dot [m/s]', fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='center right', fontsize='small')

# # Second subplot (y_dot_ref, y_dot, y_hat_dot)
# ax6 = plt.subplot(4, 1, 2)
# plt.plot(t, y_dot_ref, '--b', linewidth=c_linewidth, label='y_dot_ref')
# plt.plot(t, statesTotal[:, 1], 'r', linewidth=1, label='y_dot')
# plt.plot(t, statesTotal_obs[:, 1], 'm', linewidth=1, label='y_hat_dot')
# plt.xlabel('t-time [s]', fontsize=c_fontsize)
# plt.ylabel('y_dot [m/s]', fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='center right', fontsize='small')

# # Third subplot (psi_ref, Car yaw angle)
# ax7 = plt.subplot(4, 1, 3)
# plt.plot(t, psi_ref, '--b', linewidth=c_linewidth, label='Yaw_ref angle')
# plt.plot(t, statesTotal[:, 2], 'r', linewidth=1, label='Car yaw angle')
# plt.plot(t, statesTotal_obs[:, 2], 'm', linewidth=1, label='Car yaw angle')
# plt.xlabel('t-time [s]', fontsize=c_fontsize)
# plt.ylabel('psi [rad]', fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='lower right', fontsize='small')

# # Fourth subplot (psi_dot_ref, psi_dot, psi_hat_dot)
# ax8 = plt.subplot(4, 1, 4)
# plt.plot(t, psi_dot_ref, '--b', linewidth=c_linewidth, label='Yaw_ref angle')
# plt.plot(t, statesTotal[:, 3], 'r', linewidth=1, label='psi_dot')
# plt.plot(t, statesTotal_obs[:, 3], 'm', linewidth=1, label='psi_hat_dot')
# plt.xlabel('t-time [s]', fontsize=c_fontsize)
# plt.ylabel('psi_dot [rad/s]', fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='upper right', fontsize='small')

# plt.show()


########################### Accelerations ######################################
# plt.subplot(3,1,1)
# plt.plot(t,accelerations_total[:,0],'b',linewidth=1,label='x_dot_dot')
# plt.xlabel('t-time [s]',fontsize=c_fontsize)
# plt.ylabel('x_dot_dot [m/s^2]',fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='upper right',fontsize='small')

# plt.subplot(3,1,2)
# plt.plot(t,accelerations_total[:,1],'b',linewidth=1,label='y_dot_dot')
# plt.xlabel('t-time [s]',fontsize=c_fontsize)
# plt.ylabel('y_dot_dot [m/s^2]',fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='upper right',fontsize='small')

# plt.subplot(3,1,3)
# plt.plot(t,accelerations_total[:,2],'b',linewidth=1,label='psi_dot_dot')
# plt.xlabel('t-time [s]',fontsize=c_fontsize)
# plt.ylabel('psi_dot_dot [rad/s^2]',fontsize=c_fontsize)
# plt.grid(True)
# plt.legend(loc='upper right',fontsize='small')
# plt.show()

# exit()











##########################
