
import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

import numpy as np
import matplotlib.pyplot as plt
from gradient import Gradient_solver
from NN_net import Net
import torch


import scipy.io


import support_files_car_general as sfc_g
import support_files_car_simple as sfc_simple
from MPC_control import MPC 


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



# Create an object for the support functions.
support=sfc_g.SupportFilesCar()
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

#------------------------- References
x_dot_ref, y_dot_ref, psi_ref, X_ref, Y_ref , psi_dot_ref = support.trajectory_generator(t)

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



######################### Gradient and Neural net parameter ########################################



''' Parameter to tune '''

new_model = True
first_model = True

D_in = 6  # input neural net
D_h = 20  # number hidden layer
D_out = 4 # output  layer

# Learning rate
lr_nn = 2
weight_decay_nn = 0     # 1e-4


# Define batch size
# Initialize variables for batch training
batch_size = 5  # Number of samples before each training step
batch_loss = 0.0  # To accumulate the loss
batch_count = 0  # To track how many samples have been processed


max_epochs = 1  # Set an upper limit on the number of epochs

window_size = 0 # smothing the output of the neural

''' End Parameter to tune '''





carGrad = Gradient_solver(support)  # Gradient



# Path where the model and optimizer are saved
model_save_path = "trained_data/trained_model.pt"
optimizer_save_path = "trained_data/optimizer_state.pt"



if new_model :
    model_QR = Net(D_in, D_h, D_out)  # Define your neural network model
    optimizer_p = torch.optim.Adam(model_QR.parameters(), lr=lr_nn , weight_decay=weight_decay_nn)

elif (not new_model and first_model ) :# not new ,but just the first
    PATH0 = "trained_data/initial_nn_model_2.pt" 
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
        PATH0 = "trained_data/initial_nn_model_2.pt" 
        model_QR = torch.load(PATH0)
        # # model_QR = Net(D_in, D_h, D_out)
        # # torch.save(model_QR,PATH0)


'''----------------- OLD  '''
# if not os.path.exists("trained_data"):
#     os.makedirs("trained_data")

# # # (since the neural network parameters are randomly initialized, use the saved initial model to replicate the same training results in the paper)
# PATH0 = "trained_data/initial_nn_model_2.pt" 
# model_QR = torch.load(PATH0)
# # model_QR = Net(D_in, D_h, D_out)
# # torch.save(model_QR,PATH0)

'''----------------- OLD  '''







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

    states=np.array([x_dot, y_dot, psi, psi_dot,X,Y])
    statesTotal=np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
    statesTotal[0][0:len(states)]=states

    ######################### State Observer Inite ########################################
    states_obs=np.array([x_dot, y_dot, psi, psi_dot])

    statesTotal_obs=np.zeros((len(t),len(states_obs))) # It will keep track of all your states during the entire manoeuvre
    statesTotal_obs[0][0:len(states_obs)]=states_obs

    f_hat = np.array([0,0,0,0])

    f_hatTotal_obs=np.zeros((len(t),len(f_hat))) 
    f_hatTotal_obs[0][0:len(f_hat)]=f_hat

    # ------------------ Observer 2 for compare
    states_obs2=np.array([x_dot, y_dot, psi, psi_dot])

    statesTotal_obs2=np.zeros((len(t),len(states_obs2))) # It will keep track of all your states during the entire manoeuvre
    statesTotal_obs2[0][0:len(states_obs2)]=states_obs2

    f_hat2 = np.array([0,0,0,0])

    f_hatTotal_obs2=np.zeros((len(t),len(f_hat2))) 
    f_hatTotal_obs2[0][0:len(f_hat2)]=f_hat2


    ######################### Inite Accelerations ########################################
    x_dot_dot = 0.
    y_dot_dot = 0.
    psi_dot_dot = 0.

    accelerations = np.array([x_dot_dot, y_dot_dot, psi_dot_dot])
    accelerations_total = np.zeros((len(t),len(accelerations)))

    ######################### Inite Control input ########################################


    controller = MPC(support)

    # Load the initial input
    U1=0 # Input at t = -0.02 s (steering wheel angle in rad (delta))
    U2=0 # Input at t = -0.02 s (acceleration in m/s^2 (a))
    UTotal=np.zeros((len(t),2)) # To keep track all your inputs over time
    UTotal[0][0]=U1
    UTotal[0][1]=U2

    # Initiate the controller - simulation loops
    hz = constants['hz'] # horizon prediction period

    
    k=0

    # inital the small change in controller
    # du_acc = 0  
    # du_steer = 0
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


    # first - ouput value of NN 

    f_nn = np.zeros((D_out, 1))
    f_nn_Save = np.zeros((D_out, len(t)))

    # gradient of x  wrt  f  == sensitivity variable , Shape = ( dim X , dim f  ) = (4 , 4)
    dx_df = np.zeros((len(states_obs) , D_out))



    ######################### Save for gradient  ########################################

    # dldf = np.array([0,0,0,0])  # Shape ( 1  ,  D_out)

    dldf_Total=np.zeros((len(t),D_out)) 
    # dldf_hatTotal_obs[0][0:len(dldf)]=dldf

    loss_nn_Total = []

    #endregion Inittial_step


    ######################### Uncertainty  ########################################

    w_uncertain_f_save=np.zeros((len(t),1)) 

    w_uncertain_r_save=np.zeros((len(t),1)) 

    w_uncertain_f_save[0][0] = 0
    w_uncertain_r_save[0][0] = 0


    '''---------------- loop of system-------------------------- '''
    for i in range(0,sim_length-1):


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

        measurement_noise =0  # Simulated noise for measurements (psi, Y)

        measurements = support.Cd_small @ states.T + measurement_noise
        # measurements = measurements.reshape(-1, 1)


        ######################### Neural input########################################
        # ------- Neural input
        
        nn_input = np.reshape(np.array([states_obs[0] - x_dot_ref[i], states_obs[1] - y_dot_ref[i], states_obs[2] - psi_ref[i], states_obs[3] - psi_dot_ref[i] ,U1,U2 ]),(D_in,1))

        # nn_input= np.reshape(np.array([states_obs[0], states_obs[1], states_obs[2], states_obs[3] ,U1,U2 ]),(D_in,1))



        # ------- Put the input into the model , and get the output , and chose Smothing the output or not
        if window_size > 1 :
            if i > 10:
                f_nn = model_QR(nn_input).detach().numpy()
                f_nn = support.moving_average_matrix(f_nn ,  f_nn_Save[:,i-window_size:i] )

            else:
                f_nn = np.zeros((D_out, 1))
        else:
            if i > 10:
                f_nn = model_QR(nn_input).detach().numpy()
                f_nn = support.moving_average_matrix(f_nn ,  f_nn_Save[:,i-window_size:i] )

            else:
                f_nn = np.zeros((D_out, 1))

        f_nn_Save[:,i:i+1] = f_nn


        ######################### Update Observer ########################################

        states_obs_new , f_hat_new=  support.observer(states_obs,U1,U2 , measurements, f_hat , f_nn  )

        states_obs2_new , f_hat2_new=  support.observer2(states_obs2,U1,U2 , measurements, f_hat2 ,  0 )

        
        # -------- Save observer 1
        # states_obs_new have shape (1,4) so not fit to shape (4,)
        # need to reshape that
        states_obs = np.squeeze(np.asarray(states_obs_new)) # rember that pain, so hurt
        f_hat = np.squeeze(np.asarray(f_hat_new))


        statesTotal_obs[i+1][:]  = states_obs
        f_hatTotal_obs[i+1][:] = f_hat

        # -------- Save observer 2 
        states_obs2 = np.squeeze(np.asarray(states_obs2_new)) # rember that pain, so hurt
        f_hat2 = np.squeeze(np.asarray(f_hat2_new))


        statesTotal_obs2[i+1][:]  = states_obs2
        f_hatTotal_obs2[i+1][:] = f_hat2




        ######################################## Update Controller MPC ########################################
        # State put into the control 
        # Create new state by combining states_obs and the last two elements of states

        if i > 50:
            new_state_obs = np.concatenate((states_obs2, states[-2:]))
        else:
            new_state_obs = states




        # Generate the discrete state space matrices (this matrix is LPV , change over time )
        Ad,Bd,Cd,Dd=support.state_space(states,U1,U2)

        # Generate the augmented current state and the reference vector

        x_aug_t=np.transpose([np.concatenate((states,[U1,U2]),axis=0)])
        

        #############################################################################

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
        U1= U1 + du[0][0]
        U2= U2 + du[1][0]

        UTotal[i+1][0]=U1
        UTotal[i+1][1]=U2

        ######################### Gradient and TRain network ########################################
        # New ref for calculate the gradient
        ref_new = np.array([[x_dot_ref[i]],
                            [y_dot_ref[i]],
                            [psi_ref[i]],
                            [psi_dot_ref[i]]])


        #-----------Policy update based on gradient descent------------#

        dx_df  = carGrad.GradientSolver_general(dx_df) # expected shape (4 , D_out ) = nominateur , denominateur
        f_nn = model_QR(nn_input).detach().numpy()

        dldf ,loss_track = carGrad.ChainRule(ref_new, states_obs,dx_df,f_hat,f_nn)


        loss_nn_pytorch   = model_QR.myloss(model_QR(nn_input), dldf)

        # Save for plot
        dldf_Total[i+1][:] = dldf 
        loss_nn_Total += [loss_nn_pytorch.detach().numpy()] 

        # Accumulate the loss for batch training
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




        ######################### Update the real system ####################################

        # w_f = 100 * random.random() * np.sin(i * 10)
        # w_r = 210 * random.random() * np.cos(i * 10)

        # w_uncertain_f_save[i][0] = w_f
        # w_uncertain_r_save[i][0] = w_r


        w_f = 0
        w_r = 0



        states,x_dot_dot,y_dot_dot,psi_dot_dot=support.open_loop_new_states_uncertain(states,U1, U2, w_r, w_f)


        statesTotal[i+1][:] = states

        ######################### Accelerations ####################################
        accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
        accelerations_total[i+1][:]=accelerations

        ######################### Check for Convergence ####################################

        # Simple loss_track =  x_hat - x_ref   
        loss_track = np.reshape(loss_track,(1))
        sum_loss += loss_track

        # Calculate error between states_obs (x_hat) and ref_new (x_ref) , in norm 2 
        error = np.linalg.norm(states_obs[:] - ref_new.flatten(), ord=2)
        total_error += error

    


    '''---------------- Save the model after each Epoch-------------------------- '''

    mean_loss = sum_loss/sim_length
    # Average error over the entire epoch
    avg_error = total_error / sim_length

    Loss += [mean_loss]
    Epochs += [epoch] 


    # Save the model and optimizer after each epoch
    torch.save(model_QR.state_dict(), model_save_path)
    torch.save(optimizer_p.state_dict(), optimizer_save_path)
    # print(f"Model saved at epoch {epoch}.")

    print('learning_iteration:',epoch,'mean_loss=',mean_loss, 'avg_error=', avg_error)


    # Save the final data after training is complete

    # Define your desired folder path
    folder_save_matlab_data = "data_test1"

    # Ensure the folder exists (creates it if it doesn't exist)
    os.makedirs(folder_save_matlab_data, exist_ok=True)

    scipy.io.savemat(os.path.join(folder_save_matlab_data, f'test_training_data_{epoch}.mat'), {
    'statesTotal': statesTotal,
    'statesTotal_obs': statesTotal_obs,
    'statesTotal_obs2': statesTotal_obs2,
    'f_hatTotal_obs': f_hatTotal_obs,
    'f_hatTotal_obs2': f_hatTotal_obs2,
    'f_nn_Save': f_nn_Save,
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
    'loss_nn_Total' : loss_nn_Total
    })


    ######################### Check for Convergence After Epoch ###########################
    if avg_error < error_threshold:
        print(f"Converged after {epoch} epochs, error within tolerance.")
        break



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


c_fontsize = 8
c_linewidth=1


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

exit()











##########################
