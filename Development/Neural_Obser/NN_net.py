"""
This file defines the class of neural network that parameterizes Q and R
------------------------------------------------------------------------
Wang, Bingheng, 02, Jan., 2021, at UTown, NUS
modified on 08, Jan., 2021, at Control & Simulation Lab, NUS
"""

from collections import deque
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

import copy
from torch.nn.utils import spectral_norm

class Net(nn.Module):
    def __init__(self, D_in, D_h, D_out):
        # D_in : dimension of input layer
        # D_h  : dimension of hidden layer
        # D_out: dimension of output layer
        super(Net, self).__init__()
        self.linear1 = spectral_norm(nn.Linear(D_in, D_h))
        self.linear2 = spectral_norm(nn.Linear(D_h, D_h))
        self.linear3 = spectral_norm(nn.Linear(D_h, D_out))
        # self.linear1 = nn.Linear(D_in, D_h)
        # self.linear2 = nn.Linear(D_h, D_h)
        # self.linear3 = nn.Linear(D_h, D_out) # linear9
        # self.dropout = nn.Dropout(1e-4)

        # Initialize weights using Xavier initialization
        self._initialize_weights()

    def _initialize_weights(self):
        for layer in [self.linear1, self.linear2, self.linear3]:
            # Apply Xavier initialization
            nn.init.xavier_uniform_(layer.weight)  # For weights
            if layer.bias is not None:
                nn.init.zeros_(layer.bias)  # For biases

    def forward(self, input):
        # # convert state s to tensor
        # S = torch.tensor(input, dtype=torch.float) # column 2D tensor
        # z1 = self.linear1(S.t()) # linear function requires the input to be a row tensor
        # z2 = F.relu(z1)  # hidden layer 1
        # # z2 = self.dropout(z2)
        # z3 = self.linear2(z2)
        # z4 = F.relu(z3)  # hidden layer 2
        # # z4 = self.dropout(z4)
        # z5= self.linear3(z4) # output layer


        # convert state s to tensor
        S = torch.tensor(input, dtype=torch.float) # column 2D tensor
        z1 = self.linear1(S.t()) # linear function requires the input to be a row tensor
        z2 = F.silu(z1)  # hidden layer 1
        # z2 = self.dropout(z2)
        z3 = self.linear2(z2)
        z4 = F.silu(z3)  # hidden layer 2
        # z4 = self.dropout(z4)
        z5= self.linear3(z4) # output layer
        return z5.t()

    def myloss(self, para, dp):
        # convert np.array to tensor
        Dp = torch.tensor(dp, dtype=torch.float) # row 2D tensor
        loss_nn = torch.matmul(Dp, para)

        # Compute L1 regularization term (sum of absolute values of weights)
        l1_norm = sum(p.abs().sum() for p in self.parameters())

        #  `reg_factor` could be 1e-4 or some other small constant
        reg_factor = 0.0
        loss_nn = loss_nn + reg_factor * l1_norm
        return loss_nn


class Learning_batch:
    def __init__(self,   feature_capacity=10):

        # Initialize feature dictionary with fixed size
        self.feature_dict = deque(maxlen=feature_capacity)
    
    # Adding new features
    # Add new feature to dictionary , 
    # Giả sử là case LossType == 'mesurement_full' => taget1 = measurement , target2 = f_hat , save also the time  
    def add_feature(self, input,f_nn ,statesTotal_obs_k,target1 , target2,dx_df, time):
        self.feature_dict.append((input,f_nn,statesTotal_obs_k ,target1 , target2,dx_df, time))  # Stores features and their targets




class ModelQueue:
    def __init__(self,  D_in , D_out , queue_size=3, decay_factor=0.5):
        # Initialize a queue to store up to `queue_size` models
        self.queue_size = queue_size
        self.model_queue = deque(maxlen=queue_size)  # queue of fixed size
        self.decay_factor = decay_factor  # set exponential decay factor, can be modified
        self.input_size = D_in

        self.output_size = D_out
    
    def add_model(self, model):
        """Adds a new model to the queue. Removes the oldest one if full."""
        # if len(self.model_queue) == self.queue_size:
        #     print(f"Removing oldest model from queue.")

        self.model_queue.append(model)
        # print(f"Added new model to queue. Queue length: {len(self.model_queue)}")

    def predict(self, input_data):
        """Calculates the weighted sum of the outputs of the models in the queue using exponential decay."""
        if not self.model_queue:
            raise ValueError("Model queue is empty.")
        
        # Ensure input data is a PyTorch tensor
        if not isinstance(input_data, torch.Tensor):
            input_data = torch.tensor(input_data, dtype=torch.float32)


        # Calculate decay weights based on model index
        #------- decay_weights like in the paper
        decay_weights = []
        for i in range(len(self.model_queue)):
            exp_weight = torch.exp(torch.tensor((i - len(self.model_queue) + 1) / 3.0))
            decay_weights.append(exp_weight)

        decay_weights = torch.tensor(decay_weights)  # Convert list to tensor
        decay_weights /= decay_weights.sum()  # Normalize the weights

        #------- Simpler decay_weights 
        # decay_weights = np.exp(-np.arange(len(self.model_queue)) / self.decay_factor)
        # decay_weights /= decay_weights.sum()  # Normalize the weights
        
        ouput_mean_sum = torch.zeros(self.output_size,1)  # Initialize the weighted sum

        # Calculate the weighted sum of model outputs
        for i, model in enumerate(reversed(self.model_queue)):
            model_output = model(input_data)
            ouput_mean_sum += decay_weights[i] * model_output
        
        return ouput_mean_sum

    def duplicate_and_train_latest_model(self, input_data, target_data, epochs=10, lr=0.001):
        """Duplicate the latest model, add it to the queue, and train the new model."""
        if not self.model_queue:
            raise ValueError("Model queue is empty.")
        
        # Duplicate the latest model
        latest_model = self.model_queue[-1]
        new_model = copy.deepcopy(latest_model)
        self.add_model(new_model)
        
        # Train the new model
        self.train_latest_model(input_data, target_data, epochs=epochs, lr=lr)

    def add_queue(self, duplicate, D_in, D_h, D_out):
        """Adds a model to the queue. duplicate a old model  or make a new model."""
        if not self.model_queue:
            # raise ValueError("Model queue is empty.")
            self.add_model(Net(D_in, D_h, D_out))
            return
        
        if (duplicate == True):
            # Duplicate the latest model
            latest_model = self.model_queue[-1]
            new_model = copy.deepcopy(latest_model)
            self.add_model(new_model)
        else:
            self.add_model(Net(D_in, D_h, D_out))


    def train_latest_model(self, dldf , input_data, epochs=10, lr=0.001):
        """Train the latest model in the queue."""
        if not self.model_queue:
            raise ValueError("Model queue is empty.")
        
        # Get the latest model
        latest_model = self.model_queue[-1]
        
        # Ensure input and target data are PyTorch tensors
        if not isinstance(input_data, torch.Tensor):
            input_data = torch.tensor(input_data, dtype=torch.float32)

        
        # Define loss function and optimizer

        optimizer = optim.Adam(latest_model.parameters(), lr=lr)


        optimizer.zero_grad()  # Zero the gradients
        # ToDO : Suy nghĩ về ouput của newest model , hay ouput của mean of full 3 models 
        # thay vì latest_model(input_data)  --  mà là  predict(input_data)
        loss_nn_pytorch   = latest_model.myloss(latest_model(input_data), dldf)

        # loss_nn_pytorch   = latest_model.myloss(self.predict(input_data), dldf)

        loss_nn_pytorch.backward()  # Backward pass (compute gradients)
        optimizer.step()  # Update model weights

        return loss_nn_pytorch


