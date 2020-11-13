import numpy as np
import imp
import torch
import model

def sigmoid(x):
    return 1 / (1 +np.exp(-x))

class Dynamics(object):
    def __init__(self, stateDim, inputDim, dt):
        self.dt         = dt
        self.steerRatio    = -20.0 * np.pi / 180
        self.throttleRatio = 1
        self.length     = 0.15
        self.lf         = 0.15/2.0
        self.lr         = self.length - self.lf
        self.width      = 0.08
        self.m       	= 0.041
        self.Iz     	= 27.8E-6
        self.Cm1        = 0.287
        self.Cm2        = 0.0545
        self.Cr0        = 0.0518
        self.Cr2        = 0.00035
        self.Br     	= 3.385
        self.Dr     	= 0.173
        self.Bf	        = 2.579
        self.Cf     	= 1.2
        self.Df     	= 0.192
        self.A          = np.array([[0, 0, -0.1084,   0.995, -0.09983,      0],\
                                    [0, 0,  0.5921, 0.09983,    0.995,      0],\
                                    [0, 0,       0,       0,        0,      1],\
                                    [0, 0,       0,  -2.437,    6.757, 0.5998],\
                                    [0, 0,       0,   4.357,    -5657, -1.018],\
                                    [0, 0,       0,    1373,    -1041,   -966]])
        self.B          = np.array([[     0,     0],\
                                    [     0,     0],\
                                    [     0,     0],\
                                    [-3.935, 9.052],\
                                    [ 17.07,     0],\
                                    [  3935,     0]])
        self.last_states = np.zeros([stateDim, 1])
        self.last_inputs = np.zeros([inputDim, 1])


        ###### Select Model Type #######
        ###### 0: Straight linear
        ###### 1: simpleBicycleModel
        ###### 2: mixed Dynamic-Kinematic Bicycle Model
        ###### 3: Linearized Dynamic Bicycle Model
        ###### 4: NPZ neural net model
        ###### 5: pytorch neural net model

        self.modelType = 5

        if self.modelType == 4:
            ## NeuralNet Variables & Params
            # self.modelPath  = '/home/sw/catkin_ws/src/autorally/autorally_control/src/path_integral/params/models/autorally_nnet_09_12_2018.npz'
            # self.modelPath  = '/home/sw/catkin_ws/src/autorally/autorally_control/src/path_integral/params/models/optitrack_nnet_20200823-042628.npz'
            # self.modelPath  = '/home/sw/rosbag/dnn_model/2020-08-16/bicycle/optitrack_nn_model_20200823-164706.npz'
            # self.modelPath  = '/home/sw/rosbag/NNModel/dnn_model/F1/2020-09-02/f1_nn_model_20200903-092356.npz'
            # self.modelPath  = '/home/sw/rosbag/NNModel/dnn_model/F1/2020-09-02/best_model_20200908-154110.npz'
            self.modelPath  = '/home/sw/Downloads/veh_dynamics_learning/0909.npz'
            
            self.nnet = np.load(self.modelPath)
            self.act_fcn = np.tanh
            self.out_fcn = np.tanh
            # self.out_fcn = sigmoid

        if self.modelType == 5:
            # pytorch model
            # self.importPtModel = imp.load_source("model.py","/home/sw/catkin_ws/src/eurecarr/eurecarr_simulation/script/module/")
            self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
            # device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
            input_size = 6 # roll, vx, vy, yaw_rate, steer, throttle
            output_size = 4 # roll_der, vx_der, vy_der, yaw_der2
            self.ptmodel = model.NeuralNet(input_size, output_size)
            # self.ptmodel = self.importPtModel.NeuralNet(input_size, output_size)
            # self.ptmodel_path = "/home/usrg/catkin_ws/src/eurecarr_field/eurecarr_simulation/src/for_model_simulation/austria_only/checkpoint_9000-8.842071110848337e-05.pt"
            self.ptmodel_path = "/home/usrg/catkin_ws/src/eurecarr_field/eurecarr_simulation/src/for_model_simulation/all_track_wo_scaling/checkpoint_1000-0.0002718334726523608.pt"
            # self.ptmodel_path = "/home/sw/Downloads/veh_dynamics_learning/saved_model/checkpoint_gpu_wo_scaler.pt"
            loaded_state = torch.load(self.ptmodel_path,map_location='cuda:0')
            self.ptmodel.load_state_dict(loaded_state)
            self.ptmodel.to(self.device).float()
            self.ptmodel.eval()



    def forward(self, states, inputs):
        # 0 1 2   3    4  5  6
        # x y yaw roll vx vy yawr
        


        if self.modelType == 0:
            states_der = self.straightLineModel(states, inputs)
        elif self.modelType == 1:
            ## Using Simeple Bicycle Model
            states_der = self.simpleBicycleModel(states, inputs)
        elif self.modelType == 2:
            ## Using mixed Dynamic-Kinematic Bicycle Model
            states_der_d = self.dynamicBicycleModel(states, inputs)
            states_der_k = self.kinematicBicycleModel(states, inputs)
            blend_vx_min = 100
            blend_vx_max = 1000
            vx = states[4]
            blend = min(max((vx-blend_vx_min)/(blend_vx_max-blend_vx_min),0.0),1.0)
            states_der = blend*states_der_d + (1-blend)*states_der_k
        elif self.modelType == 3:
            ## Using Linearized Dynamic Bicycle Model
            states_der = self.linearBicycleModel(states, inputs)
        elif self.modelType == 4:
            states_der = self.neuralNetModel(states, inputs)
        elif self.modelType == 5:
            states_der = self.InferNN(states, inputs)
            

        return states_der

    def local2global(self, x, y, yaw):
        x_g = np.cos(yaw)*x - np.sin(yaw)*y
        y_g = np.sin(yaw)*x + np.cos(yaw)*y
        return x_g, y_g

    def global2local(self, x, y, yaw):
        x_l = np.cos(-yaw)*x - np.sin(-yaw)*y
        y_l = np.sin(-yaw)*x + np.cos(-yaw)*y
        return x_l, y_l

    def straightLineModel(self, states, inputs):
        x_dot, y_dot = self.local2global(states[4], states[5], states[2])
        # x_dot        = states[4]
        # y_dot        = states[5]
        yaw_dot      = 0.0
        roll_dot     = 0.0
        vx_dot       = self.motorModel(states, inputs)#inputs[1]
        vy_dot       = 0.0
        yaw_dotdot   = 0.0
        yaw_dot      = 0.0
        states_der   = np.array([x_dot, y_dot, yaw_dot, roll_dot, vx_dot, vy_dot, yaw_dotdot])
        self.last_states = states
        self.last_inputs = inputs
        return states_der

    def simpleBicycleModel(self, states, inputs):
        x_dot, y_dot = self.local2global(states[4], states[5], states[2])
        yaw_dot      = states[4]*np.sin(inputs[0]*self.steerRatio)/self.length
        roll_dot     = 0.0
        vx_dot       = self.motorModel(states, inputs)#inputs[1]
        vy_dot       = 0
        yaw_dotdot   = (vx_dot * np.sin(inputs[0]*self.steerRatio)\
                         + states[4] * np.cos(inputs[0]*self.steerRatio) * self.steerRatio * (inputs[0]-self.last_inputs[0])/self.dt ) / self.length
        # yaw_dotdot   = (vx_dot*np.sin(inputs[0]*self.steerRatio)+states[4]*np.cos(inputs[0]*self.steerRatio)*(states[4]*np.sin(inputs[0]*self.steerRatio)/self.length)) / self.length
        # yaw_dot      = states[4]*np.sin(inputs[0]*self.steerRatio)/self.length
        yaw_dot      = states[6] + yaw_dotdot * self.dt
        states_der   = np.array([x_dot, y_dot, yaw_dot, roll_dot, vx_dot, vy_dot, yaw_dotdot], dtype=float)
        self.last_states = states
        self.last_inputs = inputs
        return states_der

    def dynamicBicycleModel(self, states, inputs):
        steer    = inputs[0]*self.steerRatio
        throttle = inputs[1]
        x        = states[0]
        y        = states[1]
        yaw      = states[2]
        roll     = states[3]
        vx       = states[4]
        vy       = states[5]
        yaw_dot  = states[6]
        
        alpha_f = np.arctan2(yaw_dot*self.lf + vy, vx) + steer
        alpha_f = np.clip(alpha_f,-0.6,0.6)
        alpha_r = np.arctan2(yaw_dot*self.lr - vy, vx)
        alpha_r = np.clip(alpha_r,-0.6,0.6)
        F_fy, F_ry = self.pacejkaTireModel(alpha_f, alpha_r)
        F_rx       = self.motorModel(states, inputs)

        x_dot, y_dot = self.local2global(vx, vy, yaw)
        roll_dot     = 0.0
        vx_dot       = 1.0/self.m * (F_rx - F_fy*np.sin(steer) + self.m*vy*yaw_dot)
        vy_dot       = 1.0/self.m * (F_ry + F_fy*np.cos(steer) - self.m*vx*yaw_dot)
        yaw_dotdot   = 1.0/self.Iz * (F_fy*self.lf*np.cos(steer) - F_ry*self.lr)

        states_der   = np.array([x_dot, y_dot, yaw_dot, roll_dot, vx_dot, vy_dot, yaw_dotdot])
        return states_der

    def kinematicBicycleModel(self, states, inputs):
        steer    = inputs[0]*self.steerRatio
        throttle = inputs[1]
        x        = states[0]
        y        = states[1]
        yaw      = states[2]
        roll     = states[3]
        vx       = states[4]
        vy       = states[5]
        yaw_dot  = states[6]       

        F_rx  = self.motorModel(states, inputs)

        x_dot, y_dot = self.local2global(vx, vy, yaw)
        roll_dot     = 0.0
        vx_dot       = F_rx / self.m
        vy_dot       = (steer*vx_dot)*self.lr/(self.lr+self.lf)
        yaw_dotdot   = (steer*vx_dot)/(self.lr+self.lf)

        states_der   = np.array([x_dot, y_dot, yaw_dot, roll_dot, vx_dot, vy_dot, yaw_dotdot])
        return states_der


    def pacejkaTireModel(self, alpha_f, alpha_r):
        F_fy = self.Df * np.sin(self.Cf * np.arctan(self.Bf*alpha_f))
        F_ry = self.Dr * np.sin(self.Cr0 * np.arctan(self.Br*alpha_r))

        return F_fy, F_ry

    def motorModel(self, states, inputs):
        vx       = states[4]
        throttle = inputs[1]

        F_rx = (self.Cm1 - self.Cm2*vx) * throttle - self.Cr0*np.sign(vx) - self.Cr2*vx**2

        return F_rx

    def linearBicycleModel(self, states, inputs):
        steer    = inputs[0]*self.steerRatio
        throttle = inputs[1]#self.motorModel(states, inputs)#inputs[1]
        x        = states[0]
        y        = states[1]
        yaw      = states[2]
        roll     = states[3]
        vx       = states[4]
        vy       = states[5]
        yaw_dot  = states[6]

        vx_global, vy_global = self.local2global(vx, vy, yaw)
        states_global     = np.array([x, y, yaw, vx_global, vy_global, yaw_dot])
        inputs_global     = np.array([steer, throttle])
        states_global_der = self.A.dot(states_global) + self.B.dot(inputs_global)
        
        x_dot          = states_global_der[0]
        y_dot          = states_global_der[1]
        yaw_dot        = states_global_der[2]
        roll_dot       = 0.0
        vx_dot, vy_dot = self.global2local(states_global_der[3], states_global_der[4], yaw)
        yaw_dotdot     = states_global_der[5]

        states_der   = np.array([x_dot, y_dot, yaw_dot, roll_dot, vx_dot, vy_dot, yaw_dotdot])
        return states_der



    def neuralNetModel(self, states, inputs):
        # 0 1 2   3    4  5  6
        # x y yaw roll vx vy yawr
        # states = np.zeros_like(states)
        yaw      = states[2]
        roll     = states[3]
        vx       = states[4]
        vy       = states[5]
        yaw_dot  = states[6]
        steer    = inputs[0]
        # throttle = inputs[1]
        throttle = 0.0
        nn_input = [roll, vx, vy, yaw_dot, steer, throttle]
        temp1    = np.zeros(len(self.nnet['dynamics_b1']))
        temp2    = np.zeros(len(self.nnet['dynamics_b2']))
        temp3    = np.zeros(len(self.nnet['dynamics_b3']))

        for i in range(0, len(self.nnet['dynamics_b1'])): # for each node for first layer
            for j in range(0, len(nn_input)): # for each inputs, node value
                temp1[i] += self.nnet['dynamics_W1'][i][j]*nn_input[j]
            temp1[i] += self.nnet['dynamics_b1'][i]
            temp1[i] = self.act_fcn(temp1[i])

        for i in range(0, len(self.nnet['dynamics_b2'])): # for each node for second layer
            for j in range(0, len(temp1)): # for each inputs, node value
                temp2[i] += self.nnet['dynamics_W2'][i][j]*temp1[j]
            temp2[i] += self.nnet['dynamics_b2'][i]
            temp2[i] = self.act_fcn(temp2[i])
        
        for i in range(0, len(self.nnet['dynamics_b3'])): # for each node for third layer
            for j in range(0, len(temp2)): # for each inputs, node value
                temp3[i] += self.nnet['dynamics_W3'][i][j]*temp2[j]
            temp3[i] += self.nnet['dynamics_b3'][i]
            # temp3[i] = self.out_fcn(temp3[i])



        x_dot, y_dot = self.local2global(vx, vy, yaw)
        states_der = np.append([x_dot, y_dot, yaw_dot], temp3)
        # states_der = np.append([vx, vy, yaw_dot], [0.0, temp3[1], temp3[2], temp3[3]])
        return states_der

    def InferNN(self, states, inputs):

        network_inputs = np.append(states[3:], inputs)
        network_inputs[0] = 0.0
        input_normalize = False
        if input_normalize == True:
            scale_mean = np.array([8.17603532e-03, 5.53549084e+01, 1.00908206e-01, -7.04567338e-02, -5.63617684e-02, 6.20993527e-01])
            scale_std  = np.array([0.02579808, 18.2459027, 1.15707734, 0.4170532, 0.35862873, 0.57737644])
            network_inputs = (network_inputs - scale_mean) / scale_std
        print("input:   ", str(np.around(network_inputs, 6)))

        dynamics = (self.ptmodel.forward(torch.from_numpy(network_inputs).to(self.device).float())).tolist()
        dynamics[0] = 0.0
        # sw: Looks like there are ranges the model works well. 2020-11-14
        dynamics[3] = (dynamics[3] - 0.042) * 10
        print("dynamics: "+str(np.around(dynamics, 6)))
        print("---------")
        # states_der = self.ptmodel(np.append(states[:4], inputs).cuda())
        kinematics = (self.kinematicBicycleModel(states, inputs))[:3]
        states_der = np.append(kinematics, dynamics)

        return states_der


# def main():


# if __name__ == "__main__":
#     main()