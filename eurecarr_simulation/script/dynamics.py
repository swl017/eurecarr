import numpy as np

class Dynamics(object):
    def __init__(self, stateDim, inputDim, dt):
        self.dt         = dt
        self.steerRatio    = -20.0 * np.pi / 180
        self.throttleRatio = 1
        self.Cm1        = 2853.6789 #0.287
        self.Cm1_brake  = 356.1283 
        self.Cm2        = 0.0#0.0545
        self.Cr0        = 71.272#0.0518
        self.Cr2        = 0.4440625#0.00035
        self.Br     	= 7.3689#3.385
        self.Cr 	    = 1.9589#1.2691
        self.Dr     	= 4702.8301#0.173
        self.Bf	        = 7.0215#2.579
        self.Cf     	= 1.9148#1.2
        self.Df     	= 3621.6843#0.192
        self.m       	= 481.6#0.041
        self.Iz     	= 550#27.8E-6
        self.length     = 2.971 # 0.15 
        self.width      = 2.03 # 0.08 
        self.lf         = 1.738476549225652 # 0.15/2.0
        self.lr         = self.length - self.lf

        # self.A          = np.array([[0, 0, -0.1084,   0.995, -0.09983,      0],\
        #                             [0, 0,  0.5921, 0.09983,    0.995,      0],\
        #                             [0, 0,       0,       0,        0,      1],\
        #                             [0, 0,       0,  -2.437,    6.757, 0.5998],\
        #                             [0, 0,       0,   4.357,    -5657, -1.018],\
        #                             [0, 0,       0,    1373,    -1041,   -966]])
        # self.B          = np.array([[     0,     0],\
        #                             [     0,     0],\
        #                             [     0,     0],\
        #                             [-3.935, 9.052],\
        #                             [ 17.07,     0],\
        #                             [  3935,     0]])
        self.last_states = np.zeros([stateDim, 1])
        self.last_inputs = np.zeros([inputDim, 1])

        '''
        ###### Select Model Type #######
        0: Straight Linear Model
        1: Simple Bicycle Model
        2: Mixed Dynamic-Kinematic Bicycle Model
        3: Linearized Dynamic Bicycle Model(Currently not supported)
        '''
        self.modelType = 1


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
            blend_vx_min = 0.5
            blend_vx_max = 20.
            vx = states[4]
            blend = min(max((vx-blend_vx_min)/(blend_vx_max-blend_vx_min),0.0),1.0)
            states_der = blend*states_der_d + (1-blend)*states_der_k
        elif self.modelType == 3:
            ## Using Linearized Dynamic Bicycle Model
            states_der = self.linearBicycleModel(states, inputs)

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
        '''
        Body centered at the rear wheel
        '''
        steering_angle = inputs[0]*self.steerRatio
        last_steering_angle = self.last_inputs[0]*self.steerRatio

        x_dot, y_dot = self.local2global(states[4], states[5], states[2])
        yaw_dot      = states[4]*np.sin(steering_angle)/self.length
        roll_dot     = 0.0
        vx_dot       = self.motorModel(states, inputs)#inputs[1]
        vy_dot       = 0
        yaw_dotdot   = 1./self.length * (states[4] * (steering_angle-last_steering_angle)/self.dt * 1/(np.cos(steering_angle)**2)\
                + vx_dot * np.tan(steering_angle))
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
        '''
        Body centered at center of gravity(cog)
        '''
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
        F_ry = self.Dr * np.sin(self.Cr * np.arctan(self.Br*alpha_r))

        return F_fy, F_ry

    def motorModel(self, states, inputs):
        vx       = states[4]
        throttle = inputs[1]

        if(vx >= 0):
            sign_vx = 1
        else:
            sign_vx = -1

        if throttle>=0:
            Cm1 = self.Cm1
        else:
            Cm1 = self.Cm1_brake
        F_rx =(Cm1 - self.Cm2*abs(vx)) * throttle - (self.Cr0 + self.Cr2*vx**2)*sign_vx

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


def main():

    dt        = 0.01
    print("dt : "+str(dt) + " seconds")

    # states
    x0        = 0.0
    y0        = 0.0
    yaw0      = 0.0
    roll0     = 0.0
    vx0       = 10.0
    vy0       = 0.0
    yaw_dot0  = 0.0

    # inputs
    steering  = 0.0
    throttle  = 0.5

    states = np.array([x0, y0, yaw0, roll0, vx0, vy0, yaw_dot0])
    inputs = np.array([steering, throttle])

    state_dim = 7
    input_dim = 2
    model = Dynamics(state_dim, input_dim, dt)
    
    num_sim = 1
    for i in range(num_sim):
        states_der = model.forward(states, inputs)
        new_states = states + states_der * dt

        print("Sim step  : " + str(i))
        print("Inputs    : " + str(np.around(inputs, 3)))
        print("State     : " + str(np.around(states, 3)))
        print("              x'     y'    yaw'     roll'      vx'    vy'    yaw_dot'")
        print("State_der : " + str(np.around(states_der, 3)))
        print("State_del : " + str(np.around(states_der * dt, 3)))
        print("State_new : " + str(np.around(new_states, 3)))

        states = new_states

        print("====================")

if __name__ == "__main__":
    main()
