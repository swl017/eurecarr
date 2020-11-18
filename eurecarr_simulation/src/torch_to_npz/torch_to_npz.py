import torch
import numpy as np
from model import *
import datetime

device = torch.device('cpu')

# Load model 
PATH = '/home/sw/Desktop/torch_to_npz/checkpoint_9000-0.41358309984207153.pt'
input_size = 6
output_size = 4
Mymodel = NeuralNet(input_size,output_size)
Mymodel.load_state_dict(torch.load(PATH,map_location=device), strict=False)

print(Mymodel)

print(Mymodel.fc1.weight.shape)
print(Mymodel.fc2.weight.shape)
print(Mymodel.fc3.weight.shape)
print(Mymodel.fc1.bias.shape)
print(Mymodel.fc2.bias.shape)
print(Mymodel.fc3.bias.shape)

Mymodel.fc1.weight.shape
Mymodel.fc2.weight.shape
Mymodel.fc3.weight.shape
Mymodel.fc1.bias.shape
Mymodel.fc2.bias.shape
Mymodel.fc3.bias.shape

# b = a
# b[0] = np.float64(a[0].T)
# b[1] = np.float64(a[1])
# b[2] = np.float64(a[2].T)
# b[3] = np.float64(a[3])
# b[4] = np.float64(a[4].T)
# b[5] = np.float64(a[5])

b = []
b.append(np.float64(Mymodel.fc1.weight.to(float).detach().numpy()))
b.append(np.float64(Mymodel.fc1.bias.to(float).detach().numpy()))
b.append(np.float64(Mymodel.fc2.weight.to(float).detach().numpy()))
b.append(np.float64(Mymodel.fc2.bias.to(float).detach().numpy()))
b.append(np.float64(Mymodel.fc3.weight.to(float).detach().numpy()))
b.append(np.float64(Mymodel.fc3.bias.to(float).detach().numpy()))

for i in range(0,len(b)):
    print(b[i].shape)

print(Mymodel.fc2.weight.shape)
print(Mymodel.fc2.weight)

label = ['dynamics_W1','dynamics_b1','dynamics_W2','dynamics_b2','dynamics_W3','dynamics_b3']
d={}
for l in label:
    d[l] = b[label.index(l)]
weightnpzName = 'checkpoint_9000-0.41358309984207153_' + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
np.savez(weightnpzName + '.npz', **d)
print("Saved weight as: " + weightnpzName + '.npz')
