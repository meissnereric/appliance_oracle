# -*- coding: utf-8 -*-
# <nbformat>3.0</nbformat>

# <codecell>

from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
import pandas as pd

numIn = 30 #how many time steps are we looking at
numHidden = 3 #number of hidden layers
numOut = 1 # len(apps) * 2 (on/off?)

# <codecell>

net = buildNetwork(numIn, numHidden, numOut)
ds = SupervisedDataSet(numIn,numOut)
trainer = BackpropTrainer(net, ds)

def train():
    global net, ds, trainer
    print trainer.train()
    

# <codecell>

laptop = pd.read_csv('laptop.csv')
laptop.head()
laptop['time'] = laptop['time'].map( lambda t: float(t - laptop['time'][0]) * 0.001) #convert to milliseconds starting at 0

print len(laptop)
laptop.head()

# <codecell>

def toWindow(data):
    for r in range(0, len(laptop), numIn):
        curr = laptop['current'][r:r+numIn]
        print len(curr)
        print float(r)/30.0
        print max(curr)
        print min(curr)
        print
        yield curr
laptop_inputs = pd.Series(toWindow(laptop))
laptop_outputs = [0] * len(laptop_inputs)
laptop_outputs[19:38] = [1] * 19
laptop_outputs[-8:-5] = [1] * 3
print laptop_outputs[15:40]
laptop_inputs.head()

# <codecell>

for i in range(len(laptop_outputs)):
    if len(laptop_inputs[i]) == numIn:
        ds.addSample(laptop_inputs[i], laptop_outputs[i])

# <codecell>

epoch_number = 10
for i in range(epoch_number):
    train()

# <codecell>


