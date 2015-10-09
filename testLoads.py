import data
import loads
import knap
import matplotlib.pyplot as plt

# D[i] is the required demand in watts
# 0<i<n, where n = 24*60/5 (5 min intervals)
sampleTime = 5 #in minutes, connot be greater than one hour
numberOfHours = 24 #can be multi day
maxTime = numberOfHours*60/sampleTime             # number of sampleTime minute intervals

#HOME LIST OF LOADS
#loads.dryerMachine,
#list of loads in the home
home = [loads.washingMachine, loads.dryerMachine, loads.lightsEarly, loads.lightsLate, loads.fridge, loads.kettle, loads.coffeeMaker] #, loads.airConditioner] #, loads.coffeeMaker,
#home = [loads.fridge, loads.lightsEarly, loads.lightsLate] #, loads.draw]
D = [d for d in data.calculateDemand(sampleTime, numberOfHours, home)]

sets = knap.findSets(home, D, 1)
#for s in zip(sets, D):
#    print s
#    print
print "**************************"
print set(sets[1][0][0])
A = [ [0] * len(D) for app in home]
print type(A)
for i, app in enumerate(home):
    A[i] = [1 if app in set(sets[j][0][0]) else 0 for j, on in enumerate(A[i])]
    print A[i]
plt.figure( figsize=(10,7.5))
plt.subplot(211)
plt.plot(D)
plt.subplot(212)
plt.imshow(A, cmap='Greys', interpolation='nearest')
plt.show()
