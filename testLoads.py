import data
import loads
import knap

# D[i] is the required demand in watts
# 0<i<n, where n = 24*60/5 (5 min intervals)
sampleTime = 5 #in minutes, connot be greater than one hour
numberOfHours = 24 #can be multi day
maxTime = numberOfHours*60/sampleTime             # number of sampleTime minute intervals

#HOME LIST OF LOADS
#loads.dryerMachine,
#list of loads in the home
home = [loads.washingMachine, loads.dryerMachine, loads.lightsEarly, loads.lightsLate, loads.fridge, loads.kettle,
        loads.coffeeMaker , loads.airConditioner , loads.coffeeMaker]
#home = [loads.fridge, loads.lightsEarly, loads.lightsLate] #, loads.draw]
D = [d for d in data.calculateDemand(sampleTime, numberOfHours, home)]
N = 3
X = 100

sets1 = knap.findNSets(home, D, N)
sets2 = knap.findSetsWithinX(home, D)
sets3 = knap.findSetsWithinX(home, D, 0.01)
sets4 = knap.findSetsWithinX(home, D, 0.10)
A1 = knap.getAppOnOffMap(sets1, D, home)
A2 = knap.getAppOnOffMap(sets2, D, home)
A3 = knap.getAppOnOffMap(sets3, D, home)
A4 = knap.getAppOnOffMap(sets4, D, home)
knap.plotOnOffMap([A1, A2, A3, A4], D)


