#from collections import namedtuple
import numpy as np

import sys
sys.path.append("ros/src")

import abt
import knap
import dtw_node as dtw
from itertools import chain, combinations

#Steps:
# New signal comes in:
# 	ABT computes predictions
#	DTW computes predictions
#	Compute lists of scores for each State
# 	Condense each list of scores for a State into a single score for each State
#	

#appliance_class = namedtuple("app_class", "ID name")
#appliance = namedtuple("app", "ID name app_class steady_state_power")

#Utility function
def powerset(iterable):
    s = list(iterable)
    return chain.from_iterable(combinations(s, r) for r in range(len(s)+1))

class ApplianceClass:
	ID_COUNTER = 0
	def __init__(self, name):
		self.name = name
		self.ID = ApplianceClass.ID_COUNTER
		ApplianceClass.ID_COUNTER += 1

class Appliance:
	# the ID of an appliance is accessed using the id(Object) python function
	ID_COUNTER = 0
	def __init__(self, name, app_class, ssp):
		self.name = name
		self.app_class = app_class
		self.steady_state_power = ssp
		self.ID = Appliance.ID_COUNTER
		Appliance.ID_COUNTER += 1

class State:
	def __init__(self):
		final_score = -1.0
		appliances = {} # Set
		individual_scores = [] # For each change. This will be as large as the number of appliances. One score for each appliance possibly added to get this State. 

class System:
	def __init__(self, appliances):
		self.appliances = appliances # list of appliances
		self.states = powerset(appliances) #PQ of the power set of appliances
		#self.predictions = [] # list of tuples for each timestep [(ABT, DTW, Power, TimeStep), ...]

	def getUnionedState(self, previous, app):
		unioned_state = [state for state in self.states if state.appliances == previous.union(set(app))]
		try:
			return unioned_state[0]
		except:
			print "Exception thrown while unioning states"
			pass

	def compute_new_states(self, new_signal, total_power):
		abt_probs = abt.predictRisingClass(new_signal) # [(id, score), (id, score), ...]
		dtw_probs = dtw.score_all_signatures(new_signal) # [ (id, score), (id, score), ...]
		for state in self.states:
			state.individual_scores = []

		for app in self.appliances:
			for state in self.states:
				try:
					s_hat = self.getUnionedState(state, app) #union
					knap_error = knap.returnError(state, total_power)
					abt_score = abt_probs[id(app.app_class)]
					dtw_score = dtw_probs[id(app)]
					self.predictions.append( (abt_probs) )
					de_score = (1 - (1 - abt_score) * (1-dtw_score) * knap_error)
					s_hat.individual_scores.append(de_score)
				except Exception:
					"Exception thrown in state scoring loop"

		for state in self.states:
			state.final_score = max(state.scores)

APPLIANCE_CLASSES = [ApplianceClass("Microwave"), ApplianceClass("Hairdryer")]
#Read from file or have a real database later 
APPLIANCES = [Appliance("Micro1", APPLIANCE_CLASSES[0], 1000.0), Appliance("Hair1", APPLIANCE_CLASSES[1], 1500.0)]
#Also read from file or database

global_system = System(APPLIANCES)

def test():
	global global_system
	global_system.compute_new_states(np.array([1,1,1,1,1,1,1,1,1]), 10)


