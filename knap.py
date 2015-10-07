
from itertools import combinations

# The problem, to include Gaussian input signals, can be reduced as follows:
# For each input appliance signal, a_i, take the mean of it's Gaussian and call that a_i'
# Now we solve a binary programming (integer programming) problem of minimizing the squared error that the mean is away
# from our true power, P, where each signal, a_i, is the weight for a binary variable x_i.
# It's clearly and definitely an NP-Complete problem, solvable by any integer programming solver available. 
# For now I'll implement a naive, optimal solver but we can't keep that for all cases as it's too hard. Have to plug
# into an already available solver to solve big problems

def find_sum_in_list(apps, target):
    results = []
    for x in range(len(apps)):
        results.extend([combo for combo in combinations(apps, x) if sum(app.power for app in combo) == target])   
    return results

def err(tup):
    return abs(sum(app.power for app in tup[0]) - tup[1])

#apps only need to come in with some notion of "power" thats comparable. Could be a static power, a Gaussian mean, etc.
def findBest(apps, target, N):
    results = []
    bestNCombos = []
    for x in range(len(apps)):
        for combo in combinations(apps,x):
            if len(bestNCombos) < N:
                bestNCombos.append(combo)
            else:
                currErr = err( (combo, target) ) 
                maxLstErr = max(map(err, zip(bestNCombos,[target]*len(bestNCombos))))
                if currErr < maxLstErr:
                    for bcombo in bestNCombos:
                        if err( (bcombo, target) ) == maxLstErr:
                            bestNCombos.remove(bcombo)
                            bestNCombos.append(combo)
    return zip(bestNCombos, map(err, zip(bestNCombos,[target]*len(bestNCombos))))

def findSets(home, D, N):
    sets = []
    for d in D:
        sets.append(findBest(home, d, N))
    return sets
    
