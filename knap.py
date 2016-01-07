from itertools import combinations
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, MultipleLocator
#import plotly.plotly as py

# The problem, to include Gaussian input signals, can be reduced as follows:
# For each input appliance signal, a_i, take the mean of it's Gaussian and call that a_i'
# Now we solve a binary programming (integer programming) problem of minimizing the squared error that the mean is away
# from our true power, P, where each signal, a_i, is the weight for a binary variable x_i.
# It's clearly and definitely an NP-Complete problem, solvable by any integer programming solver available. 
# For now I'll implement a naive, optimal solver but we can't keep that for all cases as it's too hard. Have to plug
# into an already available solver to solve big problems

def returnError(state, total_power):
    return abs(sum(app.power for app in state) - total_power)

def find_sum_in_list(apps, target):
    results = []
    for x in range(len(apps)):
        results.extend([combo for combo in combinations(apps, x) if sum(app.power for app in combo) == target])   
    return results

def err(tup):
    return abs(sum(app.power for app in tup[0]) - tup[1])

#Returns all combinations for a daay with error leq than X% of target power. X = 0.05, type number
def findBestWithinX(apps, target, X):
    results = []
    bestWithinXCombos = []
    for x in range(len(apps)):
        for combo in combinations(apps,x):
            currErr = err( (combo, target) ) 
            if currErr <= abs(X*float(target)):
                bestWithinXCombos.append(combo)
    return zip(bestWithinXCombos, map(err, zip(bestWithinXCombos,[target]*len(bestWithinXCombos))))

#apps only need to come in with some notion of "power" thats comparable. Could be a static power, a Gaussian mean, etc.
def findBestN(apps, target, N):
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

def findNSets(home, D, N):
    sets = []
    for d in D:
        sets.append(findBestN(home, d, N))
    return sets
    
def findSetsWithinX(home, D, X=0.05):
    sets = []
    for d in D:
        sets.append(findBestWithinX(home, d, X))
    return sets

def getAppOnOffMap(sets, D, home):
    A = [ [0.0] * len(D) for app in home]
    for i, app in enumerate(home):
        for j, on in enumerate(A[i]):
            for combo in sets[j]:
                if app in combo[0]:
                    A[i][j] += 1
            #scale the number by the number of possible combinations for that timestep
            if len(sets[j]) > 0:
                A[i][j] = A[i][j] / float(len(sets[j]))
    return A

def plotOnOffMap(lstA, D):
    plt.figure( figsize=(10,7.5))
    plt.subplot(len(lstA)+1,1,1)
    plt.imshow(lstA[0], cmap='Greys', interpolation='nearest', aspect="auto")
    plt.plot(D)
    for i, A in enumerate(lstA):
        plt.subplot(len(lstA)+1,1,2+i)
        plt.imshow(A, cmap='Greys', interpolation='nearest')
    plt.show()

def plotOnOffSingle(A, D, home):
    #fig = plt.figure()
    fig = plt.figure( figsize=(10,7.5))
    fig.patch.set_facecolor('white')
    fig.subplots_adjust(hspace=0.1)
    sp1 = fig.add_subplot(211)
    sp1.plot([abs(d) for d in D])
    sp1.set_ylabel("Power Consumption (W)")
    sp1.axes.get_xaxis().set_visible(False)
    sp1.axes.set_xticks([])
    sp1.spines['left'].set_visible(False)
    sp1.spines['right'].set_visible(False)
    sp1.spines['top'].set_visible(False)
    sp1.tick_params(
        axis='y',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        right='off',      # ticks along the bottom edge are off
        left='off',         # ticks along the top edge are off
        labelleft='on') # labels along the bottom edge are off
    sp1.tick_params(
        axis='x',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        bottom='off',      # ticks along the bottom edge are off
        top='off',         # ticks along the top edge are off
        labelbottom='on') # labels along the bottom edge are off

    sp2 = fig.add_subplot(212)
    sp2.imshow(A, cmap='Greys', interpolation='nearest', aspect="auto")
    sp2.set_xlabel("Time (5 min. Intervals)")
    sp2.set_ylabel("Appliance Running?")
    sp2.spines['left'].set_visible(False)
    sp2.spines['right'].set_visible(False)
    sp2.spines['top'].set_visible(False)
    sp2.spines['bottom'].set_visible(False)
    sp2.set_yticks(range(len(home)))
    sp2.set_yticklabels([app.name for app in home], fontsize=10)
    sp2.tick_params(
        axis='y',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        right='off',      # ticks along the bottom edge are off
        left='off',         # ticks along the top edge are off
        labelleft='on') # labels along the bottom edge are off
    sp2.tick_params(
        axis='x',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        bottom='off',      # ticks along the bottom edge are off
        top='off',         # ticks along the top edge are off
        labelbottom='on') # labels along the bottom edge are off
    plt.savefig("appliances.png")
    plt.show()
    #unique_url = py.iplot_mpl(fig, filename="plotly_app")
    #return unique_url


