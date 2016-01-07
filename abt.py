import numpy as np
import pandas as pd
import scipy.signal
import cmath
from itertools import chain, izip
from sklearn.externals import joblib


# numHidden = 3 #number of hidden layers
# numOut = 1 #num_type * 2
# numInput = 31
# onOffShiftParam = 10
# onOffWindowSize = 2000
# onOffShiftSize = onOffWindowSize / onOffShiftParam
# nnWindowSize = 30000 #how many time steps are we looking at
# nnShiftSize = nnWindowSize / onOffShiftParam
# numOffZeros = 10000
# translateParam = 0.1

# on = "On"
# off = "Off"
# static = ""
# onOffTypesList = list(chain.from_iterable(map(lambda t: [t + on, t + off],Unq_type)))
# typesDict = dict(zip(Unq_type, range(len(Unq_type))))

def makeWindows(base_data):
    data = abs(base_data) ** 2
    #data = base_data / np.linalg.norm(base_data)
    outMean = np.mean(data)
    outStd = np.std(data)
    
    foundCycle = False
    startCycle = -1
    endCycle = -1
    tol = outStd*1.5
    window = 10000
    totalLen = 9 # 9 * 10000 @ 30,000Hz =  3 second window
    
    first_window = np.mean(map(abs,data[-1 * window:]))
    windows = []    
    mins = []
    maxs = []
    for i, _ in list(enumerate(data))[::-1]:
        curr_win = data[i-window:i]
        if i < window:
            continue
        if i % window == 0:
            windows.append(curr_win)
            mins.append(min(curr_win))
            maxs.append(max(curr_win))
            mv_av = np.mean(map(abs,curr_win))
            diff = abs(mv_av-first_window)
            #print mv_av
    sums = map(np.mean,windows[::-1])
    while len(sums) < totalLen:
        if len(sums) > 0:
            sums.append(sums[-1])
            mins.append(mins[-1])
            maxs.append(maxs[-1])
        else:
            sums.append(0.0)
            mins.append(0.0)
            maxs.append(0.0)
    fv =  list(chain.from_iterable([sums[:totalLen], mins[:totalLen], maxs[:totalLen]]))
    return fv


def centroid(mags, freqs):
    return np.dot(mags, freqs) / np.sum(mags)
    
def windowToInputFeatures(window):
    featureVector = []
    N = 2
    #fft = np.fft.rfft(window)
    
    fft = np.fft.rfft(window)
    ps = abs(fft) ** 2
    time_step = 1.0 / 30000.0
    freqs = np.fft.rfftfreq(window.shape[-1], time_step)
    #idx = np.argsort(freqs)
    #plt.plot(freqs[idx], ps[idx])

    topNInd = np.argpartition(fft, -1 * N)[-1 * N:]
    topNInd.sort()
    #featureVector.extend(topNFreq)
    topNFreq = freqs[topNInd]
    topNMags = fft[topNInd]
    polars = map(cmath.polar,topNMags)
    p0 = [p[0] for p in polars]
    p1 = [p[1] for p in polars]
    featureVector.extend(topNFreq)
    cent = cmath.polar(centroid(fft, freqs))
    featureVector.append(cent[0])
    featureVector.append(cent[1])
    #maxi = np.nanmax(window)
    #mini = np.nanmin(window)
    #featureVector.append(maxi)
    #featureVector.append(mini)
    w = makeWindows(window)
    featureVector.extend(w)
    return featureVector

def inputFeaturesToFullVector(input_features, t, action_type):
    global numOut, numIn, onOffTypesList, typesDict
    final_vec = input_features
    final_vec.append(typesDict[t])
    return final_vec

def parseIntoTurningOnAndStable(base_data):
    totalSeconds = 3
    hz = 30000
    return base_data[:totalSeconds * hz], base_data[totalSeconds * hz:]

def checkConfidence(model, prediction):
    #predict_log_proba(X)
    #predict_proba(X) -> returns array of probabilites for each class
    return 1.0

def predictRisingClass(window):
    turningOn, running = parseIntoTurningOnAndStable(window)
    finalTurningOn = windowToInputFeatures(turningOn)   
    abt = joblib.load('models/bdt_discrete_plaid_all.pkl') 
    prediction = abt.predict(finalTurningOn)
    probabilities = abt.predict_proba(finalTurningOn)
    return prediction, probabilities

def addMarkedWindowToDS(window, markedClass):
    return

def batchLearnDS():
    return


