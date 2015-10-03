
from itertools import combinations
def find_sum_in_list(apps, target):
    results = []
    for x in range(len(apps)):
        results.extend([combo for combo in combinations(apps, x) if sum(app.power for app in combo) == target])   
    return results

def findSets(home, D):
    for d in D:
        yield find_sum_in_list(home, d)
    
