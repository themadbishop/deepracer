import json
import math
import pandas as pd
import numpy as np

with open("output.txt", "r") as filein:
    lines = filein.read().splitlines()


def calcbearing(this, last):
    direction = math.atan2(this[0] - last[0], this[1] - last[1])
    print(direction)
    track_direction = (math.pi/2 - direction)*180/math.pi
    print(track_direction)
    if track_direction > 180:
        track_direction = track_direction - 360
    return track_direction

entrylist = []
for x in lines:
    entry = {}
    x = str(x).replace("'", '"')
    x = str(x).replace("True", 'true')
    x = str(x).replace("False", 'false')
    x = str(x).replace("(", '[')
    x = str(x).replace(")", ']')
    y = json.loads(json.loads(json.dumps(x)))
    params = y['params']
    waypoints = params['waypoints']
    heading = params['heading']
    closest_waypoints = params['closest_waypoints']
    last = waypoints[closest_waypoints[0]]
    next1 = waypoints[closest_waypoints[1]]
    next2 = waypoints[(closest_waypoints[1]+1)%len(waypoints)]
    next3 = waypoints[(closest_waypoints[1]+2)%len(waypoints)]
    entry['last'] = json.dumps(last)
    entry['next1'] = json.dumps(next1)
    entry['next2'] = json.dumps(next2)
    entry['next3'] = json.dumps(next3)
    for key in params.keys():
        if key in ['heading','distance_from_center','speed','steps']: entry[key] = params[key]
    errors = y['errors']
    for key in errors.keys():
        entry[key] = errors[key]
    rewards = {}
    if 'rewards' in y: rewards = y['rewards']
    for key in rewards.keys():
        entry[key] = rewards[key]
    #if 'finalrewards' in y: 
    entry['finalreward']=y['finalreward']
    #else:
    #    if 'rewards' in y:
    #        entry['finalreward'] = 0
    #        #np.product([ rewards[key] for key in rewards.keys()])
    entrylist.append(entry)

    logframe = pd.DataFrame( entrylist)
    #print(entry.keys())

printlog = logframe[:150]
print(printlog.columns)
for idx, row in printlog.iterrows():
    print( '{:<40}{:>4}{:>15}{:>15}{:>15}{:>15}{:>15}{:>15}'.format( 'coords (last,1,2,3)', "", 'heading', 'w1', 'w2', 'w3', 'dfc','speed' ) )
    print( '{:<40}{:>4.0f}{:>15.8f}{:>15.8f}{:>15.8f}{:>15.8f}{:>15.8f}{:>15.8f}'.format( row['last'], row['steps'], row['heading'], row['w1'], row['w2'], row['w3'], row['distance_from_center'],row['speed'] ) )
    print( '{:<40}{:>4}{:>15}{:>15.8f}{:>15.8f}{:>15.8f}{:>15.8f}{:>15.8f}'.format( row['next1'],"", "", row['w1e'], row['w2e'], row['w3e'], row['dfce'], row['se'] ) )
    print( '{:<40}{:>4}{:>15}{:>15.8f}{:>15.8f}{:>15.8f}{:>15.8f}{:>15.8f}'.format( row['next2'],"", "", row['w1p'], row['w2p'], row['w3p'], row['dfcp'], row['sp'] ) )
    print( '{:<40}{:>4}{:>15.8f}'.format( row['next3'], "", row['finalreward'] ) )
    print( "" )

    






