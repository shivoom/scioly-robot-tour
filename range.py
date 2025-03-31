import math
totaldistance = 0
instructions = [(0,0), (0,25), (-100,25), 90, (100, 25), (100, 75), (50, 125), (0,125), (0,75), (-50, 75), (-50, 175), (0, 175), (-100, 175), (-100, 75)]
checkpoints = [(0,0), (0,25), (-100,25), (100, 25), (100, 75), (50, 125), (0,125), (0,75), (-50, 75), (-50, 175), (0, 175), (-100, 175), (-100, 75)]

for i in range(len(checkpoints) - 1):
        totaldistance += math.dist(checkpoints[i], checkpoints[i+1])
        
print(totaldistance)
