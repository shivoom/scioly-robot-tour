import math
totaldistance = 0
checkpoints = [(0,0), (0,25), (50, 25), (50, 175), 180, (50,75), (0,75), (-100, 125), (-150, 125), (-150, 25), 0, (-150, 175), (-2, 175)]
legnth = len(checkpoints)




for i in range(len(checkpoints)):
    if isinstance(checkpoints[i], tuple):
        totaldistance = math.dist(checkpoints[i], checkpoints[i +1])
    print(totaldistance)
    