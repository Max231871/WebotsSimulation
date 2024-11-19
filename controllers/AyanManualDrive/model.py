import random

def getInstructions(speed, steering, position, image, endPos, time):
    print(time)
    return random.randint(1, 30), (random.random() - 0.5);
    
    
def computeFitnessScore(shortestDistance, time, maxSpeed):
     minTime = shortestDistance / maxSpeed
     return minTime / time;