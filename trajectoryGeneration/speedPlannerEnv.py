import numpy as np
import queue as Q
import math
import matplotlib.pyplot as plt
import copy

import time

class State(object):
    def __init__(self,pathDist,speed,acceleration,time):
        self.pathDist = pathDist
        self.speed = speed
        self.acceleration = acceleration
        self.time = time

class Obstacle(object):
    # polyPoints expected to be : NX2 (t,s)
    def __init__(self,polyPoints):
        self.polyPoints = polyPoints
        self.n = polyPoints.shape[0]

    # Checks if test point is on left or right of the line
    # formed by pointA and pointB
    def CheckIfOnHalfPlane(self,pointA,pointB,testPoint):
        out = np.cross(testPoint - pointA,pointB - pointA)

        if out>=0:
            return 1
        else:
            return -1


    def CollisionCheck(self,test):

        positive = 0
        negative = 0
        for i in range(self.n):
            if self.CheckIfOnHalfPlane(self.polyPoints[i-1,:], self.polyPoints[i,:], test) >= 0:
                positive = positive + 1
            else:
                negative = negative + 1

        if positive>0 and negative>0:
            return 0
        else:
            return 1


class SpeedPlannerEnv(object):
    def __init__(self,speedProfile,accD,accDDerivative):

        # Assumption: speed profile is a numpy array with each row as pathDist, speed
        self.speedProfile = speedProfile

        self.accD = accD
        self.accDDerivative = accDDerivative

        # Static obstacle
        self.staticObstacleEnable = 0
        self.timeStart = 0
        self.timeStop = 1
        self.distStart = 0
        self.distStop = 1

    # Returns the speed limit at the input path instant
    def GetSpeedProfileLimit(self,pathInstant):
        return np.interp(pathInstant, self.speedProfile[:,0], self.speedProfile[:,1])

    # Check if state is in collision with ACC
    def CheckACCCollision(self,state):
        if self.accDDerivative!=0:
            time = (state.pathDist - self.accD)/self.accDDerivative
        else:
            if state.pathDist>self.accD:
                return 1
            else:
                return 0

        if time<state.time or time<0:
            return 0
        else:
            return 1

    def CompareACCSpeed(self,inputSpeed):
        if inputSpeed<=self.accDDerivative:
            return 1
        else:
            return 0

    # Enable crossing obstacle
    def EnableObstacle(self,timeStart,timeStop,distStart,distStop):
        self.staticObstacleEnable = 1
        self.timeStart = timeStart
        self.timeStop = timeStop
        self.distStart = distStart
        self.distStop = distStop

    # Disable crossing obstacle
    def DisableObstacle(self):
        self.staticObstacleEnable = 0

    def GetSpeedPlanBetweenStates(self,state1,state2):
        oldPathDist = state1.pathDist
        oldSpeed = state1.speed
        oldAcc = state1.acceleration
        oldTime = state1.time

        dtIntervals = (state2.time - oldTime) / self.dtDelta
        accelerationIncrement = (state2.acceleration - oldAcc) / dtIntervals

        speedPlan = []

        for i in range(int(dtIntervals)):
            newTime = oldTime + self.dtDelta
            newAcceleration = oldAcc + accelerationIncrement
            newPathDist = oldPathDist + oldSpeed * self.dtDelta + 0.5 * self.dtDelta * self.dtDelta * (oldAcc + accelerationIncrement)
            newSpeed = oldSpeed + (oldAcc + accelerationIncrement)* self.dtDelta

            speedPlan.append([newPathDist,newSpeed,newAcceleration,newTime])

            oldPathDist = newPathDist
            oldSpeed = newSpeed
            oldAcc = newAcceleration
            oldTime = newTime

        return speedPlan

    # Check if there's a collision with the crossing obstacle
    # Since dt used is high
    # Check if the state is in the obstacle region of t-s space obstacle map is not enough
    # Thats why old state is also used to find if any of the states in between the oldState and newState is in
    # Collision
    # This has been tested only one case, might need more testing
    def CheckCollisionObstacle(self, state, oldState, speedPlan):

        if self.staticObstacleEnable==1:
            if state.pathDist>=self.distStart and state.pathDist<= self.distStop and state.time>=self.timeStart and state.time<=self.timeStop:
                return 1
            elif oldState==None:
                return 0
            elif oldState.pathDist>=self.distStart and oldState.pathDist<=self.distStop and oldState.time>=self.timeStart and oldState.time<=self.timeStop:
                return 1
            else:
                # This should not gappen as we
                if speedPlan==None:
                    return 0

                for i in range(1,len(speedPlan)-1,1):
                    speedConfig = speedPlan[0]
                    if speedConfig[0]>=self.distStart and speedConfig[0]<=self.distStop and speedConfig[3]>=self.timeStart and speedConfig[3]<=self.timeStop:
                        return 1
        else:
            return 0

    def CheckENVCollision(self,state,oldState,dt):
        if self.CheckACCCollision(state)==1 or self.CheckCollisionObstacle(state,oldState,dt)==1:
            return 1
        else:
            return 0

    def CheckCollisionConfig(self, oldState, state):

        s2 = state[0]
        t2 = state[1]

        s1 = oldState[0]
        t1 = oldState[1]

        if self.staticObstacleEnable==1:
            if s2>=self.distStart and s2<= self.distStop and t2>=self.timeStart and t2<=self.timeStop:
                return [self.distStart, self.timeStop], True
            elif oldState==None:
                return [], False
            elif s1>=self.distStart and s1<=self.distStop and t1>=self.timeStart and t1<=self.timeStop:
                return [self.distStart, self.timeStop], True
            else:
                return [], False
        else:
            return [], False


