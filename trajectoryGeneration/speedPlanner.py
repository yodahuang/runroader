import numpy as np
import queue as Q
import math
import matplotlib.pyplot as plt
import  pickle
import copy

import time

from speedPlannerEnv import SpeedPlannerEnv
from speedPlannerEnv import State
from speedPlannerEnv import Obstacle
import trajectoryGenerator
import visualizer
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection

# /*
#  * Search based speed planner
#  * The inputs of this planner are:
#  * 1. Speed Limit profile. If the final speed of the vehicle required is 0 due to stop sign, traffic light etc, the speed
#  * profile limit should be zero at the station at which vehicle is required to be stopped and goal path should be set
#  * to this path value.
#  *
#  * 2. ACC Distance and ACC Derivative:
#  * The distance and speed of the ACC object or vehicle in front of the car. The speed should be an absolute value in m/s
#  *
#  * 3. Start State: path, speed, acceleration and time. Time is ignored
#  * 4. Goal State: path, speed, acceleration and time. Only path is considered and the speed plan is generated to be able to reach this goal as
#  * fast as possible
#  *
#  * Approach:
#  * 1. Minimum time solution:
#  * The essential component of this approach is the minimum time solution. The idea is to transform the speed profile into a speed profile which
#  * is possible for the car to follow. For ex: If the final speed of the speed profile limit is 0, The min time solution profile
#  * doesnt let the vehicle more speed than it can lose in time to reach the final path station.
#  *
#  * To compute the minimum time solution, a forward pass is made, where we start from the first speed limit profile state and project
#  * while increasing speed if the next value in speed limit profile is higher. If the next value in the speed limit profile is lower than
#  * the current speed, speed profile is not changed.
#  *
#  * After the forward pass a backward pass is done in which we start from the last state and then go backwards while limiting acceleration and
#  * jerk.
#  *
#  * The forward pass and backward pass convert the speed limit profile into a smooth profile. The forward and backward pass may not meet,
#  * leading to a straight path connecting the two.
#  *
#  * This speed profile is directly not feasible for the car to follow due to the jerk at the transition from smooth profile to flat profile.
#  *
#  * 2. A Star search:
#  * GetSuccessors: Each state is projected in time with different acceleration values from the acceleration set. Only the acceleration values
#  * which do not lead to jerk more than the maxJerk are allowed.
#  *
#  * The dt used for projecting the state is reduced from 2.0 to 0.5 as the search goes closer to the goal state. This is required for cases when the
#  * goal speed is 0.
#  *
#  * Each successor are checked for collision in t-s space.
#  *
#  *
#  * 3. Collision check:
#  * The problem of speed planning can be visualized in t-s space. Some part of t-s space maybe occupied indicating obstacle at certain distance and
#  * time range. Different kind of obstacles can be represented t-s space like stop sign, traffic light, cut-in vehicle, vehicle passing at the
#  * intersection, ACC, etc. The goal then becomes to find a path in t-s space to reach the s in min possible time.
#  *
#  * 4. There can be multiple ways to search for the final speed profile. One approach could be to search in t-s space and manipulating the
#  * grid connectivity to limit speed, acceleration and jerk.
#  * Another method is to project the state with some acceleration for a fixed dt.
#  *
#  * Second approach is used in this code
#  *
#  * 5. Minimum time heuristic:
#  * The minimum time heuristic computation uses the algorithm to compute the min time solution. Instead of starting from the
#  * start state of the speed limit profile. We start from the node state(s,v,a,t) and go until required to find the min time
#  * speed profile. The final time for the min time speed profile is used as heuristic.
#  *
#  * 6. Inevitable collision check:
#  * Each time a new node is created it is projected with max and min jerk. If both lead to collision, we add that node
#  * to closed set instead of the open set.
#  *
#  * 7. Implementation hacks to speed-up the search:
#  * a. New nodes are ignored if they are in vicinity of nodes already added. This is required as with varying dt the number of nodes becomes alot
#  * b. the time and acceleration cost is kept low compared to the heuristic to get a solution fast. This pretty much makes it kind of the best first search
#  *
#  *
#  * Some configurable parameters are in SpeedPlanner constructor
#  *
#  *
#  */

class Node(object):
    def __init__(self,state,parentNode,speedProfile):
        self.state = state
        self.parentNode = parentNode

        self.fCost = 0
        self.hCost = 0
        self.gCost = 0
        self.speedProfile = speedProfile

    def GetStateConfig(self):
        return [self.state.pathDist,self.state.speed,self.state.acceleration,self.state.time]

    def PrintNode(self):
        print("Node values:",self.state.pathDist," ",self.state.speed," ",self.state.acceleration," ",self.state.time)

def minimum(a,b):
    if a<b:
        return a
    else:
        return b

def maximum(a, b):
    if a > b:
        return a
    else:
        return b

def limit(val,min,max):
    if min>max:
        print("Problem with limit function: min cant be higher than max")
        return val

    if val<min:
        return min
    elif val>max:
        return max
    else:
        return val

class SpeedPlanner():

    def __init__(self,visualize,accD,accDDerivative):

        self.visualize = visualize

        # AStar related variables
        self.closedSet=set()
        self.fCostPQ=Q.PriorityQueue()
        self.nodeMap={}
        self.NodeID=-1

        self.startState = State(0, 0, 0, 0)
        self.goalState = State(0, 0, 0, 0)

        # plotting related variables
        self.pathLimit = 50
        self.timeLimit = 70
        self.speedLimit = 40

        # Some configurable parameters

        # Distance at which we start reducing the dt
        self.goalDistReducedResolution = 50

        # Acceleration cost
        self.accelerationCostScaler = 0.0

        # Time cost, no cost used so as to speed up the search
        self.timeCostScaler = 0.0

        # Scaler used for heuristic
        self.heuristicScaler = 1.0

        # Delta DT used to smoothen the speed profile during final stitching
        self.dtDelta = 0.25

        #Dt at the start of the search
        self.dtInitial = 2.0

        # dt used during
        self.dt = self.dtInitial

        # Max initial jerk
        self.maxJerkInitial = 0.50

        # Max jerk
        self.maxJerk = 0.5

        # self.accelerationSet = [-2.,-1.75,-1.5,-1.25,-1.,-0.75,-0.5,-0.25,0.,0.25,0.5,0.75,1.,1.25,1.5,1.75,2.]
        self.accelerationSet = [-5, -4, -3, -2., -1.5, -1., -0.5, 0., 0.5, 1., 1.5, 2., 2.5, 3.0]

        self.heuristicMax = 1000
        self.inevitableCollision = -1
        self.nodesExpanded = 0
        self.windowSize = 50
        self.marginToObstacle = 0

        # For the purpose of minimum time solution, should be same as limits of self.accelerationSet
        self.maxAcceleration = self.accelerationSet[-1]
        self.maxDeceleration = self.accelerationSet[0]

        # Speed limit profile, will later be updated real time based on road speed limit , etc
        self.speedProfile = []
        speed = list(np.linspace(25.0, 35.0, num=20)) + list(np.linspace(35.0, 35.0, num=20)) + list(np.linspace(35.0, 20.0, num=20)) + \
                list(np.linspace(20.0, 20.0, num=20)) + list(np.linspace(20.0, 10.0, num=20)) + [0]
        speedSize = len(speed)
        path = list(np.linspace(0.0, 50.0, num=speedSize))

        # Special Case:
        # If the ACC stopped in front, we should be able to either find a speed profile to stop before
        # or try more agressive braking
        # or tell thatcollision is inevitable
        # To search with stopped obstacle in front, we need to update the speed profile such that it converges to 0 speed before the obstacle
        # Goal distance should be set to lower than acc distance
        if accDDerivative == 0 and accD < self.windowSize:

            if accD - self.marginToObstacle < 0:
                print("Obstacle is too close, can't do anything")
            else:
                obstaclePathIndex = min(range(len(path)), key=lambda i: abs(path[i] - (accD - self.marginToObstacle))) + 1

                speed = speed[:obstaclePathIndex]
                speed[-1] = 0
                path = path[:obstaclePathIndex]
                path[-1] = (accD - self.marginToObstacle)

                speedSize = len(path)


        acceleration = list(np.linspace(0.0, 0.0, num=speedSize))
        time = list(np.linspace(0.0, 0.0, num=speedSize))
        self.speedProfile.append(path)
        self.speedProfile.append(speed)
        self.speedProfile.append(acceleration)
        self.speedProfile.append(time)
        self.speedProfile = np.transpose(np.array(self.speedProfile))

        # Get Min time solution
        self.minTimeSolutionProfile = self.GetMinimumTimeSolution(0,self.speedProfile[0][1], 0,self.speedProfile, self.maxAcceleration, self.maxDeceleration)
        self.env = SpeedPlannerEnv(self.minTimeSolutionProfile,accD=accD,accDDerivative=accDDerivative)
        self.finalSpeedPlan = None


        # path, speed, acceleration, time
        for i in range(1, len(self.minTimeSolutionProfile)):
            state = self.minTimeSolutionProfile[i]
            prevState = self.minTimeSolutionProfile[i-1]

            if prevState[2] == 0.0:
                t = prevState[3] + (state[0]-prevState[0]) / prevState[1]
            else:
                t = prevState[3] + (state[1] - prevState[1]) / prevState[2]

            self.minTimeSolutionProfile[i][3] = t


    # This helps in reducing the number of nodes expanded by ignoring nodes with similar s,v,a,t values
    def GetID(self,inputTuple):

        inputList = list(inputTuple)

        # Distance is rounded to 1m and speed is rounded to 0.5m/s
        # So if a node with same time and acceleration and similar path distance and velocity is added. It is not expanded upon
        inputList[0] = round(inputList[0])
        inputList[1] = round(inputList[1]*2)
        # inputList[2] = round(inputList[2])
        # inputList[3] = round(inputList[3])

        return tuple(inputList)


    def AddNodeToClosedSet(self,node):
        newTuple = (node.state.pathDist, node.state.speed, node.state.acceleration, node.state.time)

        self.closedSet.add(self.GetID(newTuple))

    def IsGoal(self,node):
        pathSnappingTol = node.state.speed * self.dt

        if pathSnappingTol < 0.5:
            pathSnappingTol = 0.5

        if math.fabs(node.state.pathDist - self.goalState.pathDist) <= pathSnappingTol:
            print("Dt when reached goal",self.dt)
            return 1
        else:
            # print("Not goal",node.state.speed-self.goalState.speed,",   ",node.state.pathDist - self.goalState.pathDist)
            return 0


    # This function returns the minimum time solution
    # It takes in the current speed, speed limit profile whose last speed is the speed target
    # Speed profile is provided in the format of v,s and the current speed
    # It runs a forward check and a backward using motion model (naive laws of motion for now) to get a
    # feasible minimum time solution.
    # This function is already implemented in the main software stack so might not be required later
    # Jerk is limited to 1

    # time returned by this function is not correct as we can't update time when doing backward pass
    def GetMinimumTimeSolution(self, pathDist, currentSpeed, currentAcceleration, speedProfileTemp,
                               maxAllowedAcceleration, maxAllowedDeceleration):

        speedProfile = copy.deepcopy(speedProfileTemp)

        speedProfile[0][0] = pathDist
        speedProfile[0][1] = currentSpeed
        speedProfile[0][2] = currentAcceleration
        speedProfile[0][3] = 0

        # forward pass
        for i in range(1, len(speedProfile), 1):
            configPathLast = speedProfile[i - 1][0]
            configSpeedLast = speedProfile[i - 1][1]
            configAccLast = speedProfile[i - 1][2]

            configPathNext = speedProfile[i][0]
            configSpeedNext = speedProfile[i][1]

            ds = math.fabs(configPathNext - configPathLast)

            if ds == 0:
                print("Why is ds 0??? exiting")

            if configSpeedNext > configSpeedLast:

                accReq = (configSpeedNext ** 2 - configSpeedLast ** 2) / (2 * ds)

                if accReq>maxAllowedAcceleration:
                    accReq=maxAllowedAcceleration
                    configSpeedNext = math.sqrt(configSpeedLast ** 2 + 2 * accReq * ds)
                    speedProfile[i][1] = configSpeedNext

                speedProfile[i][2] = accReq

                time = (configSpeedNext - configSpeedLast) / accReq

                jerkRequired = (accReq - configAccLast) / time
                accReqTemp = accReq
                if math.fabs(jerkRequired) > self.maxJerk:

                    if jerkRequired > 0:
                        dAcc = -0.01
                        maxAccAfterJerkLim = limit(accReqTemp + dAcc, configAccLast, maxAllowedAcceleration)
                        while maxAccAfterJerkLim > configAccLast:
                            vSquare = configSpeedLast ** 2 + 2 * maxAccAfterJerkLim * ds

                            # If this is happening it means the current node will inevitably reach 0 speed,
                            # so should not even be added to the open list
                            if  vSquare < 0:
                                # maxV = math.sqrt(-vSquare)
                                return []
                            else:
                                maxV = math.sqrt(vSquare)

                            # 0 acceleration, so we are pretty much as the same speed as the last speed
                            if math.fabs(maxAccAfterJerkLim) < 1e-4:
                                time = ds/maxV
                            else:
                                time = (maxV - configSpeedLast) / (maxAccAfterJerkLim)

                            if math.fabs((maxAccAfterJerkLim - configAccLast) / time) > self.maxJerk:
                                maxAccAfterJerkLim = limit(maxAccAfterJerkLim + dAcc, configAccLast, maxAllowedAcceleration)
                            else:
                                break
                        maxAccAfterJerkLim = limit(maxAccAfterJerkLim, maxAllowedDeceleration, maxAllowedAcceleration)
                    else:
                        dAcc = 0.01
                        maxAccAfterJerkLim = limit(accReqTemp + dAcc, 0, configAccLast)
                        while maxAccAfterJerkLim < configAccLast:
                            vSquare = configSpeedLast ** 2 + 2 * maxAccAfterJerkLim * ds

                            # This should never happen
                            if vSquare < 0:
                                return []
                            else:
                                maxV = math.sqrt(vSquare)

                            if math.fabs(maxAccAfterJerkLim) < 1e-4:
                                time = ds/maxV
                            else:
                                time = (maxV - configSpeedLast) / (maxAccAfterJerkLim)
                            if math.fabs((maxAccAfterJerkLim - configAccLast) / time) > self.maxJerk:
                                maxAccAfterJerkLim = limit(maxAccAfterJerkLim + dAcc, 0,
                                                           configAccLast)
                            else:
                                break
                        maxAccAfterJerkLim = limit(maxAccAfterJerkLim, 0, maxAllowedAcceleration)
                else:
                    if accReq <=maxAllowedAcceleration and accReq >=maxAllowedDeceleration:
                        continue
                    else:
                        maxAccAfterJerkLim = limit(accReq, maxAllowedDeceleration, maxAllowedAcceleration)

                vSquare = configSpeedLast ** 2 + 2 * maxAccAfterJerkLim * ds
                if vSquare < 0:
                    maxV = math.sqrt(-vSquare)
                else:
                    maxV = math.sqrt(vSquare)

                speedProfile[i][1] = maxV
                speedProfile[i][2] = maxAccAfterJerkLim

        # backward pass
        for i in range(len(speedProfile) - 1, 0, -1):
            configPathLast = speedProfile[i][0]
            configSpeedLast = speedProfile[i][1]
            configAccLast = speedProfile[i][2]

            configPathNext = speedProfile[i - 1][0]
            configSpeedNext = speedProfile[i - 1][1]

            ds = math.fabs(configPathNext - configPathLast)

            if i == len(speedProfile) - 1:
                configAccLast = 0
                # speedProfile[i][2] = configAccLast

            if ds == 0:
                print("Why is ds 0??? exiting")

            if configSpeedNext > configSpeedLast:

                accReq = (configSpeedLast ** 2 - configSpeedNext ** 2) / (2 * ds)

                if accReq<maxAllowedDeceleration:
                    accReq=maxAllowedDeceleration
                    configSpeedNext = math.sqrt(configSpeedLast ** 2 - 2 * accReq * ds)
                    speedProfile[i - 1][1] = configSpeedNext

                speedProfile[i - 1][2] = accReq

                time = (configSpeedLast - configSpeedNext) / (accReq)

                jerkRequired = (accReq - configAccLast) / time
                accReqTemp = accReq
                if math.fabs(jerkRequired) > self.maxJerk:

                    if jerkRequired>0:
                        dAcc = -0.01
                        maxAccAfterJerkLim = limit(accReqTemp + dAcc, configAccLast, 0)
                        while maxAccAfterJerkLim > configAccLast:
                            maxV = math.sqrt(configSpeedLast ** 2 - 2 * maxAccAfterJerkLim * ds)
                            if math.fabs(maxAccAfterJerkLim) < 1e-4:
                                time = ds / maxV
                            else:
                                time = (configSpeedLast - maxV) / (maxAccAfterJerkLim)

                            if math.fabs((maxAccAfterJerkLim - configAccLast) / time) > self.maxJerk:
                                maxAccAfterJerkLim = limit(maxAccAfterJerkLim + dAcc, configAccLast, 0)
                            else:
                                break
                        maxAccAfterJerkLim = limit(maxAccAfterJerkLim, maxAllowedDeceleration, 0)
                    else:

                        dAcc = 0.01
                        maxAccAfterJerkLim = limit(accReqTemp + dAcc, maxAllowedDeceleration, configAccLast)

                        while maxAccAfterJerkLim < configAccLast:
                            maxV = math.sqrt(configSpeedLast ** 2 - 2 * maxAccAfterJerkLim * ds)
                            if math.fabs(maxAccAfterJerkLim) < 1e-4:
                                time = ds / maxV
                            else:
                                time = (configSpeedLast - maxV) / (maxAccAfterJerkLim)

                            if math.fabs((maxAccAfterJerkLim - configAccLast) / time) > self.maxJerk:
                                maxAccAfterJerkLim = limit(maxAccAfterJerkLim + dAcc, maxAllowedDeceleration,
                                                           configAccLast)
                            else:
                                break
                        maxAccAfterJerkLim = limit(maxAccAfterJerkLim, maxAllowedDeceleration, 0)
                else:
                    if accReq >= maxAllowedDeceleration and accReq<=0:
                        continue
                    else:
                        maxAccAfterJerkLim = limit(accReq,maxAllowedDeceleration,0)

                maxV = math.sqrt(configSpeedLast ** 2 - 2 * maxAccAfterJerkLim * ds)
                if maxV-configSpeedLast>2:
                    a=1
                speedProfile[i - 1][1] = maxV
                speedProfile[i - 1][2] = maxAccAfterJerkLim

        return speedProfile

    def InterpolateLinear(self,startS,startV,endS,endV,path):
        if endS<startS:
            print("This should not happen, s should always increase")
            exit(0)

        return startV + (endV-startV)*(path-startS)/(endS-startS)



    # This function returns the minimum time solution
    # It takes in the current speed, speed limit profile whose last speed is the speed target
    # Speed profile is provided in the format of v,s and the current speed
    # It runs a forward check and a backward using motion model (naive laws of motion for now) to get a
    # feasible minimum time solution.
    # This function is already implemented in the main software stack so might not be required later
    # Jerk is limited to 1
    # To get a better heuristic value for speed planning among obstacles, collision check can also be done in this function
    # The speed profile computed can be checked for collision and if collision exists, it returns the speed profile from the
    # the initial configuration to the collision
    # This is described better in the GetHeuristic function
    def GetMinTimeHeuristic(self, pathDist, currentSpeed, currentAcceleration, currentTime, speedProfileTemp,
                            maxAllowedAcceleration,
                            maxAllowedDeceleration, collisionCheck=0,dt=1):

        pathList = list(speedProfileTemp[:, 0])
        startIndex = min(range(len(pathList)), key=lambda i: abs(pathList[i] - pathDist))

        speedProfile = copy.deepcopy(speedProfileTemp)

        # Add another v-s point before
        if speedProfile[startIndex][0] > pathDist:

            if startIndex==0:
                print("Problem, path should start from 0")
                exit(0)

            interpolatedSpeed = self.InterpolateLinear(speedProfile[startIndex-1][0],speedProfile[startIndex-1][1],speedProfile[startIndex][0],speedProfile[startIndex][1], pathDist)
            speedProfile = list(speedProfile)
            speedProfile.insert(startIndex,[pathDist,interpolatedSpeed,0,0])
            speedProfile = np.array(speedProfile)
        # Replace the first v-s point by interpolated point
        elif speedProfileTemp[startIndex][0] <pathDist:
            interpolatedSpeed = self.InterpolateLinear(speedProfile[startIndex][0], speedProfile[startIndex][1],
                                   speedProfile[startIndex+1][0], speedProfile[startIndex+1][1], pathDist)
            speedProfile[startIndex][1]=interpolatedSpeed

        # Cut the speed profile to start at station=pathDist
        speedProfile = speedProfile[startIndex:, :]

        minTimeProfile = self.GetMinimumTimeSolution(pathDist, currentSpeed, currentAcceleration, speedProfile,
                                                     maxAllowedAcceleration, maxAllowedDeceleration)

        if len(minTimeProfile)==0:
            return (0, 0, None, 0)

        # self.PlotSpeedPathPlan(minTimeProfile, color='-b')
        # plt.show()

        # It is same as minTime solution but with corrected time. It is cut short if a collision is found to state just before collision
        outputSpeedProfile = []

        time = 0
        collision = 0

        oldState = None

        # Should never lead to collision as we would have checked the first state before adding it it to the open list
        if collisionCheck == 1:
            newState = State(minTimeProfile[0][0], minTimeProfile[0][1], minTimeProfile[0][2], currentTime)
            if self.CheckCollision(newState,oldState):
                print("problem in min time heuristic")
                collision = 1
                return (collision, time, np.array(outputSpeedProfile),1)

            oldState = newState

        outputSpeedProfile.append([minTimeProfile[0][0], minTimeProfile[0][1], minTimeProfile[0][2], currentTime + time])

        for j in range(0, len(minTimeProfile) - 1, 1):

            acceleration = minTimeProfile[j + 1][2]
            speed = minTimeProfile[j + 1][1]

            if acceleration != 0:
                dTime = math.fabs((minTimeProfile[j + 1][1] - minTimeProfile[j][1]) / acceleration)
            elif speed > 0:
                dTime = math.fabs((minTimeProfile[j + 1][0] - minTimeProfile[j][0]) / speed)
            else:
                dTime = 0

            time = time + dTime

            if collisionCheck == 1:
                newState = State(minTimeProfile[j + 1][0], speed, acceleration, time)
                check_collision = self.CheckCollision(newState, oldState)
                if check_collision:
                    collision = 1
                    break

                oldState = newState

            outputSpeedProfile.append([minTimeProfile[j + 1][0], speed, acceleration, currentTime + time])

        return (collision, time, np.array(outputSpeedProfile),1)

    def CheckForInevitableCollision(self,node,dt=1.0):

        collision = []

        jerkList = [-self.maxJerk,self.maxJerk]

        for i in range(len(jerkList)):

            pathDist = node.state.pathDist
            speed = node.state.speed
            acceleration = node.state.acceleration
            time = node.state.time

            while True:
                # Project with jerk and check for collision
                nextAcceleration = acceleration + jerkList[i] * dt
                if nextAcceleration > self.maxAcceleration:
                    nextAcceleration = self.maxAcceleration
                elif nextAcceleration < self.maxDeceleration:
                    nextAcceleration = self.maxDeceleration

                nextPath = pathDist + speed * dt + 0.5 * nextAcceleration * dt * dt

                nextSpeed = speed + acceleration * dt

                nextTime = time + dt

                # Check for collision
                newState = State(nextPath, nextSpeed, nextAcceleration, nextTime)
                if self.env.CheckACCCollision(newState):
                    collision.append(1)
                    break

                # Conditions to confirm no collision
                if nextPath > self.goalState.pathDist or nextSpeed < 0:
                    collision.append(0)
                    break

                pathDist = nextPath
                speed = nextSpeed
                acceleration = nextAcceleration
                time = nextTime


        if sum(collision)<2:
            return 0
        else:
            return 1

    def GetHeuristic(self,node,dt):

        # Check if the node will lead to inevitable collision. If so, it will be added to closed list instead of openList
        if self.CheckForInevitableCollision(node,self.dt)==1:
            print("Inevitable collision")
            return self.inevitableCollision

        (collision,time,speedProfile,feasibility) = self.GetMinTimeHeuristic(node.state.pathDist,
                                                                 node.state.speed,
                                                                 node.state.acceleration,
                                                                 node.state.time,
                                                                 self.speedProfile,
                                                                 self.maxAcceleration,
                                                                 self.maxDeceleration,
                                                                 collisionCheck=0,dt=dt)

        # This is for the case when no jerk limited
        if feasibility==0:
            return self.inevitableCollision

        return time


    def GetCostList(self,node):

        # speedTarget = self.env.GetSpeedProfileLimit(node.state.pathDist)
        speedTarget = self.goalState.speed

        if node.parentNode != None:
            node.gCost = node.parentNode.gCost + self.timeCostScaler*(node.state.time-node.parentNode.state.time) + self.accelerationCostScaler * node.state.acceleration ** 2
        else:
            node.gCost = 0

        # Can ignore obstacles and use minimum time solution as the heuristic
        node.hCost = self.heuristicScaler*self.GetHeuristic(node, self.dt)

        node.fCost = node.gCost + node.hCost

        return [node.fCost,self.NodeID]

    def checkIfAlreadyAdded(self,node):
        newTuple = (node.state.pathDist, node.state.speed, node.state.acceleration, node.state.time)

        if self.GetID(newTuple) in self.closedSet:
            return 1
        else:
            return 0

    def CheckCollision(self,newState,oldState):

        speedLimit = self.env.GetSpeedProfileLimit(newState.pathDist)

        successorSpeedPlan = self.GetSpeedPlanBetweenStates(oldState, newState)

        if newState.speed>speedLimit or self.env.CheckENVCollision(newState,oldState,successorSpeedPlan)==1:
            return 1
        else:
            return 0

    def GetSuccessors(self,node):
        successorStateList = []

        currentPathDist = node.state.pathDist
        currentSpeed = node.state.speed
        currentAcceleration = node.state.acceleration
        currentTime = node.state.time

        oldState = State(currentPathDist,currentSpeed,currentAcceleration,currentTime)

        for action in self.accelerationSet:
            newState = State(0,0,0,0)

            if math.fabs(currentAcceleration - action)>self.maxJerk:
                continue

            dtIntervals = self.dt / self.dtDelta
            accelerationIncrement = (action - currentAcceleration) / dtIntervals;

            newState.time = currentTime + self.dt;
            newState.acceleration = action;

            newState.pathDist = currentPathDist + currentSpeed * dtIntervals * self.dtDelta + \
                                self.dtDelta * self.dtDelta * 0.5 * ((dtIntervals * currentAcceleration) + \
                                (accelerationIncrement * 0.5 * dtIntervals * (dtIntervals + 1))) + \
                                self.dtDelta * self.dtDelta * \
                                ((accelerationIncrement * \
                                ((dtIntervals - 1) + 2 * (dtIntervals - 1) * (dtIntervals - 2) * 0.5 + ((dtIntervals - 1) * (dtIntervals - 2) * (dtIntervals - 3)) / 6)) + \
                                 (currentAcceleration * (dtIntervals - 1) * (dtIntervals) * 0.5))

            newState.speed = currentSpeed + self.dtDelta * (currentAcceleration * dtIntervals +
                                                     accelerationIncrement * dtIntervals * (dtIntervals + 1) / 2.0);

            # The new state has to be ahead in path distance and speed and time
            if newState.pathDist>=currentPathDist and newState.speed>0 and newState.time>=currentTime and self.CheckCollision(newState,oldState)==0:
                successorStateList.append(newState)

        return successorStateList

    def InitializePlanner(self):
        self.closedSet.clear()
        self.fCostPQ=Q.PriorityQueue()
        self.NodeID=-1
        self.nodeMap={}

    def CheckForStoppedACC(self):
        if self.env.accDDerivative == 0 and self.env.accD<self.windowSize:
            if self.goalState.pathDist > (self.env.accD - self.marginToObstacle):
                self.goalState.pathDist = self.env.accD - self.marginToObstacle

    def search(self, startState, goalState):

        self.InitializePlanner()

        # Set the start and goal
        self.startState = copy.deepcopy(startState)
        self.goalState = copy.deepcopy(goalState)

        self.CheckForStoppedACC()

#       create root node
        root = Node(startState,None,None)
        inputCostList = self.GetCostList(root)

        if root.hCost == self.inevitableCollision:
            print("Inevitable collision")
            return None

        self.NodeID = self.NodeID + 1
        inputCostList[1]=self.NodeID
        self.fCostPQ.put(tuple(inputCostList))
        self.nodeMap[self.NodeID]=root

        if self.CheckCollision(startState,None):
            print("Start in collision. Path is empty")
            return None

        while not self.fCostPQ.empty():
            currentTuple=self.fCostPQ.get()
            currentNode=self.nodeMap[currentTuple[1]]

            # Reducing dt as we approach goal makes the search finer and
            # This is required to reach goals at lower speed.
            # It lower values make the last path of speed profile smoother
            # These values also affect the search time
            if math.fabs(currentNode.state.pathDist - self.goalState.pathDist) < 0.50 * self.goalDistReducedResolution:
                self.dt = self.dtInitial / 4.0
            elif math.fabs(currentNode.state.pathDist - self.goalState.pathDist) < self.goalDistReducedResolution:
                self.dt = self.dtInitial / 2.0
            else:
                self.dt = self.dtInitial

            if self.visualize:
                self.PlotSpeedPlanWhileSearching(currentNode)

            currentNode.PrintNode()
            print(currentNode.hCost)
            self.nodesExpanded += 1

            # print("Node Expanded:", currentNode.state.x, " ", currentNode.state.y, " ", currentNode.state.theta)
            if self.IsGoal(currentNode):
                break

            if currentNode.state.speed!=1:
                a=1

            successorStates=self.GetSuccessors(currentNode)
            for successor in successorStates:

                newNode = Node(successor, currentNode,None)

                # Before we calculate the cost which will increase the NodeID, we check if the node is new or it has already been
                # added before.. For this we use the new node and current node(parent). These two nodes uniquely identfies a path
                if self.checkIfAlreadyAdded(newNode):
                    continue
                else:
                    inputCostList = self.GetCostList(newNode)

                    if newNode.hCost==self.inevitableCollision:
                        print("Inevitable collision, not using node")

                        # Add it to closed node so that we dont have to check it again
                        self.AddNodeToClosedSet(newNode)

                        continue

                    self.NodeID = self.NodeID + 1
                    inputCostList[1]=self.NodeID
                    self.fCostPQ.put(tuple(inputCostList))
                    self.nodeMap[self.NodeID] = newNode
                    self.AddNodeToClosedSet(newNode)

        # START FROM HERE:::::::
        finalSpeedPlan=[]
        finalNumNodes=0

        # print("Printing all the final path nodes and their parents")
        while currentNode.parentNode!=None:
            finalSpeedPlan = finalSpeedPlan + self.GetSpeedPlanBetweenStates(currentNode.parentNode.state,currentNode.state)[::-1]
            currentNode = currentNode.parentNode
            finalNumNodes = finalNumNodes+1

        finalSpeedPlan.append(currentNode.GetStateConfig())

        if len(finalSpeedPlan)>0:
            self.finalSpeedPlan = finalSpeedPlan[::-1]
            return finalSpeedPlan[::-1]
        else:
            self.finalSpeedPlan = None
            return None

    def GetSpeedPlanBetweenStates(self,state1,state2):

        if state1 == None or state2 == None:
            return None

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


    def PlotSpeedPlanWhileSearching(self,node):

        if node.parentNode==None:
            return

        parentPath = node.parentNode.state.pathDist
        parentSpeed = node.parentNode.state.speed

        currentPath = node.state.pathDist
        currentSpeed = node.state.speed

        plt.plot([parentPath,currentPath], [parentSpeed,currentSpeed], 'g.', linewidth=2, markersize=12)
        # plt.axis("equal")
        plt.grid(True)
        plt.ylim(0, self.speedLimit)
        plt.xlim(0, self.pathLimit)
        plt.pause(0.0001)

    def PlotSpeedTimePlan(self,speedPlan,color='-r'):
        if speedPlan is None:
            print("Cant't plot, empty speed plan")
            return
        speedPlanArray= np.array(speedPlan)
        time = speedPlanArray[:, 3]
        speed = speedPlanArray[:, 1]

        plt.plot(time, speed, color)
        # plt.axis("equal")
        plt.ylim(0, self.speedLimit)
        plt.xlim(0, self.timeLimit)
        plt.grid(True)

    def PlotSpeedPathPlan(self,speedPlan,color='-r'):
        if speedPlan is None:
            print ("Cant't plot, empty speed plan")
            return
        speedPlanArray= np.array(speedPlan)
        path = speedPlanArray[:, 0]
        speed = speedPlanArray[:, 1]

        plt.plot(path, speed, color)
        plt.ylim(0, self.speedLimit)
        plt.xlim(0, self.pathLimit)
        # plt.axis("equal")
        plt.grid(True)
        # plt.savefig('SpeedPlan:' + ".png")

    def PlotPathTimePlan(self,speedPlan,color='-r'):
        if speedPlan is None:
            print("Cant't plot, empty speed plan")
            return
        speedPlanArray= np.array(speedPlan)
        path = speedPlanArray[:, 0]
        time = speedPlanArray[:, 3]

        plt.plot(path, time, color)
        # plt.axis("equal")
        plt.ylim(0, self.timeLimit)
        plt.xlim(0, self.pathLimit)
        plt.grid(True)
        # plt.savefig('SpeedPlan:' + ".png")

    def PlotACCPathSpeed(self,accD,accDerivative,goalDist,color='-k'):
        speed = accDerivative*np.ones(100)
        path = np.linspace(accD, goalDist, num=100)

        plt.plot(path, speed, color)
        # plt.axis("equal")
        plt.ylim(0, self.speedLimit)
        plt.xlim(0, self.pathLimit)
        plt.grid(True)

    def PlotACCPathTimePlan(self,accD,accDerivative,goalDist,color='-k'):
        speed = accDerivative*np.ones(100)
        path = np.linspace(accD, goalDist, num=100)
        initialTime =0

        time = np.zeros(100)
        time[0] = initialTime
        for i in range(1,len(path)):
            time[i]=time[i-1] + 2*(path[i] - path[i-1])/(speed[i] + speed[i-1])

        plt.plot(path, time, color)
        # plt.axis("equal")
        plt.xlim(0, self.pathLimit)
        plt.ylim(0, self.timeLimit)
        plt.grid(True)


def CheckCollisionConfig(env, oldConfig, config):
    config, collision = env.CheckCollisionConfig(oldConfig, config)
    if collision:
        return config,True
    else:
        return config,False

def CheckCollisionSpeedProfile(env, speedProfile):
    for i in range(1,len(speedProfile)):
        new = [speedProfile[i][0], speedProfile[i][3]]
        old = [speedProfile[i-1][0], speedProfile[i-1][3]]
        val, collision = CheckCollisionConfig(env, old, new)
        if collision:
            return val, True

    return [], False

# This can be used to enable a obstacle crossing the vehicle path in front of the car
# parameters: startTime, endTime, startPath, endPath
if __name__ == "__main__":

    # Farther obstacle without ACC
    planner = SpeedPlanner(visualize=0, accD=500.0, accDDerivative=10.0)
    s1 = 30.
    s2 = 40.
    t1 = 0.5
    t2 = 3.0
    planner.env.EnableObstacle(t1, t2, s1, s2)
    startState = State(0, 17.0, 0, 0)
    goalState = State(planner.windowSize, 0, 0, 0)

    startTime = time.time()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    planner.PlotPathTimePlan(planner.minTimeSolutionProfile, color='-r')

    # Optimization based method
    print(planner.minTimeSolutionProfile)
    val, collision = CheckCollisionSpeedProfile(planner.env, planner.minTimeSolutionProfile)
    if collision:
        traj = trajectoryGenerator.test_optimize_speed_profile(initialSpeed=17., initial_aceleration=0., final_s=val[0],
                                final_t=val[1], maxSpeed=20., minSpeed=1.,
                                max_acc=4.)

        if len(traj):
            print("Optimization successful")

            speedProfile = []
            speed = list(np.linspace(25.0, 35.0, num=20)) + list(np.linspace(35.0, 35.0, num=20)) + list(
                np.linspace(35.0, 20.0, num=20)) + \
                    list(np.linspace(20.0, 20.0, num=20)) + list(np.linspace(20.0, 10.0, num=20)) + [0]
            speedSize = len(speed)
            path = list(np.linspace(val[0], 50.0, num=speedSize))
            acceleration = list(np.linspace(0.0, 0.0, num=speedSize))
            time = list(np.linspace(0.0, 0.0, num=speedSize))
            speedProfile.append(path)
            speedProfile.append(speed)
            speedProfile.append(acceleration)
            speedProfile.append(time)
            speedProfile = np.transpose(np.array(speedProfile))


            minTimeSolutionProfile = planner.GetMinimumTimeSolution(val[0], 1./traj[-1].yaw, 0, speedProfile,
                                                                      planner.maxAcceleration, planner.maxDeceleration)

            minTimeSolutionProfile[0][3] = traj[-1].y

            # path, speed, acceleration, time
            for i in range(1, len(minTimeSolutionProfile)):
                state = minTimeSolutionProfile[i]
                prevState = minTimeSolutionProfile[i - 1]

                if prevState[2] == 0.0:
                    t = prevState[3] + (state[0] - prevState[0]) / prevState[1]
                else:
                    t = prevState[3] + (state[1] - prevState[1]) / prevState[2]

                minTimeSolutionProfile[i][3] = t

            planner.PlotPathTimePlan(minTimeSolutionProfile, color='-m')

            x = [traj[i].x for i in range(len(traj))]
            y = [traj[i].y for i in range(len(traj))]
            yaw = [traj[i].yaw for i in range(len(traj))]
            k = [traj[i].k for i in range(len(traj))]
            # for i in range(len(x)):
            #     trajectoryGenerator.plot_arrow(x[i], y[i], yaw[i], length=0.1)

            target = trajectoryGenerator.motion_model.State(val[0], val[1], traj[-1], 0.0, 0.)
            plt.plot(x, y, "-r")
            # plt.axis("equal")
            plt.grid(True)
            # trajectoryGenerator.show_trajectory(target, x, y)

    # 1.5, 2.0, 30, 50)
    polygons = [np.array([[s1, t1],[s2, t1],[s2,t2],[s1,t2]])]

    patches = []
    for polygon in polygons:
        patches.append(Polygon(polygon, True, fill=True))

    alpha = 0.7
    color = [0, 1, 0]
    p = PatchCollection(patches, alpha=alpha, match_original=True)
    p.set_color(color)
    ax.add_collection(p)
    plt.xlabel('distance (m)')
    plt.ylabel('time (s)')
    plt.xlim(0, 50)
    plt.ylim(0, 8)
    plt.show()
