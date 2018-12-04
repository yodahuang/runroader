import math
import numpy as np

# motion parameter
L = 2.7  # wheel base
ds = 0.5  # course distanse
v = 50 / 3.6  # velocity [m/s]
nIntervals=100

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, s=0.0, k=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.s = s
        self.k = k

        # These are assigned later
        self.v = 0.0
        self.t = 0.0
        self.a = 0.0


def pi_2_pi(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def update(initial_state,curvature,poly, s, ds ):

    # acc = curvature
    # yaw = yaw + ds*curvature
    x = initial_state.x + math.cos(initial_state.yaw) * ds
    y = initial_state.y + math.sin(initial_state.yaw) * ds
    yaw = initial_state.yaw + poly[0]*s + poly[1]*(s**2)/2.0 + poly[2]*(s**3)/3 + poly[3]*(s**4)/4
    k = curvature

    newState = State(x,y,yaw,s,k)





    return newState


def generate_trajectory(params, initial_state):

    sArr = np.linspace(0.0, params[0], num=nIntervals)
    (a,b,c,d)= GetPolynomialCoefficients(0.0, params[0], params[1], params[2], params[3])
    kArr = [(a+b*s + c*s**2 + d*s**3) for s in sArr]

    sArr = np.array(sArr)
    kArr = np.array(kArr)
    poly = (a, b, c, d)

    # list of states
    traj = []

    traj.append(initial_state)
    newState = initial_state
    for i in range(1,kArr.shape[0]):

        newState = update(newState, kArr[i], poly, sArr[i], sArr[i]-sArr[i-1])
        traj.append(newState)

    return traj


def GetPolynomialCoefficients(p0, sf, p1,p2,p3):
    a = p0
    b = -(11*p0 - 18*p1 + 9*p2 -2*p3)/(2*sf)
    c = 9*(2*p0 - 5*p1 + 4*p2 -p3)/(2*sf**2)
    d = -9*(p0-3*p1 + 3*p2 - p3)/(2*sf**3)

    return (a,b,c,d)

