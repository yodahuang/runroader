import numpy as np
import matplotlib.pyplot as plt
import math
import motion_model
import ExportJson
import copy
import os
from collections import defaultdict

# optimization parameter
max_iter = 100
deltaParams = np.array([0.1, 0.02, 0.02, 0.02])

xtol = 0.02
ytol = 0.02
thetaTol = 0.01
kTol = 0.01

learningRate = 0.1

show_animation = False
plot_trajectories = False

speedDiscretization = 3.0
maxSpeed = 21.0
initialSpeed = 3.0
maxCurvatureRate = 0.20
minX = 10.0
maxX = 50.0
laneSize = 4.0


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    u"""
    Plot arrow
    """
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)
    plt.plot(0, 0)


def GetJacobian(target, params, deltaParams, state):

    J=[]

    newParam = params
    newParam[0] = newParam[0] + deltaParams[0]
    traj = motion_model.generate_trajectory(newParam, state)
    dp = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    newParam = params
    newParam[0] = newParam[0] - deltaParams[0]
    traj = motion_model.generate_trajectory(newParam, state)
    dn = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    dp = np.array(dp)
    dn = np.array(dn)
    J1 = (dp-dn)/(2*deltaParams[0])
    J.append(J1)


    newParam = params
    newParam[1] = newParam[1] + deltaParams[1]
    traj = motion_model.generate_trajectory(newParam, state)
    dp = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    newParam = params
    newParam[1] = newParam[1] - deltaParams[1]
    traj = motion_model.generate_trajectory(newParam, state)
    dn = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    dp = np.array(dp)
    dn = np.array(dn)
    J1 = (dp - dn) / (2 * deltaParams[1])
    J.append(J1)



    newParam = params
    newParam[2] = newParam[2] + deltaParams[2]
    traj = motion_model.generate_trajectory(newParam, state)
    dp = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    newParam = params
    newParam[2] = newParam[2] - deltaParams[2]
    traj = motion_model.generate_trajectory(newParam, state)
    dn = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    dp = np.array(dp)
    dn = np.array(dn)
    J1 = (dp - dn) / (2 * deltaParams[2])
    J.append(J1)



    newParam = params
    newParam[3] = newParam[3] + deltaParams[3]
    traj = motion_model.generate_trajectory(newParam, state)
    dp = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    newParam = params
    newParam[3] = newParam[3] - deltaParams[3]
    traj = motion_model.generate_trajectory(newParam, state)
    dn = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    dp = np.array(dp)
    dn = np.array(dn)
    J1 = (dp - dn) / (2 * deltaParams[3])
    J.append(J1)

    J = np.array(J).T

    return J


def show_trajectory(target, xc, yc):

    # plt.clf()
    plot_arrow(target.x, target.y, target.yaw)
    plt.plot(xc, yc, "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.1)


def ReachedTarget(target, lastState):

    if math.fabs(target.x-lastState.x) > xtol:
        return False
    elif math.fabs(target.y-lastState.y) > ytol:
        return False
    elif math.fabs(target.yaw-lastState.yaw) > thetaTol:
        return False
    elif math.fabs(target.k-lastState.k) > kTol:
        return False

    return True


def optimizePath(target, state, params):

    p = copy.deepcopy(params)
    foundPath = False

    for i in range(max_iter):
        traj = motion_model.generate_trajectory(p, state)

        x = [traj[i].x for i in range(len(traj))]
        y = [traj[i].y for i in range(len(traj))]
        theta = [traj[i].yaw for i in range(len(traj))]

        if np.any(np.array(theta) < 0.01) or np.any(np.array(theta) > 5):
            xc, yc, yawc, p = None, None, None, None
            break


        if ReachedTarget(target, traj[-1]):
            # print("found path")
            foundPath = True

            # snap the last configuration to target state, to avoid discont
            # during planning
            traj[-1].x = target.x
            traj[-1].y = target.y
            traj[-1].yaw = target.yaw
            traj[-1].k = target.k

            break
        else:

            error = [(target.x-traj[-1].x),(target.y-traj[-1].y),(target.yaw-traj[-1].yaw),(target.k-traj[-1].k)]
            error = np.array(error)
            error = error.reshape(error.shape[0],1)
            J = GetJacobian(target, params, deltaParams, state)

            try:
                dp = - np.dot(np.linalg.inv(J), error)
            except np.linalg.linalg.LinAlgError:
                print("cannot calc path LinAlgError")
                xc, yc, yawc, p = None, None, None, None
                break
            alpha = learningRate

            p += alpha * dp.reshape(dp.shape[0])

            if show_animation:
                x = [traj[i].x for i in range(len(traj))]
                y = [traj[i].y for i in range(len(traj))]
                theta = [traj[i].yaw for i in range(len(traj))]

                show_trajectory(target, x, y)

    return traj, params, foundPath


# minimize final curvature and maximize final theta
# Max speed: 20m/s
# max acceleration 4.0 m/s, min acceleration -4m/s
# Initial configuration x=0, y=0, theta = speed, curvature = 0 (not accelerating before),
# Final configuration: x=s, y=t, theta = max_speed possible without violating maximum acceleration
def test_optimize_speed_profile(initialSpeed=10., initial_aceleration=0., final_s=30.,
                                final_t=20.,      maxSpeed = 20.,         minSpeed = 1.,
                                max_acc = 4.):

    speeds = np.arange(maxSpeed, minSpeed, -1.)
    curvatures = np.arange(max_acc, -max_acc, -1.)
    # for l in range(curvatures.shape[0]):
    for k in range(speeds.shape[0]):

        initial_state = motion_model.State(x=0., y=0., yaw=1./initialSpeed, s=0.0, k=initial_aceleration)
        target = motion_model.State(x=final_s, y=final_t, yaw=1./speeds[k], s=0.0, k=0.)

        # initialize parameters
        init_p = np.array([final_s/math.cos(1.0/initialSpeed), 0.0, 0.0, 0.0])

        traj, params, foundPath = optimizePath(target,initial_state, init_p)

        if foundPath:
            x = [traj[i].x for i in range(len(traj))]
            y = [traj[i].y for i in range(len(traj))]
            yaw = [traj[i].yaw for i in range(len(traj))]
            curvatures = [traj[i].k for i in range(len(traj))]

            pathValid = True
            for j in range(len(curvatures)):
                if math.fabs(curvatures[j])>max_acc or yaw[j]<(1./maxSpeed) or yaw[j]<0.:
                    pathValid=False
                    break

            if pathValid:
                for i in range(len(x)):
                    plot_arrow(x[i], y[i], yaw[i], length=0.1)

                if plot_trajectories:
                    show_trajectory(target, x, y)
                    plt.axis("equal")
                    plt.grid(True)
                    # plt.show()

                return traj

    return []



def main():
    print(__file__ + " start!!")

    plt.xlabel('distance (m)')
    plt.ylabel('time (s)')
    test_optimize_speed_profile()

    plt.show()









if __name__ == '__main__':
    main()
