import math

import click
import cvxpy as cvx
import dccp
import numpy as np
from cvxpy import inv_pos, sqrt, sum

from .constants import A_0, A_MAX, S_0, S_F, V_0, V_MAX, W_J, W_T
from .environment import Environment


class FancyModel():
    def __init__(self, obstacle, resolution=200):
        self.w_t, self.w_j = (cvx.Parameter(nonneg=True) for _ in range(2))
        self.w_t.value = W_T
        self.w_j.value = W_J

        self.alpha = cvx.Variable(resolution + 1)
        self.beta = cvx.Variable(resolution + 1)

        delta_s = (S_F - S_0) / resolution
        self.delta_s = delta_s
        self.J_t = 2 * sum(delta_s * inv_pos(sqrt(self.beta)[:-1] + sqrt(self.beta)[1:]))
        self.J_s = sum(((self.alpha[1:] - self.alpha[:-1]) / delta_s)**2 * delta_s)

        self.dynamic_model = [(self.beta[1:] - self.beta[:-1]) / delta_s == 2 * self.alpha[-1]]

        t_corner = obstacle.bottom_left_corner[1] + obstacle.height
        s_corner = obstacle.bottom_left_corner[0]
        index_s = int(math.ceil((s_corner - S_0) / delta_s))
        self.obstacle_constraint = (2 * sum(delta_s * inv_pos(sqrt(self.beta)[:(index_s - 1)] + sqrt(self.beta)[1:index_s])) >= t_corner)

        self.friction_circle_constraints = [self.beta <= (V_MAX ** 2), self.alpha <= A_MAX]
        self.initial_condition = [self.beta[0] == (V_0**2), self.alpha[0] == A_0]

    def to_t_space(self):
        delta_t = 2 * self.delta_s / (np.sqrt(self.beta.value)[:-1] + np.sqrt(self.beta.value)[1:])
        return np.cumsum(delta_t)

    def optimize(self):
        obj = cvx.Minimize(self.w_t * self.J_t + self.w_j * self.J_s)
        constraints = self.dynamic_model + [self.obstacle_constraint] + self.friction_circle_constraints + self.initial_condition
        prob = cvx.Problem(obj, constraints)
        print("problem is DCCP:", dccp.is_dccp(prob))  # true
        result = prob.solve(method='dccp')
        print("cost value =", result[0])
        # print("optimal var", self.alpha.value, self.beta.value)
        t_space = self.to_t_space()


@click.command()
@click.option('--name', help='The name of the environment, nor providing means use temp environment')
def optimize(name):
    environment = Environment.from_pickle(name)
    # Just for testing
    model = FancyModel(environment.rectangles[0])
    model.optimize()


if __name__ == "__main__":
    optimize()
