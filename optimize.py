import cvxpy as cvx
from cvxpy import sqrt, sum, inv_pos
import click
from constants import W_T, W_J, V_0, A_0, V_MAX, A_MAX, S_0, S_F
from create_environment import load_environment, Rectangle
import math


class FancyModel():
    def __init__(self, obstacle, resolution=200):
        self.w_t, self.w_j = (cvx.Parameter(nonneg=True) for _ in range(2))
        self.w_t.value = W_T
        self.w_j.value = W_J

        self.alpha = cvx.Variable(resolution + 1)
        self.beta = cvx.Variable(resolution + 1)

        delta_s = (S_F - S_0) / resolution
        self.J_t = 2 * sum(delta_s * inv_pos(sqrt(self.beta)[:-1] + sqrt(self.beta)[1:]))
        self.J_s = sum(((self.alpha[1:] - self.alpha[:-1]) / delta_s)**2 * delta_s)

        t_corner = obstacle.bottom_left_corner[1] + obstacle.height
        s_corner = obstacle.bottom_left_corner[0]
        index_s = int(math.ceil((s_corner - S_0) / delta_s))
        self.obstacle_constraint = (2 * sum(delta_s * inv_pos(sqrt(self.beta)[:(index_s - 1)] + sqrt(self.beta)[1:index_s])) == t_corner)

        self.friction_circle_constraints = [self.beta <= (V_MAX ** 2), self.alpha <= (A_MAX ** 2)]
        self.initial_condition = [self.beta[0] == (V_0**2), self.beta[1] == A_0]

    def optimize(self):
        obj = cvx.Minimize(self.w_t * self.J_t + self.w_j * self.J_s)
        constraints = [self.obstacle_constraint] + self.friction_circle_constraints + self.initial_condition
        prob = cvx.Problem(obj, constraints)
        prob.solve()
        print("status:", prob.status)
        print("optimal value", prob.value)
        print("optimal var", self.alpha.value, self.beta.value)
    

@click.command()
@click.option('--name', help='The name of the environment, nor providing means use temp environment')
def optimize(name):
    environment = load_environment(name)
    # Just for testing
    model = FancyModel(environment[0])
    model.optimize()


if __name__ == "__main__":
    optimize()
