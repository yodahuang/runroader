## Run Roader

A speed profile planner for autonomous vehicles with convex optimization

Actually it's non-convex.

### Setup and run

```shell
pip install -e .
```

For running the project, two command line entries are provided: `create-runroader-env` and `optimize-runroader`. Use `--help` for details.

### Mathematical foundation

Please render the following Latex by human compiler.

Let us reparametrize our trajectory $r$ with $s$ which is arc length. Let $s = f(t)$.

We want to optimize on 1D speed profile, which is basically speed and acceleration. Let $\alpha(a) = \ddot{f}$, $\beta(s) = \dot{f}^2$. Here the derivative are both with respect to time.

We want to minimize the time to get to the goal:

$$
J_T = T = \int^T_0 \sqrt{\beta(s)} dt = \int^{S_f}_0 \frac{1}{\sqrt{\beta(s)}}ds = 2 \sum^{N-1}_{i=0}\frac{\delta s}{\sqrt{\beta_i} + \sqrt{\beta_{i+1}}}
$$

The summation part is simply discretation.

We also want to minimize jitter, 

TODO: finish this README