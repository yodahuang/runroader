## Run Roader

A speed profile planner for autonomous vehicles with convex optimization

Actually it's non-convex.

### Setup and run

```shell
pip install -e .
```

For running the project, three command line entries are provided: `create-runroader-env`, `optimize-runroader` and `visualize-runroader-env`. Use `--help` for details.

### Mathematical foundation

Please render the following Latex by human compiler.

Let us reparametrize our trajectory $r$ with $s$ which is arc length. Let $s = f(t)$.

We want to optimize on 1D speed profile, which is basically speed and acceleration. Let $\alpha(a) = \ddot{f}$, $\beta(s) = \dot{f}^2$. Here the derivative are both with respect to time.

We want to minimize the time to get to the goal:

$$
J_T = T = \int^T_0 \sqrt{\beta(s)} dt = \int^{S_f}_0 \frac{1}{\sqrt{\beta(s)}}ds = 2 \sum^{N-1}_{i=0}\frac{\Delta s}{\sqrt{\beta_i} + \sqrt{\beta_{i+1}}}
$$

The summation part is simply discretation.

We also want to minimize jitter, 

$$
J_S = \int^{S_f}_0 ||\dot{\ddot{s}}||
$$

This is not convex, so we have

$$
J_S = \int^{S_f}_0 ||\dot{\ddot{s}}|| \approx \int^{S_f}_0||\alpha'(s)|| = \sum^{N+1}_{i=0}||\frac{\alpha(s+1)-\alpha(s_i)}{\Delta s}||^2 \Delta s
$$

Now let's look at the constraints.

First we have the dynamic model constraint:

$$
\begin{aligned}
    \beta'(s) &= 2 \alpha(s)\\
    \frac{\beta(s_{i+1}) - \beta(s_i)}{\Delta s} &= 2 \alpha(s_i) \quad \forall i
\end{aligned}
$$


Then we have obstacle constraint: For the obstacle we pick, we have:
$$
T(s_{corner}) = 2 \sum^{\text{index}_{corner}}_{i=0} \frac{\Delta s}{\sqrt{\beta_i} + \sqrt{\beta_{i+1}}} \ge T_{corner}
$$
Note this does not satisfy DCP rules.

We also have friction circle constraints and initial condition:
$$
\begin{aligned}
    \beta(s_i) &\le v_{max}^2 \quad \forall i\\
    \alpha(s_i) &\le a_{max} \quad \forall i \\
    \beta(s_0) &= v_0^2\\
    \alpha(s_0) &= a_0
\end{aligned}
$$

To summarize, let's put them all together:

$$
\min J_T + J_S = 2 \sum^{N-1}_{i=0}\frac{\Delta s}{\sqrt{\beta_i} + \sqrt{\beta_{i+1}}} + \sum^{N+1}_{i=0}||\frac{\alpha(s+1)-\alpha(s_i)}{\Delta s}||^2 \Delta s\\
\begin{aligned}
    s.t. & \quad \frac{\beta(s_{i+1}) - \beta(s_i)}{\Delta s} = 2 \alpha(s_i) \quad \forall i\\
    & 2 \sum^{\text{index}_{corner}}_{i=0} \frac{\Delta s}{\sqrt{\beta_i} + \sqrt{\beta_{i+1}}} \ge T_{corner}\\
    &\beta(s_i) \le v_{max}^2 \quad \forall i\\
    &\alpha(s_i) \le a_{max} \quad \forall i \\
    &\beta(s_0) = v_0^2\\
    &\alpha(s_0) = a_0

\end{aligned}
$$
