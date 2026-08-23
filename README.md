# MuSHR Autonomous Racing 

This repository contains my implementation of the CSE 478 (Autonomous Robotics) course project, built on the [MuSHR](https://mushr.io/) (Multi-agent System for non-Holonomic Racing) platform. I implemented a full autonomy stack for a 1/10-scale car in ROS: **particle-filter localization**, **feedback control (PID / Pure Pursuit / MPC)**, and **sampling-based motion planning (Lazy A\*)**, integrated end-to-end so the car can localize, track a path, and plan around obstacles in simulation.

---

## Table of Contents

1. [System Overview] (#System-Overview)
2. [Particle Filter Localization](#project-1--particle-filter-localization)
3. [Feedback Control](#project-2--feedback-control)
4. [Sampling-Based Motion Planning](#project-3--sampling-based-motion-planning)
6. [Full Pipeline Demo](#full-pipeline-demo)
7. [Repository Structure](#repository-structure)
8. [Setup & Running](#setup--running)
9. [Acknowledgements](#acknowledgements)

---

## System Overview

The MuSHR car is modeled as a **kinematic bicycle (rear-axle) car**, with state and control defined as:

$$
x = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}, \qquad
u = \begin{bmatrix} v \\ \delta \end{bmatrix}
$$

where $(x, y)$ is the 2D position, $\theta$ the heading, $v$ the commanded speed, and $\delta$ the steering angle. The autonomy stack is a standard sense → localize → plan → control loop:

```
LIDAR + Odometry ──▶ Particle Filter (Project 2) ──▶ State Estimate x̂
                                                          │
Map ──▶ Roadmap + Lazy A* (Project 4) ──▶ Reference Path │
                                                          ▼
                                    Controller (Project 3: PID / Pure Pursuit / MPC)
                                                          │
                                                          ▼
                                                   Control command (v, δ)
```

All four projects share this same kinematic car model and the same map-based simulator, so the components built in earlier projects (e.g., the motion model from Project 2) are reused directly in later ones (e.g., MPC rollouts in Project 3, roadmap edge validation in Project 4).

---

## Project 1 — Particle Filter Localization

**Goal:** Estimate the car's pose $x_t = (x, y, \theta)$ from noisy odometry and LIDAR, using a particle filter (sequential Monte Carlo / Bayes filter).

### State Estimation Setup

- **Motion model:** $p(x_t \mid u_t, x_{t-1})$ — probability of the new state given the applied control.
- **Sensor model:** $p(z_t \mid x_t, m)$ — probability of a LIDAR scan given the state and known map $m$.
- **Belief:** approximated by $M$ weighted particles $\{x_t^{[i]}, w_t^{[i]}\}_{i=1}^{M}$.

### Q1 — Kinematic Car Motion Model

**Deterministic update.** Integrating the kinematic bicycle equations over $\Delta t$ with wheelbase $L$:

$$
\theta' = \theta + \frac{v}{L}\tan(\delta)\,\Delta t
$$

If $\delta \neq 0$:

$$
x' = x + \frac{L}{\tan(\delta)}\big(\sin\theta' - \sin\theta\big), \qquad
y' = y - \frac{L}{\tan(\delta)}\big(\cos\theta' - \cos\theta\big)
$$

If $\delta = 0$ (degenerate case, straight-line motion):

$$
x' = x + v\cos(\theta)\,\Delta t, \qquad y' = y + v\sin(\theta)\,\Delta t
$$

**Probabilistic (noisy) update.** Three-step noise injection with action noise $(\sigma_v, \sigma_\delta)$ and model noise $(\sigma_x, \sigma_y, \sigma_\theta)$:

1. Sample noisy controls: $\tilde v \sim \mathcal N(v, \sigma_v^2)$, $\tilde\delta \sim \mathcal N(\delta, \sigma_\delta^2)$
2. Integrate the deterministic model with $(\tilde v, \tilde\delta)$ to get $(x', y', \theta')$
3. Add model noise: $x'' = x' + \varepsilon_x,\; y'' = y' + \varepsilon_y,\; \theta'' = \theta' + \varepsilon_\theta$, with $\varepsilon_x \sim \mathcal N(0,\sigma_x^2)$, etc.

The heading is finally wrapped into $[-\pi, \pi)$:

$$
\theta_{\text{wrapped}} = \big((\theta'' + \pi) \bmod 2\pi\big) - \pi
$$

The whole update is vectorized over all $M$ particles simultaneously using NumPy broadcasting/boolean indexing rather than a Python loop.

**Tuning — motion model noise plots** (target: match the staff-tuned "banana"-shaped noise cone for control $(v, \delta, \Delta t) = (3.0,\, 0.4,\, 0.5)$):

| Iteration 1 | Iteration 2 | Final tuned |
|---|---|---|
| ![mm1](plots/mm1.png) | ![mm2](plots/mm2.png) | ![mm3](plots/mm3.png) |

### Q2 — LIDAR Sensor (Beam) Model

For a single beam, let $z$ be the real measured range and $z^\*$ the simulated ("expected") range from ray-casting on the map. The sensor model is a **4-component mixture**:

$$
p(z \mid z^\*) = z_{\text{hit}}\, p_{\text{hit}}(z\mid z^\*) + z_{\text{short}}\, p_{\text{short}}(z\mid z^\*) + z_{\text{max}}\, p_{\text{max}}(z) + z_{\text{rand}}\, p_{\text{rand}}(z)
$$

with mixture weights $z_{\text{hit}} + z_{\text{short}} + z_{\text{max}} + z_{\text{rand}} = 1$, and:

- **Hit** (Gaussian around the expected return, noise $\sigma_{\text{hit}}$):
$$
p_{\text{hit}}(z\mid z^\*) \propto \exp\!\left(-\frac{(z - z^\*)^2}{2\sigma_{\text{hit}}^2}\right)
$$
- **Short** (unexpected obstacle before the wall — modeled as an exponential decay for $0 \le z \le z^\*$).
- **Max** (point mass at maximum sensor range, for beams that miss everything).
- **Rand** (uniform distribution over all possible ranges, capturing sensor noise/artifacts).

To make this tractable in real time, probabilities are **pre-computed and cached** into a lookup table indexed by (discretized measured range, discretized expected range), normalized so each row sums to 1; at runtime the raycaster (`rangelibc`) produces $z^\*$ and the model becomes an $O(1)$ table lookup per beam.

**Tuning — sensor model likelihood field**, evaluated over every map cell for a fixed scan at $(x, y, \theta) = (-9.6,\, 0,\, -2.5)$ on `maze_0`:

| Position-only likelihood | Position + orientation likelihood |
|---|---|
| ![sensor likelihood position](plots/sensor_likelihood_position.png) | ![sensor likelihood angle](plots/sensor_likelihood_angle.png) |

### Q3 — Particle Filter: Initialization & Low-Variance Resampling

**Initialization:** particles are sampled from a Gaussian prior around a clicked pose $(x_0, y_0, \theta_0)$ with configurable position/heading variance.

**Low-variance resampling.** Rather than drawing $M$ i.i.d. samples (an $O(M \log M)$ operation with high variance), a single random offset is drawn and $M$ evenly-spaced pointers walk the cumulative weight distribution:

$$
r \sim \mathcal U\!\left[0, \tfrac{1}{M}\right), \qquad
u_i = r + \frac{i-1}{M}, \quad i = 1, \dots, M
$$

For each $u_i$, walk along the cumulative sum of normalized weights $c = \sum w^{[i]}$ until $c \geq u_i$, and select that particle. This is $O(M)$, guarantees that particles proportional to their weight are represented (even under uniform weights, no particle is dropped), and has much lower variance than independent sampling — critical for real-time performance.

**Full particle filter evaluation** — 60-second live drive through the CSE2 building, comparing estimated vs. ground-truth path:

![Particle filter path plot in CSE2](plots/particle_filter_path.png)

*Median position/heading error on `full.bag`: **< 0.1** (all axes).*

---

## Project 3 — Feedback Control

**Goal:** Given the current (estimated) state and a reference path with velocities, compute steering commands to track the path, implementing and comparing three controllers of increasing sophistication.

### Shared machinery

**Reference point selection:** find the closest waypoint on the path to the current state, then walk forward along the path to the first waypoint at least `distance_lookahead` away (restricting to points *after* the closest one, so the car is never driven backwards).

**Position in a rotated frame.** Given the car's position $p = (x, y)$ and a reference pose $(x_r, y_r, \theta_r)$, the position of $p$ expressed in the frame defined by the reference pose is:

$$
\begin{bmatrix} x_e \\ y_e \end{bmatrix}
=
\begin{bmatrix} \cos\theta_r & \sin\theta_r \\ -\sin\theta_r & \cos\theta_r \end{bmatrix}
\begin{bmatrix} x - x_r \\ y - y_r \end{bmatrix}
$$

This is used by both PID (error relative to the *closest* reference point) and Pure Pursuit (error relative to the *lookahead* point).

### Q2 — PID (really PD) Controller

$$
u(t) = -K_p\, e(t) - K_d\, \dot e(t)
$$

where $e(t)$ is the lateral (cross-track) error $y_e$ from the frame transform above, and the derivative is computed analytically rather than via finite differences:

$$
\dot e(t) = -v\sin\big(\theta_e\big)
$$

with $\theta_e$ the heading error. The steering command $\delta = u(t)$; the commanded velocity is copied directly from the reference path.

**Tuning — PD gains** $(K_p, K_d)$ on three reference paths:

| Circle | Left-turn | Wave |
|---|---|---|
| ![pid_circle](plots/pid_circle.png) | ![pid_left](plots/pid_left.png) | ![pid_wave](plots/pid_wave.png) |

### Q3 — Pure Pursuit Controller

Pure Pursuit fits a circular arc from the vehicle's rear axle through a lookahead point at distance $\ell_d$ (the "carrot"), expressed in the vehicle frame as $(x_e, y_e)$. With wheelbase $L$, the required curvature and resulting steering angle are:

$$
\kappa = \frac{2 y_e}{\ell_d^2}, \qquad \delta = \arctan\!\big(L \, \kappa\big) = \arctan\!\left(\frac{2 L y_e}{\ell_d^2}\right)
$$

**Tuning — lookahead distance** $\ell_d$ on the `wave` path:

| Lookahead too small | Well-tuned | Lookahead too large |
|---|---|---|
| ![pp_small](plots/pp_small.png) | ![pp_wave](plots/pp_wave.png) | ![pp_large](plots/pp_large.png) |

A too-small $\ell_d$ causes oscillation/overshoot as the controller chases nearby curvature; a too-large $\ell_d$ cuts corners and undershoots sharp turns.

### Q4 — Model-Predictive Control (MPC)

MPC solves a finite-horizon ($T$ steps), sampling-based ($K$ rollouts) optimization at every control step:

1. **Sample controls** — $K$ candidate steering angles evenly spanning $[\delta_{\min}, \delta_{\max}]$, each held constant for $T-1$ steps, giving a control tensor of shape $(K, T-1, 2)$.
2. **Rollout** — for each of the $K$ sequences, forward-simulate the (Project 2) kinematic car model for $T$ steps from the current state estimate, producing state sequences of shape $(K, T, 3)$.
3. **Cost** — each rollout is scored as a weighted sum of a goal-distance term and a collision-avoidance term:

$$
J_k = w_{\text{error}} \cdot \big\lVert x^{(k)}_T - x_{\text{ref}} \big\rVert + w_{\text{collision}} \cdot \sum_{t=1}^{T} \mathbb{1}\!\left[x^{(k)}_t \in \text{obstacle}\right]
$$

4. **Select** — execute the first control of the lowest-cost rollout $k^\* = \arg\min_k J_k$; replan at the next step (receding horizon).

Because MPC reasons over obstacles that the reference path ignores, it is the only controller of the three that can navigate a slalom of obstacles not accounted for by the planner:

![MPC rollouts colored by cost](plots/mpc_rollouts.png)

**Tuning — MPC on standard paths:**

| Circle | Wave | Saw (hardest — sharp corners exceed max curvature within the horizon) |
|---|---|---|
| ![mpc_circle](plots/mpc_circle.png) | ![mpc_wave](plots/mpc_wave.png) | ![mpc_saw](plots/mpc_saw.png) |

**MPC navigating unmodeled obstacles in `slalom_world`:**

| Slalom 1 | Slalom 2 |
|---|---|
| ![mpc_slalom1](plots/mpc_slalom1.png) | ![mpc_slalom2](plots/mpc_slalom2.png) |

---

## Project 4 — Sampling-Based Motion Planning

**Goal:** Build a probabilistic roadmap (PRM-style graph) over free space, search it efficiently with Lazy A\*, and shortcut the resulting path — then connect the whole stack (localization → planning → control) into one closed loop.

### Q1 — Roadmap Construction

**Halton sequence sampling.** A deterministic, low-discrepancy quasi-random sequence used instead of uniform random sampling for better coverage with fewer samples. For index $i$ and prime base $b$, the radical-inverse function is:

$$
\phi_b(i) = \sum_{k=0}^{n} \frac{d_k}{b^{k+1}}, \qquad \text{where } i = \sum_{k=0}^{n} d_k\, b^k \; (0 \le d_k < b)
$$

i.e., reverse the base-$b$ digits of $i$ around the "decimal" point. A separate Halton generator (distinct prime base) is used per configuration-space dimension, and samples are scaled into the extents of the problem.

![Halton samples, free space](plots/halton_samples.png)
![Halton samples on map1.txt (collision-free)](plots/halton_samples_map1.png)

**Collision checking.** For each sampled state, validity requires the state to lie within the problem extents and its $(x,y)$ position to fall in the permissible (free) region of the map. Edges between nearby vertices (within a connection radius $r$) are similarly checked by discretizing the edge and validating every intermediate state.

![Roadmap graph with edges, map1.txt](plots/roadmap_graph.png)

### Q2 — Graph Search: A\*, Lazy A\*, and Shortcutting

**A\*** expands nodes in order of $f(n) = g(n) + h(n)$, where $g(n)$ is the cost-to-come (accumulated edge length) and $h(n)$ is an admissible heuristic estimate of cost-to-go (Euclidean distance for R2, a relaxed Dubins-length bound for SE2).

$$
f(n) = g(n) + h(n)
$$

![A* shortest path, map1.txt](plots/astar_map1.png)

**Lazy A\*.** Edge collision-checking dominates roadmap construction cost. Lazy A\* defers this check: edges are optimistically assumed valid during search and are only actually collision-checked when a node is *expanded* (i.e., when the search commits to using that edge). If the edge turns out to be invalid, the corresponding queue entry is discarded and search continues — trading some re-expansion for a much cheaper roadmap-construction phase.

![Lazy A* path, map1.txt (dashed = invalid edges skipped)](plots/lazy_astar_map1.png)

**Shortcutting.** A\* returns the shortest path *on the graph*, not necessarily the shortest path in continuous space (since only sampled vertices are connectable). The shortcut post-processor repeatedly picks two random indices along the path, and if a direct, collision-free edge between them is both feasible and shorter than the existing sub-path, replaces the sub-path with that direct edge.

![Shortcut path vs. original Lazy A* path, map1.txt](plots/shortcut_map1.png)

**Radius / vertex-count sensitivity on `map2.txt`** *(600 vertices baseline, $r=100$)*:

| Connection radius $r$ | Path length | Planning time |
|---|---|---|
| *(fill in from experiments)* | | |

| Num. vertices $n$ | Path length | Planning time |
|---|---|---|
| *(fill in from experiments)* | | |

### Q3 — SE(2) Planning with Dubins Paths

For the car (which has orientation and a minimum turning radius), edges connect configurations via **Dubins paths** — the shortest path between two oriented points subject to a maximum curvature (minimum turning radius) constraint:

$$
\kappa_{\max} = \frac{1}{r_{\min}}
$$

Because Dubins paths are direction-dependent (a path from $A \to B$ is generally not the reverse of $B \to A$), the SE(2) roadmap graph is **directed**, and typically needs more vertices than R2 to remain well-connected.

**Maximum curvature for the MuSHR car:** with max steering angle $\delta_{\max} = 0.34$ rad and wheelbase $L = 0.33$ m, from the kinematic car model $\dot\theta = \frac{v}{L}\tan\delta$, the minimum turning radius is $r_{\min} = L / \tan(\delta_{\max})$, giving:

$$
\kappa_{\max} = \frac{\tan(\delta_{\max})}{L} = \frac{\tan(0.34)}{0.33} \approx 1.09~\text{m}^{-1}
$$

**Effect of curvature constraint** on the SE(2) path (fixed $n=40$, $r=4$):

| $c = 3$ | $c = 4.5$ | $c = 9$ | $c = 15$ |
|---|---|---|---|
| ![se2_c3](plots/se2_curvature_3.png) | ![se2_c4.5](plots/se2_curvature_4_5.png) | ![se2_c9](plots/se2_curvature_9.png) | ![se2_c15](plots/se2_curvature_15.png) |

### Q4 — Full Integration

`planner_ros` ties the particle filter's state estimate to a cached roadmap (indexed by map, problem type, sampler, vertex count, connection radius, and curvature), runs Lazy A\* + shortcutting from the estimated current state to a clicked goal, and hands the resulting path to the MPC controller for tracking — closing the full localize → plan → control loop.

**`maze_0`** (roadmap: *fill in num_vertices / connection_radius / curvature used*):

![Planning + tracking in maze_0](plots/planning_maze_0.png)

**`cse2_2`** (larger map — roadmap: *fill in parameters used*):

![Planning + tracking in cse2_2](plots/planning_cse2_2.png)

---

## Full Pipeline Demo

End-to-end videos of the integrated stack (particle filter localization + Lazy A\* planning + MPC tracking) driving the MuSHR car to a goal pose, in simulation and (optionally) on the physical car.

<!-- Replace with actual video embeds/links, e.g.:
https://user-images.githubusercontent.com/.../demo-maze0.mp4
or a GIF: ![demo](videos/demo_maze0.gif)
-->

| Simulation — `maze_0` | Simulation — `cse2_2` |
|---|---|
| *(insert video/GIF here)* | *(insert video/GIF here)* |

| Optional — physical MuSHR car in the lab |
|---|
| *(insert video/GIF here)* |

---

## Repository Structure

```
mushr478/
├── cse478/            # Shared utilities, launch files, maps, RViz configs
├── introduction/       # Project 1 — ROS basics, Fibonacci node, PoseListener
│   ├── src/introduction/{fibonacci.py, listener.py}
│   └── writeup/README.md
├── localization/        # Project 2 — Particle filter
│   ├── src/localization/{motion_model.py, sensor_model.py, particle_filter.py, resampler.py}
│   └── writeup/README.md
├── control/             # Project 3 — PID, Pure Pursuit, MPC
│   ├── src/control/{controller.py, control_ros.py}
│   └── writeup/README.md
├── planning/            # Project 4 — Roadmap, A*/Lazy A*, shortcutting
│   ├── src/planning/{problems.py, samplers.py, roadmap.py, search.py}
│   └── writeup/README.md
└── README.md            # (this file)
```

## Setup & Running

This project targets **ROS Noetic** on the CSE 478 course VM (Ubuntu 20.04). High-level setup:

```bash
# One-time: build dependency workspace
source /opt/ros/noetic/setup.bash
cd ~/dependencies_ws && catkin build
source ~/dependencies_ws/devel/setup.bash

# Build this project workspace
cd ~/mushr_ws && catkin build
source ~/mushr_ws/devel/setup.bash

# Run individual package tests
catkin test introduction
catkin test localization
catkin test control
catkin test planning

# Launch the full integrated stack in simulation
roslaunch planning planner_sim.launch \
  map:='$(find cse478)/maps/maze_0.yaml' \
  num_vertices:=1000 connection_radius:=10 curvature:=1
```

See each subpackage's `writeup/README.md` for detailed per-project instructions, parameter tuning notes, and answers to the course writeup questions.

## Acknowledgements

Built for CSE 478 (Autonomous Robotics), University of Washington, Paul G. Allen School of Computer Science & Engineering, on the open-source [MuSHR](https://mushr.io/) platform. Course materials, starter code, and figures referenced from the course website.
