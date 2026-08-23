# Autonomous Driving with MuSHR

This repository contains my implementation of the CSE 478 (Autonomous Robotics) course project, built on the [MuSHR](https://mushr.io/) (Multi-agent System for non-Holonomic Racing) platform. I implemented a full autonomy stack for a 1/10-scale car in ROS: **particle-filter localization**, **feedback control (PID / Pure Pursuit / MPC)**, and **sampling-based motion planning (Lazy A\*)**, integrated end-to-end so the car can localize, track a path, and plan around obstacles in simulation.

---

## Table of Contents

1. [System Overview](#System-Overview)
2. [Sampling-Based Motion Planning](#sampling-based-motion-planning)
3. [Particle Filter Localization](#particle-filter-localization)
4. [Feedback Control](#feedback-control)
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

where $(x, y)$ is the 2D position, $\theta$ the heading, $v$ the commanded speed, and $\delta$ the steering angle.

The car's motion is governed by the continuous-time **rear-axle kinematic bicycle model**, with wheelbase $L$ (distance between front and rear axles):

$$
\dot x = v\cos\theta, \qquad \dot y = v\sin\theta, \qquad \dot\theta = \frac{v}{L}\tan\delta
$$


---

## Sampling-Based Motion Planning

**Goal:** Build a probabilistic roadmap (PRM-style graph) over free space, search it efficiently with Lazy A\*, and shortcut the resulting path — then connect the whole stack (localization → planning → control) into one closed loop.

### Roadmap Construction

**Halton sequence sampling.** A deterministic, low-discrepancy quasi-random sequence used instead of uniform random sampling for better coverage with fewer samples. 

**Collision checking.** For each sampled state, validity requires the state to lie within the problem extents and its $(x,y)$ position to fall in the permissible (free) region of the map. Edges between nearby vertices (within a connection radius $r$) are similarly checked by discretizing the edge and validating every intermediate state.

### Graph Search: A\*, Lazy A\*, and Shortcutting

**A\*** expands nodes in order of $f(n) = g(n) + h(n)$, where $g(n)$ is the cost-to-come (accumulated edge length) and $h(n)$ is an admissible heuristic estimate of cost-to-go (Experimented with Euclidean distance and Dubins-length bound).

**Lazy A\*.** Edge collision-checking dominates roadmap construction cost. Lazy A\* defers this check: edges are optimistically assumed valid during search and are only actually collision-checked when a node is *expanded*.

**Shortcutting.** A\* returns the shortest path *on the graph*, not necessarily the shortest path in continuous space (since only sampled vertices are connectable). The shortcut post-processor repeatedly picks two random indices along the path, and if a direct, collision-free edge between them is both feasible and shorter than the existing sub-path, replaces the sub-path with that direct edge.

---

## Particle Filter Localization

**Goal:** Estimate the car's pose $x_t = (x, y, \theta)$ from noisy movement data and LIDAR scans using a particle filter.

### Particle Filter Initialization
Particles are sampled from a Gaussian prior around current pose $(x_0, y_0, \theta_0)$ with configurable position/heading variance.

### Kinematic Car Motion Model
This step predicts where the car will go based on its speed and steering commands (using a kinematic bicycle model). To account for real-world unpredictability, we intentionally inject random noise to simulate tire slip, delayed controls, and physical inaccuracies.

### LIDAR Sensor (Beam) Model
The LIDAR sensor model grades each particle's accuracy by comparing the car's actual LIDAR scan against a simulation of what that specific particle should see based on the map. Instead of assuming a perfect sensor, it intelligently accounts for four real-world behaviors: accurate map hits, early blocks from unexpected obstacles (like people), max-range beams that hit nothing, and random sensor noise.

### Low-Variance Resampling
To keep the algorithm efficient, we systematically weed out the bad guesses. We delete the low-scoring particles and clone the high-scoring ones, pulling the "cloud" of guesses tighter together around the car's true location.

---

## Feedback Control

**Goal:** Given the current (estimated) state and a reference path with velocities, compute steering commands to track the path.

### Shared machinery

**Reference point selection:** find the closest waypoint on the path to the current state, then walk forward along the path to the first waypoint at least `distance_lookahead` away (restricting to points *after* the closest one, so the car is never driven backwards).

**Position in a rotated frame.** Given the car's position $p = (x, y)$ and a reference pose $(x_r, y_r, \theta_r)$, the position of $p$ expressed in the frame defined by the reference pose — i.e., a translation by $(-x_r, -y_r)$ followed by a rotation by $-\theta_r$ — is:

$$
x_e = (x - x_r)\cos\theta_r + (y - y_r)\sin\theta_r
$$

$$
y_e = -(x - x_r)\sin\theta_r + (y - y_r)\cos\theta_r
$$

This is used by both PID (error relative to the *closest* reference point) and Pure Pursuit (error relative to the *lookahead* point).

### PD Controller
$$
u(t) = -K_p\. e(t) - K_d\. \dot e(t)
$$

where $e(t)$ is the lateral (cross-track) error $y_e$ from the frame transform above, and the derivative is computed analytically rather than via finite differences:

$$
\dot e(t) = -v\sin\big(\theta_e\big)
$$

with $\theta_e$ the heading error. The steering command $\delta = u(t)$; the commanded velocity is copied directly from the reference path.

### Pure Pursuit Controller

Pure Pursuit fits a circular arc from the vehicle's rear axle through a lookahead point at distance $\ell_d$ (the "carrot"), expressed in the vehicle frame as $(x_e, y_e)$. With wheelbase $L$, the required curvature and resulting steering angle are:

$$
\kappa = \frac{2 y_e}{\ell_d^2}, \qquad \delta = \arctan\!\big(L \, \kappa\big) = \arctan\!\left(\frac{2 L y_e}{\ell_d^2}\right)
$$

### Model-Predictive Control (MPC)

MPC solves a finite-horizon ($T$ steps), sampling-based ($K$ rollouts) optimization at every control step:

1. **Sample controls** — $K$ candidate steering angles evenly spanning $[\delta_{\min}, \delta_{\max}]$, each held constant for $T-1$ steps, giving a control tensor of shape $(K, T-1, 2)$.
2. **Rollout** — for each of the $K$ sequences, forward-simulate the (Project 2) kinematic car model for $T$ steps from the current state estimate, producing state sequences of shape $(K, T, 3)$.
3. **Cost** — each rollout is scored as a weighted sum of a goal-distance term and a collision-avoidance term:

$$
J_k = w_{\text{error}} \cdot \big\lVert x^{(k)}_T - x_{\text{ref}} \big\rVert + w_{\text{collision}} \cdot \sum_{t=1}^{T} \mathbb{1}\!\left[x^{(k)}_t \in \text{obstacle}\right]
$$

4. **Select** — execute the first control of the lowest-cost rollout $k^\* = \arg\min_k J_k$; replan at the next step (receding horizon).

Because MPC reasons over obstacles that the reference path ignores, it is the only controller of the three that can navigate a slalom of obstacles not accounted for by the planner:

## Full Pipeline Demo

End-to-end videos of the integrated stack (particle filter localization + Lazy A\* planning + MPC tracking) driving the MuSHR car to a goal pose, in simulation and (optionally) on the physical car.

<!-- Replace with actual video embeds/links, e.g.:
https://user-images.githubusercontent.com/.../demo-maze0.mp4
or a GIF: ![demo](videos/demo_maze0.gif)
-->

| Simulation — `maze_0` | Simulation — `cse2_2` |
|---|---|
| *(insert video/GIF here)* | *(insert video/GIF here)* |

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
