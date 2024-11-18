# Legged Robotics

This repository contains implementations of locomotion algorithms for legged robots, focusing on bipedal and quadrupedal locomotion. The projects explore various techniques such as trajectory planning, Central Pattern Generators (CPGs), and Reinforcement Learning (RL).

---

## Authors
- [Noah Lugon](https://github.com/nlugon)
- [Vincent Gherold](https://github.com/VinceGHER?tab=repositories)
- Théo Pugin-Bron


## Repository Structure

- **`biped-robot/`**: Contains code and resources for bipedal locomotion, focusing on Divergent Component of Motion (DCM)-based planning and control.
- **`quadruped-robot/`**: Contains code and resources for quadrupedal locomotion, utilizing Central Pattern Generators (CPGs) and Deep Reinforcement Learning (DRL).

---

## 1. Bipedal Locomotion (`biped-robot/`)

### Overview
This project focuses on planning the Center of Mass (CoM) trajectory for a bipedal robot using the Divergent Component of Motion (DCM) framework. The implementation addresses dynamic balancing and stable walking on flat terrain.

### Key Features
- **DCM Planning**: Trajectory generation for CoM based on the DCM model.
- **Foot Trajectory Planning**: Fixed-step positions for dynamic balance.
- **Inverse Kinematics**: Mapping desired trajectories to joint space.


### Video
![Bipedal Locomotion Video](biped-robot/atlas.gif)

---

## 2. Quadrupedal Locomotion (`quadruped-robot/`)

### Overview
This project implements locomotion controllers for quadrupedal robots using:
1. Central Pattern Generators (CPGs) for various gaits (walk, trot, pace, bound).
2. Deep Reinforcement Learning (DRL) for adaptive locomotion across challenging terrains.

### Key Features
#### Central Pattern Generators (CPG)
- Coupled oscillators for gait generation.
- Cartesian and joint PD control for foot placement.

#### Deep Reinforcement Learning (DRL)
- Markov Decision Process (MDP) design for observation, action, and reward spaces.
- Implementation of PPO and SAC algorithms for policy learning.


### Video
![Quadruped Locomotion Video](quadruped-robot/quadruped.gif)


[On YouTube](https://www.youtube.com/watch?v=mAbwYRhE2rQ)



---

## How to Use

### Prerequisites
- Install [PyBullet](https://pybullet.org/wordpress/) for simulation.
- Install [Stable Baselines3](https://github.com/DLR-RM/stable-baselines3) for reinforcement learning (quadruped).

### Running the Code
1. **Bipedal Locomotion**:
   - Navigate to the `biped-robot/` directory.
   - Run `python DCMTrajectoryGenerator.py` for DCM-based CoM trajectory planning.

2. **Quadrupedal Locomotion**:
   - Navigate to the `quadruped-robot/` directory.
   - Use `run_cpg.py` for CPG-based locomotion.
   - Use `run_sb3.py` for training policies with DRL.

