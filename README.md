# Kinematics Project

In this university project we designed, manufactured and program a prototype 4 DOF robotic arm with the task to scoop materials. The arm composed of Dynamixel
MX-28AR motors, laser cut parts and a 3D-printed end-effector. The scope of the project was about 2 weeks with details given in the [report](docs/report.pdf).

<p align="center">
  <img src="demos/gif_bot.gif" />
</p>

## Repository Overview
The repository is divided into the following sections:
1. `cad` - robotic arm and part designs
2. `kinematics` - simulations of robotic arm, including forward and inverse kinematics
3. `electronics` - controller logic and trajectory execution for scooping tasks.
4. `demos` - example videos of the arm performing the task.

## Installation

1. Install Matlab R2021b or newer.
2. Follow the instructions in [MX-28AR_instructions.pdf](docs/MX-28AR_instructions.pdf) to configure the motors and controller.

## Running
1. To control the arm and run a trajectory use `electronics/main.m`.
2. To run the simulation run `kinematics/main.m`.
