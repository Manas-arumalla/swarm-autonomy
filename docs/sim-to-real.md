# Sim-to-Real Path

Swarm Autonomy is simulation-grade by scope. This document outlines the hardware path: what would
transfer, what would need rework, and the cost and timeline to build it.

## Target platform

- **Airframe:** 250–350 mm quad, PX4 (Pixhawk 6C / 6X).
- **Companion computer:** NVIDIA Jetson Orin Nano (8–16 GB) running ROS 2 Jazzy.
- **Sensors:** global-shutter mono/stereo camera (e.g. OAK-D or an Arducam global-shutter module)
  plus an IMU (ICM-42688 class), time-synchronized to the camera for VIO.
- **Radio:** ESP-NOW / 2.4 GHz mesh or 900 MHz LoRa for the bandwidth-limited link.

## What transfers

- The decentralized architecture (per-drone VIO → mapping → planning → control).
- CBBA role allocation and pursuit geometry — pure algorithms, hardware-agnostic.
- The comms middleware's interface; real radios replace the simulated gating policy.

## What needs rework

- **Camera noise and rolling shutter.** Synthetic imagery is too clean; OpenVINS needs re-tuning on
  real sensor noise, exposure, and motion blur.
- **IMU bias and temperature drift.** Real bias instability versus the simulator's near-ideal IMU.
- **Communication latency and jitter.** The model captures range, rate, and dropout but not real PHY
  latency.
- **Compute budget.** The Orin Nano is well below a desktop GPU, so nvblox voxel resolution and the
  planner replan rate must be reduced — the CPU mapping/planning path is directly relevant here.
- **Battery, thermal, and safety.** Flight time, geofencing, failsafes, and netting for pursuit.

## Reality-gap mitigations

Domain randomization of camera/IMU noise in simulation; hardware-in-the-loop with one real drone and
N simulated; conservative planner margins; log-and-replay datasets for VIO tuning.

## Rough cost and timeline

- Bill of materials: ~$1.5–2.5k per drone (airframe + Orin + sensors + radio); three drones ≈ $5–7k.
- Timeline: ~6–10 weeks for single-drone bring-up, plus ~4–6 weeks per added multi-drone behaviour.
