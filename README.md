# Search-Detect-Land

## Status

- Adjusting Ardupilot for flight/combatting yaw offset resulting in spinning. 
- Developing power solution for Raspberry Pi 5 companion computer, independent vs dependent power source (leaning independent)

## Overview

An old racing quad from middle school converted into an autonomous landing platform, utilizing a Raspberry Pi 5 companion computer to detect AprilTag markers via onboard camera, calculating relative pose, and commanding ArduPilot through MAVLink2 to descend onto target. 

I plan to eventually migrate computer vision to a Jetson Orin Nano with CUDA acceleration to efficiently grow from a landing target system to a moving target tracking system. The project is both for fun and to help me get hands on experience with hardware-software integration, computer vision, real time control, and embedded systems before college, in a real world environment (an isolated parking-lot)

## Architecture

*Diagram coming tomorrow*

## Hardware

| Component | Part |
|---|---|
| Frame | Mark4 7" |
| Flight Controller | GEPRC Taker H743 (running ArduPilot) |
| ESC | GEPRC Taker PDB, 4-in-1, 60A |
| Motors | 4× 1300KV |
| Battery | 4S LiPo, 14.8V |
| Companion Computer | Raspberry Pi 5, 8GB RAM |
| Camera | Raspberry Pi Camera 3 NoIR (MIPI CSI) |
| Companion Power | Geekworm X1200 UPS HAT, 2× 18650 Li-ion |


### Power System Notes

Initial attempt: power Pi 5 from main 4S battery via buck converter tapping power from ESC's battery rail/steps down to 5V 5A, capacitor, fuse, and USB PD controller. Output was electrically noisy and unreliable under high throttles.

Switched to Geekworm X1200 UPS HAT with dedicated 2× 18650 cells. Independent power means motor noise can't affect the Pi, and the UPS gives clean shutdown behavior. Though it adds weight, the tradeoff was worth it for stability/not breaking $100 worth of tech.

### Flight Stack Migration

Initial bring-up was on Betaflight to validate frame, motors, and basic flight 
mechanics. Once manual flight was stable, project moved to ArduPilot for the 
autonomous work. Betaflight has limited support for autonomous flight modes and external position commands. However, ArduPilot is built for it.

## Software Stack

- ArduPilot (flight controller firmware)
- Python on RPi for perception + MAVLink bridge
- pymavlink for FC communication
- AprilTag detection library

## Repo Structure

- `src/` — Python source code
- `resources/calibration/` — camera calibration files

## Milestones

- [x] Stable manual flight on Betaflight (initial bring-up)
- [x] AprilTag detection running on Pi 5 standalone
- [x] MAVLink2 over UART6, telemetry working
- [x] CV-triggered arming on the bench
- [x] Position derived from tag pose
- [ ] ArduPilot migration
- [ ] Position commands quad from tag pose
- [ ] Autonomous landing on stationary tag
- [ ] Jetson migration + CUDA acceleration
- [ ] Moving target / chase