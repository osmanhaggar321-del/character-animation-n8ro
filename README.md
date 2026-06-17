# Character Animation — N8RO
**Student ID:** 230208712

## Project Overview
A custom animation plugin for N8RO that controls 10 joints of a human character using kinematic joint angle overrides.

## 10 Joints
LeftArm, RightArm, LeftForeArm, RightForeArm, LeftUpLeg, RightUpLeg, LeftLeg, RightLeg, LeftFoot, RightFoot

## 4 Motion States
1. **Idle Breathing** — Natural standing posture with breathing simulation
2. **Idle Neutral** — Walking gait cycle (Hip ±25°, Knee 0-60°, Ankle ±15°)
3. **Idle Shake** — Deep squat (Hip 60°, Knee 90°, Ankle -20°)
4. **Turning** — Upper/lower body counter-rotation via direct joint control

## Files
- `SimCharAnimCustomModelPlugin.cpp` — Main C++ plugin
- `SimCharAnimCustomModelPlugin.h` — Header file
- `human_animation_loop.lua` — Mission script# character-animation-n8ro
