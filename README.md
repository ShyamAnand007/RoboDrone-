# Drone Flight Training Simulator

Python + [Ursina](https://www.ursinaengine.org/) (Panda3D renderer). Full quadcopter physics, a procedural city, flight rings, precision landing pads, dynamic shadows, day/night mode, HUD and mini-map.

## Features
- 6-axis flight model: throttle, pitch, roll, yaw
- Physics: gravity, thrust along the drone's real up-vector (tilt = horizontal motion), motor lag, battery voltage sag, ground effect, linear + quadratic drag, wind and gusts
- Crash detection: descent > 4.5 m/s, touchdown tilt > 15 deg, skidding > 8 m/s, buildings, trees
- 30 procedural buildings (neon bands, rooftop units, blinking antenna beacons), roads with lane markings, parks, 120+ trees, hills, drifting clouds
- 8 flight rings with combo scoring and time bonus; HOME and PAD B precision-landing pads
- Sun shadows, fog, MSAA, day/night, blob shadow, rotor blur, motion trail, explosion + debris + smoke
- Chase / FPV / orbit cameras, speed-based FOV, crash shake
- HUD: altitude, speed, vertical speed, heading, tilt, throttle + battery bars, score, next-ring guidance, mini-map

## Controls
| Key | Action |
|---|---|
| SPACE / LEFT SHIFT | Throttle up / down (hover is ~50%) |
| W / S (or arrows) | Pitch forward / backward |
| A / D | Roll left / right |
| Q / E | Yaw left / right |
| R | Reset |
| C | Cycle camera |
| Z | Hover assist (holds altitude) |
| T | Wind on/off |
| N | Day / night |
| H | Help overlay |
| F11 | Fullscreen |
| ESC | Quit |

Tip: hold SPACE until you lift off, then feather it. Battery sag means hover throttle creeps upward as the pack drains.

## Run from source
```
pip install -r requirements.txt
python main.py
```
Weak GPU? Set `DRONE_LOW=1` (turns off shadows):  `set DRONE_LOW=1 && python main.py`

## Build the .exe (Windows)
1. Install Python 3.10+ (tick "Add Python to PATH").
2. Double-click `build.bat`.
3. Get `dist\DroneFlightSimulator.exe` - copy it to any Windows 10/11 PC.

## Project structure
```
drone-sim/
├── main.py           # entire simulator: physics, world, HUD, cameras
├── requirements.txt
├── build.bat         # one-click Windows EXE builder
└── README.md
```

## Tuning (top of main.py)
`TILT_MAX`, `K_LIN`, `K_QUAD` (drag), `CRASH_*` thresholds, `RING_DEFS` (course layout), `rng = random.Random(2024)` (change seed for a new city).

## Team ideas (one contribution every 20 min)
- New course layouts in `RING_DEFS` - sounds - gamepad support - replay/ghost - timed race mode - moving obstacles - better HUD horizon - rain/fog weather
