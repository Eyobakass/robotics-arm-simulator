# Screenshots & Demo Media

Add simulation screenshots and GIFs here to demonstrate the running simulator.

## Recommended captures

| Filename | Content |
|---|---|
| `demo_wave.gif` | Animated GIF of the arm doing a sinusoidal wave sweep (30–60 s) |
| `demo_sweep.gif` | Animated GIF of sequential joint sweeps (each joint min → max) |
| `screenshot_default.png` | Static screenshot of the default pose at startup |
| `screenshot_rotated.png` | Screenshot with joints at non-zero angles |

## How to capture a GIF on Windows

1. Start the simulator: `python src/main.py`
2. Start the sender: `python sender/python_sender.py --mode wave`
3. Use **ScreenToGif** (free, open source) to record the browser window
4. Export as GIF and save to this folder

## How to take a screenshot

Open the VPython browser scene and press `Print Screen`, or use the Windows
Snipping Tool (Win + Shift + S) to capture the 3D viewport.

Save as `screenshots/screenshot_default.png`.
