# Autonomous Vehicle Motion Control  & Path Planning (WIP)

A modular, high-fidelity Python framework for simulating and benchmarking classic autonomous vehicle lateral and longitudinal motion controllers side by side in real-time.

##  Features
* **utils**         :  Contains different paths to test the algorithms.
* **Controllers**   :  Pure pursuit with Dynamic Lookahead, Stanley, LQR and MPC using **cvxpy** library.
* **Visualization** :  Vehicle plotter and animation files.
* **Vehicle**       :  Vehicle configuration and state.
---
##  Repository Architecture

```text
├── Controllers/
│   ├── pure_pursuit_controller.py  # Geometric path tracker
│   ├── Stanley_controller.py       # Front-axle error control logic
│   ├── MPC_controller.py           # Model Predictive Control optimizer
│   ├── LQR_controller.py           # Linear Quadratic Regulator tracker
│   └── pid_speed_controller.py     # Longitudinal velocity control loop
├── Vehicle/
│   ├── Vehicle_config.py           # Wheelbase, limits, and vehicle specs
│   ├── vehicle_state.py            # Kinematic state models (x, y, yaw, v)
|   ├── controlInput.py             # acceleration and steering control
│   └── vehicle.py                  # Plant vehicle model and step physics
├── utils/
│   ├── path_generation.py          # Path boundary calculations
|   ├── reference_path.py           # Reference data of paths                 
│   └── geometry.py                 # Mathematical transformations
├── Visualization/
    ├── VehiclePlotter.py           # Underlying canvas rendering pipeline
    ├── PurePursuit.py              # Standalone PP routine / data packager
    ├── Stanley.py                  # Standalone Stanley routine / data packager
    ├── MPC.py                      # Standalone MPC routine / data packager
    ├── LQR.py                      # Standalone LQR routine / data packager
    └── AllinOne.py         # 2x2 Master Subplot Runner & GIF Exporter
```
![Alternative text](Images/vehicle_controllers_comparison.gif)
<table>
  <tr>
    <td><img src="Images/AStar_stanley.gif" alt="AStar Stanley" width="100%"></td>
    <td><img src="Images/Dijkstra_stanley.gif" alt="Dijkstra Stanley" width="100%"></td>
  </tr>
  <tr>
    <td><img src="Images/Hybrid_AStar.gif" alt="Hybrid AStar" width="100%"></td>
    <td><img src="Images/State_Lattice_Stanley.gif" alt="State Lattice Stanley" width="100%"></td>
  </tr>
</table>

##  Setup & Installation

### Prerequisites
Ensure your local system environment is running **Python 3.10+**.

### Dependency Installation
Clone the repository and install the standard dependencies along with the `pillow` engine for GIF rendering:

```bash
git clone https://github.com
cd YOUR_REPO_NAME
pip install numpy matplotlib pillow
```

---

##  Usage Guide

### 1. Isolated Algorithm Testing
Each control script contains an isolated local loop. You can execute any tracker completely independently to observe specific cross-track errors and tracking performance:

```bash
python PurePursuit.py
```

### 2. Multi-Controller Subplot Benchmarking
To run the full 2x2 multi-vehicle matrix, process the background simulations simultaneously, and automatically export your animated comparison chart, launch the master script:

```bash
python AllinOne.py
```

---

##  Core Mechanics & API Pipeline

Every algorithm implementation cleanly exposes an exportable `animation_data` dictionary structure. This eliminates code pollution and guarantees structural alignment when feeding metrics into the `VehiclePlotter` animation engine:

```python
animation_data = {
    "plotter": plotter,
    "path": path,
    "search_windows": search_windows_history,
    "closest_points": closest_points_history,
    "target_points": target_points,
    "states": state_snapshots,
    "history": history_snapshots,
    "title": "Controller Name"
}
```

---
