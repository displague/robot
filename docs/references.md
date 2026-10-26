# References (external projects to borrow test scenarios and tooling from)

Added at P-008. One line each on what the project is and what we would take from it. Summaries
are from the project pages on 2026-10-25; check the repositories before relying on details.

| Project | What it is | What we take from it |
| --- | --- | --- |
| [AlessioPagliai/primo](https://github.com/AlessioPagliai/primo) | Open-source 1.40 m humanoid, still in design: 3D-printed structure, 30 RobStride quasi-direct-drive actuators, Jetson AGX Thor, ~45 kg estimated. CC0 except the Isaac Lab task code (BSD-3). Walking policy trained in Isaac Lab reaches ~87 % survival on rough terrain with stairs, no pushes or domain randomization yet. | The Isaac Lab locomotion task layout (observations, rewards, terrain curriculum) as a template for our Phase 6 training scenarios; a realistic mass/actuator comparison point for a printed humanoid at the other end of the cost scale. |
| [Rhoban/microban](https://github.com/Rhoban/microban) | ~30 cm 3D-printable humanoid from Rhoban (LaBRI, Bordeaux), ~$550-600, 19 Dynamixel XL330 servos, Raspberry Pi Zero 2W with Pollen robot hat, two 18650 cells. Software GPL v3, hardware CC BY-NC-SA 4.0. Has taken first steps; keyboard/gamepad control. | A cheap, printed, walking biped from a RoboCup team: their foot design, walk engine conventions and controller structure are the nearest proven relatives of our Phase 6a static walk, at one fifth the scale. |
| [robodienst.com](https://robodienst.com/) | Presents itself as free tools for humanoid robots. The page content was not readable by the fetcher used here; revisit in a browser. | To be determined after a manual look. |
| [NVIDIA/DreamDojo](https://github.com/NVIDIA/DreamDojo) | NVIDIA's open-source interactive world model trained on 44k hours of egocentric human video, post-trained for robots; 2B and 14B checkpoints, GR-1 teleoperation datasets, ~10 FPS long-horizon generation after distillation. Apache-2.0. | Much later: a video-based world model for manipulation policy evaluation once the upper body does teleoperated tasks (Phase 3-4). Not a locomotion tool. |
| [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) | NVIDIA's GPU-parallel robot learning framework on Isaac Sim: articulated robot assets from URDF/USD, locomotion and manipulation environments, RL training with domain randomization. | The simulation target for Phase 6b dynamic walking training. Needs an NVIDIA GPU; our URDF (generated from `cad/skeleton.json`) is the import path. MuJoCo stays the lightweight local check. |

## How these fit the roadmap

| Phase | Simulator / scenario source |
| --- | --- |
| Now (kinematics, CoM, reach) | Blender rig from `cad/skeleton.json`; URDF export for a quick MuJoCo load |
| 5 (stand, balance) | MuJoCo: push tests on the URDF with tendon-stiffness joints; microban's foot and stance conventions |
| 6a (static walk) | MuJoCo ZMP walk; microban walk engine as a reference |
| 6b (dynamic walk) | Isaac Lab, task layout borrowed from primo; domain randomization over tendon stiffness, drag thresholds and foot friction, which primo's README notes it lacks |
| 3-4 (manipulation) | DreamDojo-style world models only if teleoperation data accumulates |
