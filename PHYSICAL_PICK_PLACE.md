# MuJoCo physical pick, carry and place

This task uses the existing B2, dual Aubo arms, Robotiq 2F-85 grippers and expanded factory. The validated task arm is **left**. Right-arm and rotated side-station tasks fail explicitly until they have their own acceptance runs. The leg actor remains experimental; this work does not constitute payload-policy training or hardware calibration.

## Run

```powershell
.\scripts\windows\run_sim.ps1 -Brain -Manual -Background
.\scripts\windows\run_pick_place.ps1
```

For ROS integration checks on a machine using software OpenGL:

```powershell
.\scripts\windows\run_sim.ps1 -Brain -Manual -Background -NoPhysicsGui -NoRviz
.\scripts\windows\run_pick_place.ps1 -FastValidation
```

`FastValidation` temporarily disables image rendering and restores it afterwards. Physics, lidar, Nav2, ownership, manipulation and brain services continue running. This task currently uses simulation truth for object pose and odometry; it is not a demonstration of vision-only grasp acquisition. Ordinary visual servoing still has its separate image-based workflow.

The API is the existing `/brain/request` service, `op=submit`, with the current `scene_version`, a unique `task_id`, and proposal:

```json
{"skill":"pick_place","object_id":"red_cube","target_id":"place_station","arm":"left"}
```

## Execution and ownership

1. The skill server acquires the base lease, docks at the pick station, and waits for measured base stability.
2. `/manipulation/command` acquires exclusive arm/gripper ownership. It rejects simultaneous trajectory and visual-servo control.
3. The physical state machine opens, approaches, aligns with the cube faces, descends, closes, lifts and checks bilateral contact and lift height.
4. The skill retreats from the workstation under bounded low-speed control before transferring authority to Nav2. A larger carrying footprint covers the extended arm and load. The south corridor goes around the rack.
5. The robot stops outside the destination workstation, precisely docks, lowers the load, opens, retreats and verifies the released cube on the destination support.

Only joint motor targets are commanded. The cube remains a free body throughout. There is no object teleportation, weld, virtual attachment or hidden support. IK uses a separate `MjData`; the live object's state is never overwritten.

Cancellation stops new motion commands and retains the closed-gripper target when a load is held. A lost or stale owner, dropped payload, excessive penetration, arm collision, failed IK or failed placement ends the task with an error. Failed carry operations hold the base and retain the gripping target; they do not silently release the object. An existing held payload requires explicit recovery or a fresh simulation episode before another pick task.

## Physics corrections

- The right fingertip and inner linkage follow the right jaw actuator, preserving parallel fingers when contact deflects the jaws differently.
- The `robotiq_contact_v1` profile uses elliptic friction, multiple mesh contacts, and explicit cube contact compliance and torsional/rolling friction. It does not enable the global no-slip postprocessor, which adversely affected locomotion in an experiment.
- Both arms use bounded gravity/bias feedforward alongside the existing PD motors. All original URDF motor limits remain enforced.
- The velocity adapter no longer assumes zero breakaway command when calibration lacks stopped samples for a direction. It uses the lowest measured moving command and retains velocity feedback; exact zero commands still stand. This addresses Nav2's small lateral commands stalling under the arm load.
- The reference robot integrity check remains. The only accepted structural deviation from old assets is the enumerated right-jaw mimic repair. Effective physics settings and signatures are recorded separately from the original actor contract.

The contact model follows [MuJoCo contact documentation](https://mujoco.readthedocs.io/en/latest/computation/index.html). Dynamic navigation footprints follow [Nav2's footprint interface](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/footprint/setup_footprint/).

## Implementation map

| File | Responsibility |
|---|---|
| `src/qda_mujoco/qda_mujoco/contact_profile.py` | Explicit contact solver profile and jaw linkage correction |
| `src/qda_mujoco/qda_mujoco/manipulation.py` | Live Cartesian/joint control, contact/lift/slip/placement checks |
| `src/qda_mujoco/qda_mujoco/node.py` | ROS service, exclusive ownership, watchdog and telemetry |
| `src/qda_mujoco/qda_mujoco/scene_control.py` | Experimental RL and PD binding, physical integrity and profile reporting |
| `src/qda_brain/qda_brain/skill_server.py` | Complete pick/carry/place orchestration, docking, Nav2 and cancellation |
| `src/qda_brain/qda_brain/mission.py` and `mission_node.py` | Whole-plan preflight and bounded sequential execution |
| `src/qda_bringup/qda_bringup/control_lease.py` and `command_mux.py` | Exclusive base control for navigation, docking and hold |
| `scripts/validate_physical_pick_place.py` | Deterministic free-body integration and fault acceptance |
| `scripts/run_physical_pick_place.py` | Production brain API acceptance and telemetry capture |

## Acceptance

`/manipulation/status` exposes the actual stage, object/TCP poses, pad normal forces, supporting bodies, maximum slip, maximum penetration and error code. These fields can also be recorded alongside the demo cameras.

RViz's existing status dock now shows physical task stage, load retention, pad force, slip and error. The red cube and its trail use the live MuJoCo object pose; they are not a prewritten animation. The standard demo recorder also captures `/manipulation/status`.

Success requires a lift of at least 10 cm with bilateral pad contact and no external support, payload retention during travel, release within 35 mm horizontally and 12 mm vertically of the destination, object speed below 15 mm/s, destination support contact and no remaining gripper contact. Arm/world penetration above 3 mm aborts. Payload-relative displacement above 35 mm or lost bilateral contact for more than 0.2 simulation seconds aborts.

Reports under `output/grasp/` retain both failed experiments and successful runs. A successful local physics experiment is distinct from a successful ROS/brain acceptance; check each report's `success`, `source`, task status and error fields. A `fault-test` success means a failure was correctly detected, not that a transfer completed.

## Verified run, 2026-10-27

The production ROS run is `output/grasp/ros-validated.json`, task ID `physical-pick-place-1793038073677546647`. It started from a fresh launch using the saved Nav2 configuration. The task journal ended in `SUCCEEDED`; post-cleanup physical telemetry remained `placed`, `completed=true`, `carrying=false`, with no error.

| Measurement | Result |
|---|---:|
| Object lift | 178.9 mm |
| Horizontal final-position error | 10.17 mm |
| Maximum payload slip relative to gripper | 6.01 mm |
| Maximum pad/object penetration | 1.28 mm |
| Final object speed | effectively zero |
| Simulation clock at task completion | 129.46 s |

The task moved the cube from `(0.78, 0.16)` to a target approximately 4.2 m away at `(5.0, 0.2)`, using the south route around the rack. This is a simulation acceptance result for one 40 g cube, not a statistical success-rate or hardware-load claim.

Additional checks:

- `drop-test.json`: opening the fingers caused a real drop; the monitor returned `PAYLOAD_LOST`.
- `cancel-test.json`: cancellation retained the gripped payload and did not report completion.
- `low-speed.json`: with the payload held and a lateral request of −0.03 m/s, measured mean lateral velocity was −0.0209 m/s.
- 14 unit/contract checks passed across brain contracts, coordination/leases, physical signature validation and terminal-state cleanup.
- Failed trials are retained, including the initial Nav2 workstation handoff, low-speed dead zone, post-task cleanup race and a corrected navigation configuration parse error. They are not counted as successes.

The live-frame recording uses `scripts/validate_physical_pick_place.py --video ...` and produces its own acceptance report. It runs the same production MuJoCo manipulation state machine and RL plant; the standalone recording harness uses bounded waypoint tracking instead of ROS Nav2. The separate ROS report above verifies the complete brain/Nav2 integration. The recording contains no pose playback, welds, narration or generated motion.

The inspected recording is `output/grasp/physical-pick-carry-place-validated.mp4` (1280 × 720, 10 fps, 122.4 seconds). Its separate report, `output/grasp/video-validated.json`, passed with 12.36 mm final horizontal error and no unintended arm contact recorded. Close-up views show grasp and release; the wider view shows the actual payload transfer around the rack. This video has not been published or embedded in the repository README.

For another complete demonstration after a successful placement, restart the simulation episode so the free cube begins on the pick table again:

```powershell
.\scripts\windows\stop_sim.ps1 -Backend mujoco
.\scripts\windows\run_sim.ps1 -Brain -Manual -Background
.\scripts\windows\run_pick_place.ps1 -FastValidation
```
