# reBotArm standalone MuJoCo control demo

This directory contains a ROS-free control example for the model under
`../xml`. It uses the seven native MJCF position actuators and adds MuJoCo
bias-force compensation to the six arm joints.

Run with the interactive viewer:

```bash
cd ~/reBotArm_develop_hjx/mujoco/rebotarm_b601_colored_control
python3 rebotarm_control_demo.py
```

Run a short headless test:

```bash
python3 rebotarm_control_demo.py --headless --no-realtime --duration 2
```

The seven target values are ordered as:

```text
[joint1, joint2, joint3, joint4, joint5, joint6, gripper]
```

Arm positions are radians. Gripper position is one finger's travel in metres.
The demo reads closed/open values from the current MJCF actuator range (now
`0.0` and `0.0485` m), validates them against both finger joint ranges, and
builds its waypoints dynamically. The home posture keeps joint4-6 at zero and
starts with the gripper closed. For custom control, replace the
`CyclicWaypointTrajectory` output with your planner and call
`RebotArmSimulation.set_target()` before each `step()`.

## Pick-and-place demo

Run the standalone yellow-cylinder pick-and-place task:

```bash
python3 rebotarm_pick_place_demo.py
```

Run it headlessly as a regression test:

```bash
python3 rebotarm_pick_place_demo.py --headless --no-realtime
```

The demo solves the user-positioned MJCF `grasp_center` site directly to the
object's live center. The open command is read from the current MJCF actuator
range. The close command is calculated from the cylinder radius and current
finger collision boxes, with only 0.3 mm preload; it does not blindly command
the gripper to zero and force the fingers through the object. A grasp succeeds
only when MuJoCo reports contact between the object and both left and right
finger collision geometries. The object is then lifted by contact forces and
friction—there is no virtual attachment, pose rewrite, or collision disabling.
Customize the destination with `--place X Y Z` or use another free body with
`--object`.

All end-effector FK, Jacobian and IK calculations use `grasp_center` directly;
the legacy `tcp` site is not read by this demo. Because `mj_jacSite` evaluates
the selected site's world pose, its offset from `end_link` is already included
and no separate TCP compensation is required.

The pick-and-place posture keeps `joint4`, `joint5` and `joint6` at zero, so the
wrist starts straight and remains straight through grasp, transfer and release.
The approach target is directly above the cylinder in world Z; `grasp_center`
then moves downward to the object's center without using a TCP-based retreat.
After placement, the arm retreats with the gripper open, returns to the initial
joint pose, and finally closes the empty gripper to the current MJCF lower
control limit.
