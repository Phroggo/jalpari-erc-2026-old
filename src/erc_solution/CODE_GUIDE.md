# Where things are

Start with `launch/solution.launch.py`, then `erc_solution/mission.py`.
The mission runs the stages in order; the other modules handle the details.

| File | What it handles |
|---|---|
| `erc_solution/mission.py` | Finding the column/book, picking it and returning to the bin |
| `erc_solution/robot.py` | ROS sensors, motion commands, feedback checks and evidence files |
| `erc_solution/localization.py` | Tracking the base from laser scans and the visible shelf |
| `erc_solution/kinematics.py` | Reading arm geometry and finding reachable joint poses |
| `erc_solution/vision.py` | Colour/depth filtering, bounding boxes and row numbering |
| `erc_solution/perception/` | Reading marker digits with the trained model |
| `erc_solution/recording_view.py` | Optional camera window, timer and fullscreen request |
| `erc_solution/navigation_view.py` | Commanded targets, localized trajectory and laser/wall markers for RViz |
| `test/test_rebuild.py` | Regression checks for perception, contact and arm geometry |

Positions are in metres and angles are in radians. Check the frame before using
a position: `arena` is the fixed room frame, `base_footprint` moves with the robot,
and the arm solver normally works from `torso_lift_link`. `matrix(target, source)`
maps a point from source into target.

`scripts/` contains manual diagnostics. `training/` contains the digit-classifier
tools (`gen_dataset.py`, `train_digit_mlp.py`) that produced `models/digit_mlp.npz`.
Neither directory is imported by the mission. Use the launch file for a complete
trial.

Comments explain why a check or offset is needed. Keep them short, keep units and
frames clear, and update them whenever behavior changes.
