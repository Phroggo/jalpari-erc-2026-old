# Jalpari submission verification — 12 September 2026

This is a new source snapshot, not a merge into either team's development repo.
The prior audit in [docs/PREVIOUS_AUDIT.md](docs/PREVIOUS_AUDIT.md) is historical;
its test counts and runtime claims do not describe this revised snapshot.

## Static checks

- All vendored ROS packages and the trained digit model are included.
- `erc_solution/launch/solution.launch.py` accepts the required integer column
  and colour arguments. Runtime validation rejects unsupported values.
- Identification publishers use the exact required topic names and Int32 type.
- Live evidence uses repository-root `erc_images/` through the supplied mount,
  with visible image timestamps and bounding boxes. JSON sidecars save detected
  coordinates in the arena frame; these are sensor-derived, not simulator poses.
- Package dependencies cover runtime imports and the optional recording viewer.
- Build products, old trials, screenshots, credentials and development Git
  history are not part of the submission snapshot. Upstream attribution remains.

## Runtime changes included

- Placement descends at most 9 cm in three 3 cm steps before opening the gripper.
- A torso lift that already reaches the existing 1.20 m carrying-height threshold
  skips the additional arm path after verifying the grasp.
- Remaining arm waypoints use a 0.7 s floor and at least joint displacement /
  0.30 seconds. Larger joint turns can take longer, not shorter. This bounds
  requested average speed, not actual peak speed. Existing checks are retained.
- Recording viewer shows wall and simulation clocks, without speed extrapolation.

## Limits on readiness claims

The last observed yellow-book trial dropped its book during transport raising.
The revised transport motion has regression coverage but has not completed a
new end-to-end simulation trial. Neither a perfect success rate, zero collisions,
gentle-placement points nor cross-machine reproducibility has been established.
Most failures stop the trial instead of autonomously recovering. Video, report,
eligibility and submission-form completion remain separate team responsibilities.

## Fresh-snapshot verification

An isolated container with networking disabled and ROS domain 85 mounted this
snapshot's source read-only into an empty workspace. It used the existing
official-Dockerfile image `sha256:4fe156dfb0a3521d8c4e1e5bd4ccc11b605b29495594e16373310efc12e044c3`;
the base image was not rebuilt or downloaded again for this check.

- All 33 ROS packages built successfully (14.9 seconds).
- All 30 mission/perception/kinematics regression tests passed.
- All 3 offscreen recording-clock tests passed.
- Installed launch argument discovery passed, including the required column and
  colour arguments and default `/opt/erc_ws/erc_images` output directory.

This verifies a clean source build on the current host, not a full mission on
another computer. The test container did not launch Gazebo or move a robot.
