# erc_solution — ERC 2026 Phase 1 (Library Assistant Robot)

Solution package for the Emirates Robotics Competition 2026 simulation phase.
TIAGo Pro reads the randomised overhead shelf-column marker, drives to the
requested column, finds the requested book colour, picks it with one arm
(the right arm for retrieval and delivery) and places it in the collection bin.
Every step is driven by the robot's own sensors: RGB-D camera, joint states,
front/rear lasers and physical contact feedback. Nothing reads an object's
true pose from the simulator, and nothing attaches or teleports the book.

```bash
# terminal 1 — base simulation (unmodified competition package)
ros2 launch erc_bringup simulation.launch.py

# terminal 2 — this solution
ros2 launch erc_solution solution.launch.py shelf_column_number:=2 book_colour:=red
```

`shelf_column_number` (1–5) is the **digit printed on the marker**, not a
fixed physical position — the marker permutation is randomised every launch,
so the robot locates it visually. `book_colour` ∈ {red, green, blue, yellow}.
See [REBUILD.md](REBUILD.md) for build/setup detail and verification history.
See [CODE_GUIDE.md](CODE_GUIDE.md) for the file layout and coordinate frames.

## Architecture

A single ROS 2 node (`book_retrieval`, entry point `erc_solution.mission:main`)
runs an observation-gated state machine. `Mission` extends `Robot`, so
perception, kinematics and localization are plain Python modules called
directly from the state machine rather than separate nodes talking over
topics — there is one process, one control loop, and no risk of the state
machine acting on a stale cross-node message.

```
Mission(Robot).run()
  park()                     -> both arms tucked into a compact, collision-clear posture
  find_column()               -> WallLocalizer + MarkerDetector: locate the requested
                                  shelf-column digit, publish /erc/shelf_column_identification,
                                  save an annotated evidence image
  navigate() / find_book()    -> laser-safe base motion; vision.objects() finds the
                                  requested book colour, publishes /erc/shelf_row_identification,
                                  saves an annotated evidence image
  pick()                      -> IK-planned insertion, bilateral-finger-contact-verified grasp
  deliver()                   -> prepare_transport() (raises low-row books clear of the shelf),
                                  return to start, find_bin(), lower and release into the bin,
                                  confirmed by /bin_contacts
```

- **`robot.py`** (`Robot(Node)`) — sensor fusion (camera, joint states, both
  lasers), scan-matched arena pose with TF sensor/arm transforms, physical grasp
  verification (`held()`, `require_held()` — bilateral finger contact only),
  trajectory publishing, base safety checks before every motion, and a
  structured per-trial JSONL log.
- **`mission.py`** (`Mission(Robot)`) — the task state machine described above.
- **`vision.py`** — HSV colour segmentation for books, spine-aware
  aspect-ratio filtering, depth-patch validation, bin-vs-book discrimination,
  row-from-height mapping.
- **`kinematics.py`** — URDF-parsed forward kinematics and bounded
  multi-start inverse kinematics (the grasp-frame X axis, not Z, is the
  physical approach direction); raises rather than silently failing on an
  unreachable pose.
- **`localization.py`** (`WallLocalizer`) — scan-matches both lasers against
  the arena's known static walls; no odometry dependency. Resolves the
  square-room heading ambiguity from the shelf marker's visual position.
- **`perception/marker_detector.py`** + **`perception/digit_mlp.py`** — a
  small dependency-free (NumPy only) MLP that classifies the overhead column
  digit from a camera crop.

## Outputs for scoring

- `/erc/shelf_column_identification` (`std_msgs/Int32`) — published once the
  column marker is stably identified.
- `/erc/shelf_row_identification` (`std_msgs/Int32`) — published once the
  target book's row is stably identified.
- `erc_images/column_<n>_<timestamp>.jpg` — marker with bounding box.
- `erc_images/row_<r>_<colour>_<timestamp>.jpg` — target book with bounding box.
  Every saved image has both simulation time and UTC wall-clock time drawn
  into the pixels.

## Error handling

`arm_pose()` and `require_held()` verify the robot actually reached a
commanded pose and is still gripping the book (not just that a trajectory
was sent) before the state machine advances, and raise a descriptive
`RuntimeError` — logged as `FAILED: <reason>` — the moment sensor evidence
stops matching what the plan assumed, instead of continuing on a false
assumption. `find_column()`, `find_book()` and `find_bin()` require several
consecutive stable detections before committing, so a single noisy frame
cannot trigger the wrong action. Navigation checks both lasers for
obstructions before every base motion.

## Development-only tooling

`scripts/*.py` are diagnostics used while developing this solution
(inspecting live sensor frames, resuming from a known contact-verified
state to isolate a later stage) — none of them are launched by
`solution.launch.py` and none of them are a substitute for a complete trial.
