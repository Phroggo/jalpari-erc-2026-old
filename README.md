# Jalpari — ERC 2026

Team Jalpari's Phase 1 Library Assistant Robot submission. The repository includes
the competition simulator and robot packages, plus our `erc_solution` package.
TIAGo Pro identifies the requested column and book with its RGB-D camera, picks
the book with its right gripper, returns to the start zone, visually identifies
the collection bin and releases the book into it.

## Build

Use an x86-64 Linux host with Docker Engine, Docker Compose v2 and an X11/XWayland
display. NVIDIA acceleration requires the NVIDIA Container Toolkit; software
rendering is slower. The container uses ROS 2 Humble and Gazebo Harmonic.
Source dependencies are included, but the first Docker build needs internet
access to download the base image and system/ROS dependencies.

```bash
git clone https://github.com/Phroggo/jalpari-erc-2026.git
cd jalpari-erc-2026
./docker/up.sh --build
./docker/attach.sh
```

`docker/up.sh` recreates the container named `erc_sim`. Do not run it while
another trial is active. Inside the container:

```bash
cd /opt/erc_ws
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
ros2 launch erc_bringup simulation.launch.py
```

## Launch the challenge trial

Keep the simulation running. In a second host terminal, from this repository:

```bash
./docker/attach.sh
```

Then, inside that second container terminal:

```bash
cd /opt/erc_ws
source install/setup.bash
ros2 launch erc_solution solution.launch.py shelf_column_number:=2 book_colour:=red
```

`shelf_column_number` accepts 1–5 and means the digit visible above the column,
not a fixed arena position. `book_colour` accepts `red`, `green`, `blue` or
`yellow`. Both terminals use the same ROS domain (23 by default). Start exactly
one solution instance, after `simulation.launch.py` is running.

For recording, after building, `./run_sim.sh` starts Gazebo and the optional
camera/timer viewer; `./run_solution.sh 2 red` starts the trial in another host
terminal. The viewer shows wall time and elapsed `/clock` simulation time.
It requests fullscreen Gazebo where the window manager supports it. These
helpers are optional; the official launch commands above work independently.

## Our package

[`src/erc_solution/README.md`](src/erc_solution/README.md) describes the team
package; [`CODE_GUIDE.md`](src/erc_solution/CODE_GUIDE.md) explains its code.
The launch file is [`solution.launch.py`](src/erc_solution/launch/solution.launch.py).
Dependencies are declared in [`package.xml`](src/erc_solution/package.xml).

| Module | Responsibility |
|---|---|
| `mission.py` | Task sequence, shelf/book/bin selection and pick/place planning |
| `vision.py`, `perception/` | Live RGB-D detection and digit classification |
| `localization.py` | Onboard laser localization, initialized from the visual shelf observation |
| `kinematics.py` | Arm geometry, reachability and inverse kinematics |
| `robot.py` | Sensor callbacks, motion execution, safety checks and evidence |
| `recording_view.py` | Optional camera display and recording timers |
| `navigation_view.py` | Read-only targets, localized trajectory and wall/scan overlay for RViz |

Perception, planning and execution are separate modules in one mission node,
not separate ROS processes. The included trained digit model is loaded locally;
evaluation does not need a training step or a network service.

See [RViz navigation screenshots](docs/NAVIGATION_SCREENSHOTS.md) for capturing
commanded targets versus the executed trajectory for the report.

## Scoring outputs

| Output | Type / location |
|---|---|
| Identified column | `/erc/shelf_column_identification`, `std_msgs/msg/Int32` |
| Identified row (1–4) | `/erc/shelf_row_identification`, `std_msgs/msg/Int32` |
| Annotated live images | Repository-root `erc_images/*.jpg` |
| Detected locations | Matching `.json` sidecars with `frame`, `detected_point` and bounding box |
| Trial stage log | `erc_images/trial_*.jsonl` |

The ROS node draws the bounding box, simulation timestamp and UTC timestamp
onto live camera images during the trial. No pre-generated trial images are
included. Compose maps this repository's `erc_images/` to
`/opt/erc_ws/erc_images`. Use the documented symlink build and launch from
`/opt/erc_ws`; if using another layout or a non-symlink installation, explicitly
set `images_dir:=/absolute/path/to/this/repository/erc_images`.

The rulebook's underscores can appear as spaces when copied from the PDF.
The exact ROS names and `erc_images` directory above retain the underscores.

## Checks and limitations

```bash
python3 -m unittest discover -s src/erc_solution/test -p test_rebuild.py -v
QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s src/erc_solution/test -p test_recording_clock.py -v
```

The solution checks sensor freshness, stable visual detections, IK reachability,
measured arm alignment and bilateral finger contact. Some brief contact gaps are
rechecked; most persistent failures abort rather than automatically regrasp or
replan. Collision checks are not a proof of collision-free motion. A matching
book/bin contact is not proof of gentle-placement scoring.

Placement lowers into the bin in up to three 3 cm steps. Carrying motion skips
unneeded arm repositioning when the torso lift provides enough clearance.
These motion changes have not yet been validated in a complete trial.
Hardware performance and physics timing can affect results. An isolated clean-container build passed
for all 33 packages; all 30 mission regression tests, 3 navigation-display tests and 3 recording-clock tests
passed. These checks do not replace a full simulation trial.

## Simulator baseline and credits

Team Jalpari contributors:

- [Phroggo](https://github.com/Phroggo)
- [gloriap05](https://github.com/gloriap05)
- [aar4n](https://github.com/aar4n)
- [Unfluctuating](https://github.com/Unfluctuating)

The included simulator sources use the public
[ERC competition baseline](https://github.com/dfl-rlab/erc_sim_2026/tree/68e175fada5b8115b0f7bbb0e9b2b08a96504f22).
All source packages and the trained model needed to build and run the solution
are included here; no private repository access is required. Contributor credits
and third-party license notices are retained in the package files.
