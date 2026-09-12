# Build and run — 11 September 2026

See the [repository README](../../README.md) for the pinned official baseline,
current test results and limitations.

From the repository root on the host:

```bash
./docker/up.sh --build
./docker/attach.sh
```

Inside the container:

```bash
cd /opt/erc_ws
colcon build --symlink-install
source install/setup.bash
python3 -m unittest discover -s src/erc_solution/test -p test_rebuild.py -v
ros2 launch erc_bringup simulation.launch.py
```

In another attached container terminal:

```bash
cd /opt/erc_ws
source install/setup.bash
ros2 launch erc_solution solution.launch.py shelf_column_number:=2 book_colour:=red
```

Both terminals must use the same ROS_DOMAIN_ID (23 by default). Direct docker
exec commands should invoke `/entrypoint.sh bash -lc '...'` for ROS/Gazebo paths.
The supplied Compose mount maps repository-root erc_images into
/opt/erc_ws/erc_images. Verify the mount when reusing a container. Always map
the repository-root erc_images folder, not a nested package folder. With non-symlink
installs the default derives from the launch working directory; set images_dir
explicitly to a host-visible repository folder if needed.

Only `erc_solution.mission:main` runs during evaluation. Diagnostic continuations
are not complete trials and must not run alongside the autonomous solution.
