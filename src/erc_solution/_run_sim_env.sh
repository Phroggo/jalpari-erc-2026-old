#!/bin/bash
# Launch helper used from `docker exec` (which bypasses the container ENTRYPOINT,
# so GZ_SIM_RESOURCE_PATH must be set here). Mirrors docker/entrypoint.sh.
source /opt/ros/humble/setup.bash
source /opt/erc_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="\
/opt/erc_ws/install/erc_description/share:\
/opt/erc_ws/install/omni_base_description/share:\
/opt/erc_ws/install/tiago_pro_description/share:\
/opt/erc_ws/install/pal_sea_arm_description/share:\
/opt/erc_ws/install/tiago_pro_head_description/share:\
/opt/erc_ws/install/pal_pro_gripper_description/share:\
/opt/erc_ws/install/pal_gripper_description/share:\
/opt/erc_ws/install/pal_urdf_utils/share:\
/opt/ros/humble/share"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/erc_ws/install/gz_ros2_control/lib"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=57
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
export GZ_IP=127.0.0.1
export GZ_TRANSPORT_IP=127.0.0.1
cd /opt/erc_ws
exec "$@"
