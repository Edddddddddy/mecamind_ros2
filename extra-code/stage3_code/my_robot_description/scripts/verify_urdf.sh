#!/usr/bin/env bash
set -euo pipefail

echo "===== P2 milestone 1 URDF verification ====="

pkg_prefix="$(ros2 pkg prefix my_robot_description)"
xacro_file="${pkg_prefix}/share/my_robot_description/urdf/my_robot.urdf.xacro"
urdf_file="/tmp/my_robot_verify.urdf"
graph_dir="/tmp/my_robot_verify_graph"

echo "[1/6] Expand Xacro"
xacro "${xacro_file}" -o "${urdf_file}"

echo "[2/6] Check URDF"
check_urdf "${urdf_file}" >/tmp/my_robot_check_urdf.log
cat /tmp/my_robot_check_urdf.log

echo "[3/6] Count structure and physical tags"
link_count="$(grep -c '<link name' "${urdf_file}")"
joint_count="$(grep -c '<joint name' "${urdf_file}")"
collision_count="$(grep -c '<collision>' "${urdf_file}")"
inertial_count="$(grep -c '<inertial>' "${urdf_file}")"
gazebo_count="$(grep -c '<gazebo reference=' "${urdf_file}")"
echo "links=${link_count} joints=${joint_count} collisions=${collision_count} inertials=${inertial_count} gazebo_refs=${gazebo_count}"

test "${link_count}" -eq 6
test "${joint_count}" -eq 5
test "${collision_count}" -eq 5
test "${inertial_count}" -eq 5
test "${gazebo_count}" -ge 5

echo "[4/6] Generate URDF graph"
rm -rf "${graph_dir}"
mkdir -p "${graph_dir}"
(
  cd "${graph_dir}"
  urdf_to_graphviz "${urdf_file}" my_robot_verify >/dev/null
)
test -f "${graph_dir}/my_robot_verify.pdf"

echo "[5/6] Check launch and RViz assets"
test -f "${pkg_prefix}/share/my_robot_description/launch/display.launch.py"
test -f "${pkg_prefix}/share/my_robot_description/rviz/my_robot.rviz"

echo "[6/6] Check expected final frames"
grep -q 'back_caster_link' "${urdf_file}"
grep -q 'left_wheel_joint' "${urdf_file}"
grep -q 'right_wheel_joint' "${urdf_file}"

echo "===== Verification passed ====="
