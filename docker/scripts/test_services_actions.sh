#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

server_pid=""
cleanup() {
  if [[ -n "${server_pid}" ]]; then
    kill "${server_pid}" 2>/dev/null || true
    wait "${server_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

run_and_match() {
  local pattern="$1"
  shift
  local output
  output=$("$@")
  printf '%s\n' "${output}"
  grep -Eq "${pattern}" <<<"${output}"
}

test_python_client() {
  run_and_match 'difference=6' ros2 run services_py service_client subtract 10 4
  run_and_match 'is_even=False' ros2 run services_py service_client even 7
  run_and_match 'fahrenheit=77' ros2 run services_py service_client temperature 25
  run_and_match 'area=28\.27' ros2 run services_py service_client circle 3
  run_and_match "reversed_text='acitobor'" ros2 run services_py service_client reverse robotica
  run_and_match 'result=7' ros2 run services_py service_client dot 1 2 3 4 0 1
  run_and_match 'minimum=2.*maximum=10.*average=6' ros2 run services_py service_client stats 4,8,2,10
  run_and_match 'minimum=0.*maximum=0.*average=0' ros2 run services_py service_client stats ''
  run_and_match 'slope=1\.9.*intercept=0\.1.*r_squared=0\.99' \
    ros2 run services_py service_client regression 1,2,3,4 2.1,3.9,6.2,7.8
  run_and_match 'index=1' ros2 run services_py service_client closest 0 0 5:5,1:1,2:2
  run_and_match 'index=-1' ros2 run services_py service_client closest 0 0 ''
  run_and_match 'reachable=True' ros2 run services_py service_client ik 1.2 0.8 1.0 1.0
  run_and_match 'reachable=False' ros2 run services_py service_client ik 10 0 1.0 1.0
}

test_cpp_client() {
  run_and_match '^6$' ros2 run services_cpp service_client subtract 10 4
  run_and_match '^false$' ros2 run services_cpp service_client even 7
  run_and_match '^77' ros2 run services_cpp service_client temperature 25
  run_and_match '^28\.27' ros2 run services_cpp service_client circle 3
  run_and_match '^acitobor$' ros2 run services_cpp service_client reverse robotica
  run_and_match '^7$' ros2 run services_cpp service_client dot 1 2 3 4 0 1
  run_and_match 'min=2 max=10 average=6' ros2 run services_cpp service_client stats 4,8,2,10
  run_and_match 'min=0 max=0 average=0' ros2 run services_cpp service_client stats ''
  run_and_match 'slope=1\.9.*intercept=0\.1.*r_squared=0\.99' \
    ros2 run services_cpp service_client regression 1,2,3,4 2.1,3.9,6.2,7.8
  run_and_match 'index=1' ros2 run services_cpp service_client closest 0 0 5:5,1:1,2:2
  run_and_match 'index=-1' ros2 run services_cpp service_client closest 0 0 ''
  run_and_match 'reachable=true' ros2 run services_cpp service_client ik 1.2 0.8 1.0 1.0
  run_and_match 'reachable=false' ros2 run services_cpp service_client ik 10 0 1.0 1.0
}

export ROS_DOMAIN_ID=41
ros2 run services_cpp service_servers >/tmp/sonho-services-cpp.log 2>&1 &
server_pid=$!
sleep 2
test_python_client
cleanup
server_pid=""
sleep 1

export ROS_DOMAIN_ID=42
ros2 run services_py service_servers >/tmp/sonho-services-py.log 2>&1 &
server_pid=$!
sleep 2
test_cpp_client
cleanup
server_pid=""
sleep 1

export ROS_DOMAIN_ID=43
ros2 run actions_cpp action_servers >/tmp/sonho-actions-cpp.log 2>&1 &
server_pid=$!
sleep 2
run_and_match 'success=True.*final_value=3' ros2 run actions_py action_client count 3
run_and_match 'completed=True' ros2 run actions_py action_client timer 0.3
run_and_match 'completed=False' ros2 run actions_py action_client timer 1.0 --cancel-after 0.15
cleanup
server_pid=""
sleep 2

export ROS_DOMAIN_ID=44
ros2 run actions_py action_servers >/tmp/sonho-actions-py.log 2>&1 &
server_pid=$!
sleep 2
run_and_match 'success=true.*final_value=3' ros2 run actions_cpp action_client count 3
run_and_match 'completed=true' ros2 run actions_cpp action_client timer 0.3
run_and_match 'completed=false' ros2 run actions_cpp action_client timer 1.0 0.15
cleanup
server_pid=""

printf '%s\n' 'All cross-language ROS 2 service and action checks passed.'
