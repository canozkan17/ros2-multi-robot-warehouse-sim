#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30

for robot in robot1 robot2 robot3; do
  echo "Activating Nav2 for $robot..."
  for node in map_server amcl planner_server controller_server behavior_server bt_navigator; do
    ros2 service call /${robot}/${node}/change_state \
      lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}" > /dev/null 2>&1
    sleep 0.5
    ros2 service call /${robot}/${node}/change_state \
      lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}" > /dev/null 2>&1
    echo "  $node: done"
  done
  echo "$robot activated!"
done
echo "All done!"
