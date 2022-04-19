# sirius_navigation
Nodes and configurations needed for sirius rover navigation autonomy.
## Usage
### Nodes
- `simple_goal_relay`
  ```bash
  rosrun sirius_navigation simple_goal_relay.py
  ```
  Relays simple goal messages from rviz (topic `move_base_simple/goal`) to move_base action `move_base/move_base`

  **Subscribed topics**:
  - `move_base_simple/goal` [geometry_msgs/PoseStamped]
  
  **Called actions**:
  - `move_base/move_base` [mbf_msgs/MoveBaseAction]
### Launch files
- `navigation.launch`
  ```bash
  roslaunch sirius_navigation navigation.launch
  ```
  Runs navigation autonomy nodes  
  Navigation server `mbf_costmap_nav` is launched with `global_planner/GlobalPlanner` as planner, `base_local_planner/TrajectoryPlannerROS` as controller `clear_costmap_recovery` and `rotate_recovery` as recovery behavior. Moreover `simple_goal_relay` node is launched to relay simple goal messages from rviz to move_base action.