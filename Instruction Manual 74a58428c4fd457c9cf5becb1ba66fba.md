# Instruction Manual

## Environment Setup

```bash
roscd; cd ..; cd src
git clone https://github.com/SamCAE/mastering_ros_robot_description_pkg.git
cd ..
catkin_make
```

```bash
roslaunch seven_dof_arm_config demo.launch
```

## 1) Manipulate each joint position with different references

![Joints Minimum Limits](Instruction%20Manual%2074a58428c4fd457c9cf5becb1ba66fba/min.png)

Joints Minimum Limits

![Joints Maximum Limits](Instruction%20Manual%2074a58428c4fd457c9cf5becb1ba66fba/max.png)

Joints Maximum Limits

The following parts of the code (mastering_ros_robot_description_pkg/scripts/main.py) is modified for random joint states:

```python
def go_to_joint_state(self):
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = 45*(tau/360)
    joint_goal[1] = 30*(tau/360)
    joint_goal[2] = -70*(tau/360)
    joint_goal[3] = 60*(tau/360)
    joint_goal[4] = 100*(tau/360)
    joint_goal[5] = 70*(tau/360)
    joint_goal[6] = -30*(tau/360)

    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
```

```python
def main():
  try:
    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    input("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    tutorial = MoveGroupPythonInterfaceTutorial()

    input("============ Press `Enter` to execute a movement using a joint state goal ...")
    tutorial.go_to_joint_state()

    """
    input("============ Press `Enter` to execute a movement using a pose goal ...")
    tutorial.go_to_pose_goal()

    input("============ Press `Enter` to plan and display a Cartesian path ...")
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    input("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    tutorial.display_trajectory(cartesian_plan)

    input("============ Press `Enter` to execute a saved path ...")
    tutorial.execute_plan(cartesian_plan)
    """

    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
```

Initialization of the Moveit API:

```bash
roscd mastering_ros_robot_description_pkg; cd scripts; python main.py
```

![Rviz Visualization](Instruction%20Manual%2074a58428c4fd457c9cf5becb1ba66fba/1.gif)

Rviz Visualization

## 2) Manipulate the end effector to two different pose references

The following parts of the code (mastering_ros_robot_description_pkg/scripts/main.py) is modified for random end effector poses:

```python
def go_to_pose_goal(self):

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.2

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
```

```python
def go_to_pose_goal(self):

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.2
    pose_goal.position.y = 0.2
    pose_goal.position.z = 0.2

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
```

[Follow the same step](https://www.notion.so/Instruction-Manual-76d1d12aa8c146e7a7365e121d88621f) to comment out the undesired actions.

Initialization of the Moveit API:

```bash
roscd mastering_ros_robot_description_pkg; cd scripts; python main.py
```

![Pose A Rviz Visualization](Instruction%20Manual%2074a58428c4fd457c9cf5becb1ba66fba/2.gif)

Pose A Rviz Visualization

![Pose B Rviz Visualization](Instruction%20Manual%2074a58428c4fd457c9cf5becb1ba66fba/3.gif)

Pose B Rviz Visualization

## 3) Create and follow a trajectory by using the “cartesian path” function

The following parts of the code (mastering_ros_robot_description_pkg/scripts/main.py) is modified for creating a circle “0” trajectory:

```python
def plan_cartesian_path(self, offset=1):

    move_group = self.move_group
    waypoints = []
    
    dz=[0, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0]
    dy=[0, 0.1, 0.1, 0.1, 0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0]

    wpose = move_group.get_current_pose().pose
    new_pose = copy.deepcopy(wpose)
    for i in range(len(dz)):

        new_pose.position.z+=1*dz[i]
        new_pose.position.y+=1*dy[i]
        waypoints.append(copy.deepcopy(new_pose))
    
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,  
                                       0.01,        
                                       0.0)         
    return plan, fraction
```

```python
def main():
  try:
    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    input("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    tutorial = MoveGroupPythonInterfaceTutorial()
    
    input("============ Press `Enter` to plan and display a Cartesian path ...")
    cartesian_plan, fraction = tutorial.plan_cartesian_path()
    
    input("============ Press `Enter` to execute a saved path ...")
    tutorial.execute_plan(cartesian_plan)
    
    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
```

Initialization of the Moveit API:

```bash
roscd mastering_ros_robot_description_pkg; cd scripts; python main.py
```

![4.gif](Instruction%20Manual%2074a58428c4fd457c9cf5becb1ba66fba/4.gif)

## 4) Follow the same trajectory in “3” by using the “go_to_pose_goal” function

Changes to be done in the code:

```python
def go_to_pose_goal(self,xx,yy,zz):

    move_group = self.move_group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1
    pose_goal.position.x = xx
    pose_goal.position.y = yy
    pose_goal.position.z = zz

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
```

```python
def main():
  try:
    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    input("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    tutorial = MoveGroupPythonInterfaceTutorial()
    
    input("============ Press `Enter` to execute a movement using a pose goal ...")
		tutorial.go_to_pose_goal(0.030516,0.22,0.43354)
    tutorial.go_to_pose_goal(0.2221,0,0.43354)
    tutorial.go_to_pose_goal(0,-0.2221,0.43354)
    tutorial.go_to_pose_goal(-0.2221,0,0.43354)
    tutorial.go_to_pose_goal(0.030516,0.22,0.43354)
    
    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
```

## 5) Difference between the coding process in (3) and (4)

In part “3” the code takes into account that the trajectory could have a specific velocity at the points it passes through and hence plans the acceleration and velocity in a smooth form. However, in part “4” the individual function go_to_pose_goal(x,y,z) will plan the trajectory while considering that the initial and final velocity to be 0. Therefore, the trajectory will appear more discrete/discontinous.