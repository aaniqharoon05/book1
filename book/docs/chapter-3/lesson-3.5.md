---
sidebar_position: 5
---

# Lesson 3.5: Nav2: How Do I Get There?

**Learning Objectives:**
*   Define the role of a "navigation stack."
*   Describe the main components of Nav2 (Planner, Controller, Costmap).
*   Understand how Nav2 uses information from VSLAM.

## From "Where Am I?" to "Here's How to Go"

Our robot now knows where it is (thanks to VSLAM). But how does it get from Point A to Point B? This is the job of the **Navigation Stack**. In ROS 2, the standard navigation stack is called **Nav2**.

Nav2 is not a single node; it's a sophisticated system of interacting nodes that handle the entire process of autonomous navigation. Its main components are:

1.  **The Planner:** You give Nav2 a goal (e.g., "go to the kitchen"). The global planner looks at the overall map (provided by SLAM) and calculates the best high-level path to get there, avoiding walls and major obstacles. It's like planning your route on a GPS before you start driving.
2.  **The Controller (Local Planner):** Once the robot starts moving, the controller takes over. Its job is to follow the global path while avoiding *new* or *moving* obstacles (like a person walking by). It only looks at a small area right around the robot and calculates immediate motor commands (e.g., "turn left slightly, move forward at 0.5 m/s"). It's like actually steering your car to stay in your lane and avoid hitting the car in front of you.
3.  **The Costmap:** This is the special map that Nav2 uses. It's built from the SLAM map but has extra layers of information. It marks areas near obstacles as "high cost" (more dangerous) and open areas as "low cost." The planners then try to find the path with the lowest total cost. You can even add custom rules, like making it "costly" to go on carpet.

For a humanoid, Nav2 is especially important and complex. Path planning for a round robot on wheels is simple. Path planning for a two-legged robot that has to step over things is a major challenge that requires specialized planners and controllers.

**[DIAGRAM: A flowchart. An arrow from a "Goal Pose" box points to Nav2. Inside a large "Nav2" box are three smaller boxes. "Planner" gets the goal and a "Global Costmap" to create a "Global Path". The "Global Path" is fed to the "Controller", which also uses a "Local Costmap" and "VSLAM Pose" to generate "Motor Commands".]**

## Practical Example: Using Nav2 in ROS 2

Running Nav2 involves a complex launch file that starts all its different servers (planner, controller, costmap, etc.). As a user, your interaction is often simple. You give it a goal.

In ROS 2, you can do this from the command line or from another node. To send a goal, you use a ROS 2 **action** (remember those from Lesson 1.4?), because navigation is a long-running task.

The action is called `/navigate_to_pose`. You send it a message containing the coordinates and orientation you want the robot to go to.

```bash
# A command-line example of sending a navigation goal to Nav2
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {
      frame_id: 'map',
      stamp: {sec: 0, nanosec: 0}
    },
    pose: {
      position: {x: 1.5, y: -2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```
When Nav2 receives this goal, its planner will create a path, and its controller will start issuing commands to the robot's motors (via another ROS 2 topic, like `/cmd_vel`) to begin moving. It will provide feedback on the action topic about its progress and a final result when it arrives.

## Recap:
*   **Nav2** is the standard ROS 2 navigation stack that gets a robot from Point A to Point B.
*   The **Global Planner** creates the overall route.
*   The **Local Planner (Controller)** handles immediate movements and obstacle avoidance.
*   The **Costmap** is a special map that shows the "danger" of different areas.
*   Nav2 takes in the robot's pose from a SLAM system and outputs motor commands.

## Try It Yourself:
Look at the costmap concept. If a humanoid robot needs to go up a staircase, how might you design a costmap? Would the stairs be high cost or low cost? (Answer: For a wheeled robot, stairs would be infinitely high cost—an impassable obstacle. For a humanoid, you might create a special "stair" layer in the costmap that is "low cost" for a bipedal planner but high cost for a wheeled one.)
