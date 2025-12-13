---
sidebar_position: 5
---

# Lesson 1.5: Describing Your Robot with URDF

**Learning Objectives:**
*   Explain the purpose of the Unified Robot Description Format (URDF).
*   Describe the key elements of a URDF file: `<link>` and `<joint>`.
*   Read and understand a simple URDF file for a two-wheeled robot.

## The Robot's Blueprint

So far, we've built the robot's nervous system, but we haven't described its body. How does the software know what the robot looks like? How many wheels does it have? Where is the camera mounted?

This is the job of the **Unified Robot Description Format (URDF)**. A URDF file is an XML file that describes the physical structure of your robot. It's like a CAD model, but for software. It defines the robot's parts and how they connect.

A URDF file is made of two main components:
1.  **`<link>`**: A link is a physical part of the robot that has mass and a shape. Think of it as a bone. Examples: a wheel, a robot arm segment, the main chassis.
2.  **`<joint>`**: A joint connects two links together. Think of it as a joint in your body, like an elbow or a knee. It defines how the two links can move relative to each other.

**Key Joint Types:**
*   `revolute`: Rotates around an axis, like a wheel on an axle.
*   `continuous`: A revolute joint with no limits, like a spinning wheel.
*   `prismatic`: Slides along an axis, like a piston.
*   `fixed`: Fuses two links together. They cannot move relative to each other. This is very common for mounting sensors.

**[DIAGRAM: A simple cartoon robot arm with two segments and a gripper. The main body is labeled "base_link". The first arm segment is "arm_link_1", connected by "joint_1 (revolute)". The second segment is "arm_link_2", connected by "joint_2 (revolute)". This clearly shows the link-joint-link structure.]**

## Practical Example: A Simple Two-Wheeled Robot URDF

Let's look at a simplified URDF for a robot with a chassis and two wheels.

```xml
<?xml version="1.0"?>
<robot name="simple_bot">

  <!-- First, define the main body of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.1" />
      </geometry>
    </visual>
  </link>

  <!-- Now, define the left wheel -->
  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04" />
      </geometry>
    </visual>
  </link>

  <!-- Connect the left wheel to the base with a joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 0.12 -0.05" rpy="1.5707 0 0" />
    <axis xyz="0 0 -1" />
  </joint>

  <!-- Define the right wheel -->
  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04" />
      </geometry>
    </visual>
  </link>

  <!-- Connect the right wheel to the base with a joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0 -0.12 -0.05" rpy="1.5707 0 0" />
    <axis xyz="0 0 -1" />
  </joint>

</robot>
```
This file tells ROS 2 that we have a robot with three parts (`base_link`, `left_wheel_link`, `right_wheel_link`). It also defines that the wheels are connected to the base by `continuous` joints, meaning they can spin forever. Simulation tools will use this file to create a 3D model of our robot.

## Recap:
*   URDF is an XML file that describes a robot's physical structure.
*   A **link** is a physical part of the robot.
*   A **joint** connects two links and defines their motion.
*   ROS 2 tools use URDF files to visualize and simulate the robot correctly.

## Try It Yourself:
Read through the example URDF file. Can you identify the name of the robot? Can you see where the `<origin>` tag is used to position the wheels relative to the `base_link`? Try to imagine adding a `camera_link` connected to the `base_link` with a `fixed` joint. What would that look like in the XML?
