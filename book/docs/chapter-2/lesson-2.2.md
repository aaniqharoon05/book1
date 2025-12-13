---
sidebar_position: 2
---

# Lesson 2.2: Building a World in Gazebo

**Learning Objectives:**
*   Understand the components of a Gazebo world file (.world).
*   Learn how to add simple shapes and models to a Gazebo world.
*   Launch a pre-built Gazebo world.

## Creating the Virtual Environment

Before we can put our robot in the simulation, we need to build the world it will live in. In Gazebo, this is done using a `.world` file, which is an XML file written in a format called **SDF (Simulation Description Format)**.

An SDF world file is simple. It lets you define everything in the scene:
*   The lighting (sun, lamps).
*   The physics properties (like gravity).
*   The ground plane.
*   Static objects like walls, tables, and obstacles.
*   The robot(s) you want to include.

Gazebo also has an online "Fuel" depot where you can find and download pre-built models for things like tables, chairs, and entire buildings, saving you a lot of time.

**[DIAGRAM: A simplified XML tree structure. The root is `<sdf>`. It branches to `<world>`. The `<world>` branch has leaves for `<light>`, `<physics>`, `<include> (for ground_plane)`, and `<model name='box_obstacle'>`.]**

## Practical Example: A Simple Gazebo World File

Here is a very simple `.world` file that defines a world with a sun, a ground plane, and a single box obstacle.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- Add a light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Define the physics engine properties -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Add a simple box model as an obstacle -->
    <model name="obstacle_box">
      <static>true</static> <!-- This means it won't move -->
      <pose>2 0 0.5 0 0 0</pose> <!-- Position: x, y, z, roll, pitch, yaw -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size> <!-- A 1x1x1 meter box -->
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```
In this file, you can see how we include existing models like the `sun` and `ground_plane`. We also define our own model, a simple `box`, and set its position in the world. The `<collision>` tag defines its physical shape for the physics engine, and the `<visual>` tag defines how it looks.

## Recap:
*   Gazebo environments are defined in `.world` files using the SDF format.
*   You can define lighting, physics, and add models to your world.
*   Models have `<collision>` properties (for physics) and `<visual>` properties (for looks).
*   You can include pre-built models from Gazebo's Fuel library to quickly build scenes.

## Try It Yourself:
If you have Gazebo installed, you can save this file as `my_world.world` and run it with the command `gazebo my_world.world`. Once it loads, try using your mouse to navigate the scene. Can you find the box you added? Try changing the `<pose>` values in the file to move the box to a different location.
