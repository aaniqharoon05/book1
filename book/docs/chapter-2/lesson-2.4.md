---
sidebar_position: 4
---

# Lesson 2.4: Simulating Physics: Gravity and Collisions

**Learning Objectives:**
*   Understand how physics engines work in simulation.
*   Identify `<collision>` and `<inertial>` tags in a URDF.
*   Recognize the importance of friction and other physical properties.

## Making it Behave like the Real World

Our robot looks right, but does it *act* right? A digital twin is only useful if it obeys the laws of physics. This is the job of a **physics engine**. Gazebo uses engines like ODE (Open Dynamics Engine) or DART.

These engines calculate the effect of forces on every object in the world, many times per second. This includes:
*   **Gravity:** Making objects fall.
*   **Collisions:** Preventing objects from passing through each other and calculating how they bounce.
*   **Inertia:** The resistance of an object to changes in its motion. A heavy object is harder to start or stop moving.
*   **Friction:** The force that resists motion between surfaces. Without friction, our robot's wheels would just spin in place!

To make this work, we need to give our URDF model more physical details.
*   ``<collision>``: We saw this before. It defines the physical shape of the link for the physics engine. This can be a simpler shape (like a sphere) than the visual model to make calculations faster.
*   ``<inertial>``: This tag defines the mass and inertia of the link. It tells the physics engine how heavy the part is and how its mass is distributed.

**[DIAGRAM: A robot wheel link is shown. An arrow points from it to two separate shapes. One is a detailed, visually appealing wheel model labeled "``<visual>``". The other is a simple, perfect cylinder labeled "``<collision>`` (for fast physics)". Another arrow points to a box of text labeled "``<inertial> mass='0.5kg'``".]**

## Practical Example: Adding Physics Properties to a URDF

Let's enhance the `base_link` from our robot in Lesson 1.5. We'll add a proper collision shape and define its mass and inertia.

```xml
<link name="base_link">
  <!-- The visual shape remains the same -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.4 0.2 0.1" />
    </geometry>
    <material name="blue"/>
  </visual>

  <!-- The collision shape for the physics engine -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.4 0.2 0.1" />
    </geometry>
  </collision>

  <!-- The inertial properties (mass and inertia tensor) -->
  <inertial>
    <mass value="5.0" /> <!-- The chassis weighs 5 kg -->
    <inertia ixx="0.05" ixy="0.0" ixz="0.0"
             iyy="0.1" iyz="0.0"
             izz="0.12" />
  </inertial>
</link>

<!-- We also need to define friction for the wheels! -->
<gazebo reference="left_wheel_link">
  <mu1>1.0</mu1>  <!-- Friction coefficient -->
  <mu2>1.0</mu2>
  <material>Gazebo/Grey</material>
</gazebo>
```
Calculating the inertia tensor (`ixx`, `iyy`, etc.) can be complex; CAD software often generates it for you. But even just setting a realistic mass is a huge step toward a more accurate simulation. We also added Gazebo-specific tags to define the friction of the wheels, which is crucial for realistic movement.

## Recap:
*   Physics engines (like ODE in Gazebo) simulate gravity, collisions, and other forces.
*   Accurate simulation requires defining physical properties in your URDF.
*   The `<collision>` tag defines the physical shape for collision detection.
*   The `<inertial>` tag defines the mass and rotational inertia of a link.
*   Properties like friction are added in special `<gazebo>` tags.

## Try It Yourself:
Imagine two robot arms. One is made of heavy steel, the other of light plastic. If you apply the same motor force to both, which one will move faster? How would you represent this difference in their URDF files? (Answer: The steel arm would have a much larger `<mass value="...">` in its `<inertial>` tag.)
