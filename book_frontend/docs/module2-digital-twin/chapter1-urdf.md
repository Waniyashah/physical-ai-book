---
sidebar_position: 1
---

# Chapter 1: URDF for Robot Modeling

Welcome to Module 2, where we delve into the world of digital twins. Our journey begins with understanding how to describe a robot's physical structure and kinematics using the Unified Robot Description Format (URDF). URDF is an XML format used in ROS to describe all elements of a robot.

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML file format that describes a robot's physical and kinematic properties. It's used to represent a robot's structure, including its rigid bodies (links) and the connections between them (joints). This description is crucial for various robotics tasks, such as simulation, motion planning, and visualization.

A URDF file typically contains:
*   **Links**: Represent the rigid bodies of the robot (e.g., base, arm segments, end-effector). They have properties like mass, inertia, visual appearance, and collision geometry.
*   **Joints**: Represent the connections between links. They define the type of motion allowed (e.g., revolute, prismatic, fixed) and their limits.

## Structure of a URDF File

A basic URDF file starts with a `<robot>` tag, which contains multiple `<link>` and `<joint>` tags.

```xml
<robot name="my_robot">

  <!-- Link definitions -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint definitions -->
  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="100"/>
  </joint>

  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.02"/>
      </geometry>
    </visual>
  </link>

</robot>
```

In this example:
*   `base_link` and `arm_link` are two rigid bodies (links).
*   `base_to_arm_joint` is a revolute joint connecting `base_link` to `arm_link`. It rotates around the Z-axis.

## Key Elements in URDF

### `<link>` Tag

-   **`name`**: A unique identifier for the link.
-   **`<visual>`**: Defines the visual properties of the link, such as its geometry (box, cylinder, mesh) and color.
-   **`<collision>`**: Defines the collision properties, which are used by physics engines for collision detection.
-   **`<inertial>`**: Defines the mass and inertia tensor of the link, crucial for accurate physics simulation.

### `<joint>` Tag

-   **`name`**: A unique identifier for the joint.
-   **`type`**: Specifies the type of joint. Common types include:
    *   `revolute`: A rotational joint with a single axis of rotation and limits.
    *   `continuous`: A rotational joint with a single axis of rotation, no limits.
    *   `prismatic`: A linear joint with a single axis of translation and limits.
    *   `fixed`: A joint that rigidly connects two links, allowing no relative motion.
-   **`<parent link="PARENT_LINK_NAME"/>`**: Specifies the parent link of the joint.
-   **`<child link="CHILD_LINK_NAME"/>`**: Specifies the child link of the joint.
-   **`<origin xyz="X Y Z" rpy="ROLL PITCH YAW"/>`**: Defines the transform from the parent link's origin to the child link's origin. `xyz` specifies the translation and `rpy` (roll, pitch, yaw) specifies the rotation in radians.
-   **`<axis xyz="X Y Z"/>`**: Defines the axis of rotation for revolute/continuous joints or the axis of translation for prismatic joints.
-   **`<limit>`**: Defines the upper and lower limits, velocity limits, and effort limits for the joint.

In the next section, we will see a complete example of a simple robot arm described using URDF.
