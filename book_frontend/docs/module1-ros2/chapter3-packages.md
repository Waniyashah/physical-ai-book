---
sidebar_position: 3
---

# Chapter 3: Building a ROS 2 Package

In the previous chapters, we learned about the fundamental communication mechanisms in ROS 2. Now, it's time to put that knowledge into practice by creating our own ROS 2 **package**. A package is the primary unit of software organization in ROS 2. It contains your nodes, along with other files like launch files, configuration files, and message definitions.

## Creating a ROS 2 Package

To create a ROS 2 package, we use the `ros2 pkg create` command. This command generates a directory with the necessary files and folder structure for a ROS 2 package.

For example, to create a Python package named `my_robot_pkg`, you would run the following command:

```bash
ros2 pkg create --build-type ament_python my_robot_pkg
```

This will create a `my_robot_pkg` directory with the following structure:

```
my_robot_pkg/
├── package.xml
├── setup.py
├── setup.cfg
└── resource/
    └── my_robot_pkg
└── my_robot_pkg/
    └── __init__.py
```

### `package.xml`

The `package.xml` file is the manifest file for your package. It contains important metadata about the package, such as its name, version, author, and dependencies.

Here is an example `package.xml` file:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.0</version>
  <description>A simple ROS 2 package.</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### `setup.py`

The `setup.py` file is the build script for your Python package. It tells ROS 2 how to build and install your package, including where to find your nodes.

Here is an example `setup.py` file:

```python
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='A simple ROS 2 package.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_robot_pkg.talker:main',
            'listener = my_robot_pkg.listener:main',
        ],
    },
)
```

## Building a ROS 2 Package

To build your ROS 2 package, you use the `colcon build` command from the root of your workspace.

```bash
colcon build
```

This will build your package and install it into the `install` directory of your workspace. To use your new package, you need to source the `setup.bash` file in the `install` directory:

```bash
source install/setup.bash
```

Now you can run your nodes using the `ros2 run` command:

```bash
ros2 run my_robot_pkg talker
```

```bash
ros2 run my_robot_pkg listener
```

Congratulations! You have successfully created and built your own ROS 2 package. In the next module, we will learn how to create a digital twin of our robot and simulate it in a virtual environment.
