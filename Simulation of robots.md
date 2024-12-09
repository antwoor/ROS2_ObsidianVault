# URDF

#### urdf - ist e tree-like structure of robot with ***one root***
![[tf_tree.svg]]
## tf_tree example
```xml
<?xml version="1.0"?>

<robot name="tf_tree">
    
    <link name="base_link"/>
    <link name="link_1"/>
    <link name="link_2"/>
    <link name="gripper"/>
    <!-- <link name="finger_1"/>
    <link name="finger_2"/> -->

    <joint name="base_joint" type="continuous">
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <parent link="base_link"/>
        <child link="link_1"/>
        <axis xyz="0.0 0.0 1.0"/>
        <!-- <limit lower="-3.14" upper="3.14" effort="1.0" velocity="0.0"/> -->
    </joint>

    <joint name="joint_1" type="revolute">
        <origin xyz="0.0 0.0 1.48" rpy="0.0 0.0 0.0"/>
        <parent link="link_1"/>
        <child link="link_2"/>
        <axis xyz="0.0 1.0 0.0"/>
        <limit lower="-3.14" upper="3.14" effort="1.0" velocity="0.0"/>
    </joint>

    <joint name="joint_2" type="revolute">
        <origin xyz="0.0 0.0 1.48" rpy="0.0 0.0 0.0"/>
        <parent link="link_2"/>
        <child link="gripper"/>
        <axis xyz="0.0 0.0 1.0"/>
        <limit lower="-3.14" upper="3.14" effort="1.0" velocity="0.0"/>
    </joint>
</robot>
```

## Structure of link
```xml
    <link name="link_1">

        <visual>
            <origin xyz="0.0 0.0 0.18" rpy="0.0 0.0 0.0"/>
            <geometry>
                <cylinder radius="0.06" length="0.3"/>
            </geometry>
            <material name="blue">
                <color rgba="0.0 0.0 1.0 1.0"/>
            </material>
        </visual>

        <collision>
            <geometry>
                <cylinder radius="0.06" length="0.3"/>
            </geometry>
        </collision>

        <inertial>
            <mass value="1.0"/>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <inertia ixx="0.03" ixy="0.0" ixz="0.0" 
            iyy="0.03" iyz="0.0" izz="0.03"/>
        </inertial>

    </link>
```

## Structure of joint
```xml
    <joint name="joint_4" type="prismatic">
        <origin xyz="-0.05 0.0 0.01" rpy="0.0 0.0 0.0"/>
        <parent link="gripper"/>
        <child link="finger_2"/>
        <axis xyz="1.0 0.0 0.0"/>
        <limit lower="-0.5" upper="0.01" effort="1.0" velocity="5.0"/>
    </joint>
```

# XACRO
#### There are also a special scripts called xacro. With xacro you can wrap lots of links and joint to one element and import it to other xacro.
## XACRO examples
### manipulator.xacro
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="manipulator">
    <!-- Включаем описание манипулятора -->
    <!-- Манипулятор из вашего файла -->
    <xacro:include filename="static_robot.urdf"/>
    <!-- <xacro:manipulator base_link="base_link"/> -->
</robot>
```
### mobile_robot.xacro
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mobile_robot">

<xacro:macro name="mobile_robot" params="base_link">
  <link name="${base_link}">
    <visual>
      <geometry>
        <box size="1.0 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="1.0 0.5 0.2"/>
      </geometry>
    </collision>
  </link>
  <!-- #Description
  ... #Description
  .. #Description
  .. #Description
  -->
  </xacro:macro>
</robot>
```
### combined robot 
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="combined_robot">

    <!-- Включение мобильной платформы -->
    <xacro:include filename="mobile_robot.xacro"/>
    <xacro:mobile_robot base_link="chassis"/>
    <joint name="connector_joint" type="fixed">
        <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
        <parent link="chassis"/>
        <child link="base_link"/>
        <axis xyz="0.0 0.0 1.0"/>
    </joint>
    <!-- Включение манипулятора -->
    <xacro:include filename="manipulator.xacro"/>
    <xacro:manipulator/>

</robot>

```
# Gazebo

# Pybullet

# Mujoco
