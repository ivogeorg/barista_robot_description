### barista_robot_description

| Real | Diagram | Model|
| --- | --- | --- |
| ![Real robot](assets/rick_and_morty.jpg) | ![Barista robot](assets/robot_diagram.png) | ![Model](assets/laser-offset.png) |

#### Implementation notes

##### 1. Handling the standoff rods

1. The robot model has a potential cycle in the design of the standoff rods. Since all rods are to attach to both `base_link` and `cup_holder`, if each rod is a separate link, `cup_holder` will have 4 joints with 4 different parents. This is a cycle and therefore not viable. Below is the result of visualizing such a model in Rviz2:  
   ![Multiple parents rejected](assets/multiple_parents_rejected.png)  
2. The `robot_state_publisher-1` parses the structure correctly, but all but the last joint with `cup_holder` as a child is ignored to preserve the tree structure:  
   ```
   user:~/ros2_ws$ ros2 launch barista_robot_description barista_urdf_separate_rods.launch.py
   [INFO] [launch]: All log files can be found below /home/user/.ros/log/2024-07-20-19-41-12-151468-1_xterm-14068
   [INFO] [launch]: Default logging verbosity is set to INFO
   Fetching URDF ==>
   [INFO] [robot_state_publisher-1]: process started with pid [14070]
   [INFO] [rviz2-2]: process started with pid [14072]
   [rviz2-2] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-user'
   [robot_state_publisher-1] Link base_link had 4 children
   [robot_state_publisher-1] Link standoff_rod_1 had 1 children
   [robot_state_publisher-1] Link cup_holder had 0 children
   [robot_state_publisher-1] Link standoff_rod_2 had 1 children
   [robot_state_publisher-1] Link cup_holder had 0 children
   [robot_state_publisher-1] Link standoff_rod_3 had 1 children
   [robot_state_publisher-1] Link cup_holder had 0 children
   [robot_state_publisher-1] Link standoff_rod_4 had 1 children
   [robot_state_publisher-1] Link cup_holder had 0 children
   [robot_state_publisher-1] [INFO] [1721504472.377891927] [robot_state_publisher_node]: got segment base_footprint
   [robot_state_publisher-1] [INFO] [1721504472.377988506] [robot_state_publisher_node]: got segment base_link
   [robot_state_publisher-1] [INFO] [1721504472.378003157] [robot_state_publisher_node]: got segment cup_holder
   [robot_state_publisher-1] [INFO] [1721504472.378015203] [robot_state_publisher_node]: got segment standoff_rod_1
   [robot_state_publisher-1] [INFO] [1721504472.378026624] [robot_state_publisher_node]: got segment standoff_rod_2
   [robot_state_publisher-1] [INFO] [1721504472.378037824] [robot_state_publisher_node]: got segment standoff_rod_3
   [robot_state_publisher-1] [INFO] [1721504472.378048920] [robot_state_publisher_node]: got segment standoff_rod_4
   ```
3. Assembling all 4 rods in the same link works fine (at least in URDF):
   ```
       <link name="standoff_rods">
        <visual>
            <origin rpy="0 0 0" xyz="0.050 0.140 0"/>
            <geometry>
                <cylinder length="0.220" radius="0.010"/>
            </geometry>
            <material name="light_gray"/>
        </visual>

        <visual>
            <origin rpy="0 0 0" xyz="-0.050 0.140 0"/>
            <geometry>
                <cylinder length="0.220" radius="0.010"/>
            </geometry>
            <material name="light_gray"/>
        </visual>

        <visual>
            <origin rpy="0 0 0" xyz="0.050 -0.140 0"/>
            <geometry>
                <cylinder length="0.220" radius="0.010"/>
            </geometry>
            <material name="light_gray"/>
        </visual>

        <visual>
            <origin rpy="0 0 0" xyz="-0.050 -0.140 0"/>
            <geometry>
                <cylinder length="0.220" radius="0.010"/>
            </geometry>
            <material name="light_gray"/>
        </visual>

        <!-- collision -->
        <collision>
            <origin rpy="0 0 0" xyz="0.050 0.140 0"/>
            <geometry>
                <cylinder length="0.220" radius="0.010"/>
            </geometry>
        </collision>

        <collision>
            <origin rpy="0 0 0" xyz="-0.050 0.140 0"/>
            <geometry>
                <cylinder length="0.220" radius="0.010"/>
            </geometry>
        </collision>

        <collision>
            <origin rpy="0 0 0" xyz="0.050 -0.140 0"/>
            <geometry>
                <cylinder length="0.220" radius="0.010"/>
            </geometry>
        </collision>

        <collision>
            <origin rpy="0 0 0" xyz="-0.050 -0.140 0"/>
            <geometry>
                <cylinder length="0.220" radius="0.010"/>
            </geometry>
            <material name="light_gray"/>
        </collision>

        <!-- approximate a very light box with rods on the edges -->
        <inertial>
            <mass value="0.02"/>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <inertia ixx="0.0002113333333333334" ixy="0.0" ixz="0.0" iyy="9.733333333333335e-05" iyz="0.0" izz="0.00014733333333333335"/>
        </inertial>

    </link>

    <joint name="joint_base_link_standoff_rods" type="fixed">
        <origin rpy="0 0 0" xyz="0 0 0.1875"/>
        <parent link="base_link"/>
        <child link="standoff_rods"/>
        <axis xyz="0 0 0"/>
    </joint>
   ```
4. The only problem is that the inertia has to be approximated. In this case, it is approximated by a light box formed by the rods, with the default inertial center of the assembly link falling in between them.

##### 2. Laser scanner setup

1. Picked the [Hokuyo URG-04LX mesh](https://bitbucket.org/theconstructcore/checkpoint4_auxiliary_files/src/master/meshes/hokuyo_urg_04lx.dae). Note that this scanner is not 360-deg.
2. Mass and dimensions for `inertial` read off the [spec sheet](https://www.hokuyo-aut.jp/dl/Specifications_URG-04LX_1513063395.pdf).
   1. Mass: 160 g.
   2. Dimensions: 50 x 50 x 70 mm.
   