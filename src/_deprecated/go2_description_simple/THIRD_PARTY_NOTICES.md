# Third-party notices

The URDF kinematic layout, physical constants and CAD mesh files under
`meshes/` in this package describe the **Unitree Go2** quadruped robot.

- Source of the model data: the public, open-source Unitree Go2 ROS 2
  descriptions, e.g. the `unitree_go2_description` package in
  https://github.com/khaledgabr77/unitree_go2_ros2 (and related repos in the
  Unitree ecosystem). Those projects in turn mirror the robot description
  shipped by Unitree Robotics with the Go2.
- The `*.dae` mesh files are CAD geometry of the Unitree Go2 hardware.
  Copyright in the robot design belongs to Unitree Robotics. This workspace
  redistributes them only to reproduce the robot for simulation/study.
- All ROS 2 package sources (xacro, launch, C++) written for this workspace
  are original and licensed Apache-2.0 unless stated otherwise.

> If you publish this workspace to a public repository, confirm you are
> permitted to redistribute the mesh assets under your chosen license. To
> avoid redistributing the CAD meshes entirely, rebuild with
> `use_meshes:=false`, which uses plain primitive geometry instead.
