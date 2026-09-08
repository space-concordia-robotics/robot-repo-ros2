# Summary
This package loads a rover URDF in RVIZ 

Description: Takes raw Solidworks-URDF exports so that they can simply be dragged and dropped in, applies an un-changing color scheme and fixes references to wrong directories. Then it automatically loads a URDF of your choice and launches RVIZ

To execute: Run the ROS2 Launcher :  ros2 launch urdf_viewer_pkg urdf_viewer_launch.xml

How to upload new URDF's: 

Upload URDF folders into the URDF_Imports folder
** URDF_Iports/Intro_URDF shows the ideal directory organization. 
--If the .URDF file is not in a folder it should still work.
--Meshes should be contained in a mesh folder


Other: This is based on the inverse_kinematics rover_arm_description visualizer. 


