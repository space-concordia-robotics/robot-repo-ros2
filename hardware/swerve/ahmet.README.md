# 1. T1: Launch the Swerve Sim
```
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch swerve_simulation sim.launch.py
```                                                                                                                 
# 2. Terminal 2: Start Teleop
                                                                                                                                                             
##  Keyboard Teleop
    
    source /opt/ros/jazzy/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -p stamped:=true -r cmd_vel:=/swerve_controller/cmd_vel

# Full report
code ~/.gemini/antigravity-cli/brain/0cc9754d-1eb6-4586-afee-35b2ebe5b835/swerve_documentation_report.md