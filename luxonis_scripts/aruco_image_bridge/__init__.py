"""
Low-rate Luxonis image + CameraInfo bridge for the ArUco tracker.

The camera can only be opened by one process, so the ROS output is added to main.py's own
DepthAI pipeline (``main.py --ros``) instead of running as a separate node.

Kept empty on purpose: importing the package must not import ROS, DepthAI or
OpenCV, so main.py keeps working for people who only use the RTSP streams.
"""
