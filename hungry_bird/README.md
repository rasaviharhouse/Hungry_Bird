# Hungry Bird
This ROS package involves the following:
- **marker_detect.launch**
  - Runs **whycon** node
  - Runs **aruco_ros** node
  - Runs **image_view** node to display WhyCon and ArUco output
  
  
- **get_marker_data.py**
  - ROS node which subscribes to */whycon/poses* & */aruco_marker_publisher/markers* 
  - Displays the position of WhyCon marker and orientation of ArUco marker in the V-REP scene
  

