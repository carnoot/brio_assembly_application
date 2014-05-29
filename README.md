brio_assembly_application
=========================

- PoseStampet -> TrasformStamped
  - child_frame_id <- "brio_piece_frame"
  - frame_id <- "head_mount_kinect_rgb_optical_frame"

- New ROS package
  - brio_assembly_executive
  - depends on brio_assebly_vision
    - calls brio_assembly_vision services
  
  - publishes the TransformStamped on TF /tf
    - happes into a separate thread

  - waits for a transformation from "brio_piece_frame" into "base_link"
 
  - calls the service moving the robot arm
  
