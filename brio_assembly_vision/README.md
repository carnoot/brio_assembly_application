brio_assembly_application
=========================

  - waits for a transformation from "brio_piece_frame" relative to "base_link"
   - in the main thread
 
  - calls the service moving the robot arm

    - two_hand_ik_trajectory_executor
      - in package.xml
           CMakeLists.txt
      - keep them commented out

    - ├── ExecuteBothArmsCartesianIKTrajectory.srv
      ├── ExecuteLeftArmCartesianIKTrajectory.srv
      └── ExecuteRightArmCartesianIKTrajectory.srv

    - two_hand_ik_trajectory_executor/ExecuteBothArmsCartesianIKTrajectory

    - two_hand_ik_trajectory_executor/ExecuteLeftArmCartesianIKTrajectory

       std_msgs/Header header
         uint32 seq
         time stamp
         string frame_id 
       geometry_msgs/Pose[] poses
         geometry_msgs/Point  
           float64 x
           float64 y
           float64 z
         geometry_msgs/Quaternion orientation
           float64 x
           float64 y
           float64 z
           float64 w
       ---
       uint32 success

    - two_hand_ik_trajectory_executor/ExecuteRightArmCartesianIKTrajectory

    - frame_id  <- "base_link"
    - position && orientation
      - are "brio_piece_frame" relative to the "base_link" from TF
