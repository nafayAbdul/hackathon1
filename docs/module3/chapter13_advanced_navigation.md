---
sidebar_position: 4
---

# Chapter 13: Advanced Navigation & Manipulation for Bipedal Humanoids (Nav2 + MoveIt 2 + Isaac)

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate Nav2 and MoveIt 2 inside Isaac Sim for floating-base bipedal planning
- Implement SMAC planner with legged controller plugins for dynamic walking
- Create a complete manipulation pipeline: perception → grasp candidate → motion planning → execution
- Execute an end-to-end demo where "athena" walks to a table, detects a cup, and picks it up using only RGB-D data

## 13.1 Introduction to Bipedal Navigation and Manipulation

Bipedal humanoid navigation presents unique challenges compared to wheeled robots. Humanoid robots have complex kinematics, require balance maintenance during movement, and must navigate diverse terrains while maintaining dynamic stability. The combination of Nav2 for navigation and MoveIt 2 for manipulation in Isaac Sim provides a complete autonomous solution for bipedal robots.

Key challenges in bipedal navigation include:
- Maintaining balance during walking
- Planning over terrains with varying step heights
- Handling legged kinematics constraints
- Integrating perception with dynamic locomotion

## 13.2 Setting up Nav2 for Bipedal Humanoids

Nav2 in Isaac Sim requires special configuration for bipedal locomotion:

```yaml
# config/athena_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.1
    alpha2: 0.1
    alpha3: 0.1
    alpha4: 0.1
    alpha5: 0.1
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: False
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.5
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::FootprintModelType"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: True
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the custom behavior tree for bipedal navigation
    # This is important for humanoids due to balance constraints
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_cleared_condition_bt_node
    - nav2_not_initially_globalized_condition_bt_node
    - nav2_not_globalized_condition_bt_node
    - nav2_localized_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    # For bipedal robots, we need different velocity limits
    # due to balance constraints
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # DWB Controller for bipedal robots
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5  # Reduced for balance
      max_vel_y: 0.1
      max_vel_theta: 0.3
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.15
      stateful: True
      oscillation_reset_dist: 0.05
      forward_point_distance: 0.325
      stop_time_buffer: 0.2
      scaling_speed: 0.25
      max_scaling_factor: 0.2

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.3  # Bipedal robot radius
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: "scan"
        scan:
          topic: "/laser_scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: "scan"
        scan:
          topic: "/laser_scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "turtlebot3_world.yaml"

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      # Using SMAC planner which is better for legged robots
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.5  # Increased tolerance for bipedal robots
      downsample_costmap: false
      downsampling_factor: 1
      cost_estimate_factor: 1.0
      optimization: "thompson_sampling"
      max_samples: 50000
      max_tree_depth: 1000
      cache_size: 100
      allow_unknown: true
      max_planning_time: 5.0

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      w_smooth: 0.3
      w_data: 0.2

behavior_server:
  ros__parameters:
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/published_footprint"
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors::Spin"
      server_timeout: 10
      clearing_rotation_allowed: true
      enabled: true
    backup:
      plugin: "nav2_behaviors::BackUp"
      server_timeout: 10
      clearing_rotation_allowed: true
      enabled: true
      backup_dist: -0.15
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors::Wait"
      server_timeout: 10
      enabled: true
      wait_duration: 1.0
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
      server_timeout: 10
      clearing_rotation_allowed: true
      enabled: true
      max_speed_translation: 0.5
      max_speed_rotation: 0.4
      min_speed_translation: 0.0
      min_speed_rotation: 0.0
      speed_scaling_factor: 0.05
      translation_scaling_type: "angular"
      rotation_scaling_type: "angular"
```

## 13.3 Using SMAC Planner with Legged Controller Plugins

The SMAC (Search-based Motion Planner with A*) planner is particularly well-suited for humanoid navigation because it can handle the discrete nature of step planning. For bipedal robots, we need to modify the controller to account for balance constraints:

```python
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

class LeggedController(Node):
    def __init__(self):
        super().__init__('legged_controller')
        
        # Action server for follow path
        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            self.execute_callback)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        # State variables for bipedal control
        self.current_path = []
        self.path_index = 0
        self.footstep_positions = []  # For bipedal path following
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing follow path action...')
        path = goal_handle.request.path
        
        # Process path for bipedal navigation
        self.current_path = self.process_path_for_biped(path)
        self.path_index = 0
        
        # Execute path following with balance considerations
        result = self.follow_path()
        goal_handle.succeed()
        
        return result
    
    def process_path_for_biped(self, path_msg):
        # For bipedal robots, we need to consider footstep planning
        # Convert continuous path to discrete footsteps
        processed_path = []
        
        for pose in path_msg.poses:
            # Add balance constraints for each step
            footstep = self.calculate_footstep(pose)
            processed_path.append(footstep)
        
        return processed_path
    
    def calculate_footstep(self, pose):
        # Calculate appropriate footstep based on balance constraints
        # This considers the robot's center of mass and support polygon
        footstep = {
            'position': (pose.pose.position.x, pose.pose.position.y),
            'orientation': pose.pose.orientation,
            'support_type': 'double'  # Default to double support
        }
        return footstep
    
    def follow_path(self):
        # Implement path following with balance maintenance
        while self.path_index < len(self.current_path) and rclpy.ok():
            target_pose = self.current_path[self.path_index]
            
            # Generate velocity commands that maintain balance
            cmd_vel = self.calculate_balanced_velocity(target_pose)
            
            # Publish velocity command
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Check if we've reached the current waypoint
            if self.reached_waypoint(target_pose):
                self.path_index += 1
            
            # Small delay to let the action run
            self.get_clock().sleep_for(Duration(seconds=0.05))
        
        # Stop robot when path is complete
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        return FollowPath.Result()
    
    def calculate_balanced_velocity(self, target_pose):
        # Calculate velocities that maintain the robot's balance
        # This involves considering the Zero Moment Point (ZMP) and center of mass
        cmd = Twist()
        
        # Simple proportional controller considering balance
        dx = target_pose['position'][0] - self.get_current_x()
        dy = target_pose['position'][1] - self.get_current_y()
        
        # Normalize for bipedal speed constraints
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance > 0.1:  # Threshold to avoid oscillation
            cmd.linear.x = min(0.2, max(-0.2, 0.5 * dx))  # Limited speed for balance
            cmd.linear.y = min(0.1, max(-0.1, 0.5 * dy))  # Lateral movement limited
            cmd.angular.z = 0.3 * np.arctan2(dy, dx)  # Turn toward target
        
        return cmd
    
    def reached_waypoint(self, target_pose):
        # Check if robot is close enough to the target waypoint
        current_x = self.get_current_x()
        current_y = self.get_current_y()
        
        dx = target_pose['position'][0] - current_x
        dy = target_pose['position'][1] - current_y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Use larger tolerance for bipedal robots due to step discretization
        return distance < 0.3  # 30cm tolerance
    
    def get_current_x(self):
        # Get current x position from odometry
        # Implementation would subscribe to /odom
        return 0.0  # Placeholder
    
    def get_current_y(self):
        # Get current y position from odometry
        # Implementation would subscribe to /odom
        return 0.0  # Placeholder
    
    def control_loop(self):
        # Main control loop running at 20 Hz
        # Here we would implement balance control algorithms
        pass

def main(args=None):
    rclpy.init(args=args)
    node = LeggedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 13.4 Complete Manipulation Pipeline

Now let's implement the complete perception-to-manipulation pipeline:

```python
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class PerceptionManipulationPipeline(Node):
    def __init__(self):
        super().__init__('perception_manipulation_pipeline')
        
        # Initialize CV bridge for image processing
        self.cv_bridge = CvBridge()
        
        # Subscribe to RGB and depth images
        self.rgb_sub = self.create_subscription(
            Image, 'camera/color/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, 'camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/color/camera_info', self.camera_info_callback, 10)
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # MoveIt interfaces
        self.move_group = self.initialize_moveit_interface()
        
        # Internal state
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.camera_intrinsics = None
        
        # Object detection model
        self.object_detector = self.initialize_object_detector()
    
    def initialize_object_detector(self):
        # Initialize object detection model for cup detection
        # This could be a YOLO model or similar
        return None  # Placeholder
    
    def initialize_moveit_interface(self):
        # Initialize MoveIt interface for the athena arm
        # This would typically use moveit_commander in Python
        try:
            import moveit_commander
            # Initialize moveit_commander for athena robot
            robot = moveit_commander.RobotCommander()
            scene = moveit_commander.PlanningSceneInterface()
            
            # Set up move group for the arm
            move_group = moveit_commander.MoveGroupCommander("arm")
            
            return move_group
        except ImportError:
            self.get_logger().error("moveit_commander not available")
            return None
    
    def rgb_callback(self, msg):
        self.latest_rgb_image = msg
    
    def depth_callback(self, msg):
        self.latest_depth_image = msg
    
    def camera_info_callback(self, msg):
        # Store camera intrinsics for 3D reconstruction
        self.camera_intrinsics = {
            'fx': msg.k[0],  # Focal length x
            'fy': msg.k[4],  # Focal length y
            'cx': msg.k[2],  # Principal point x
            'cy': msg.k[5]   # Principal point y
        }
    
    def detect_object(self, image_cv):
        # Detect cup in the image using hardware-accelerated perception
        # Using Isaac ROS foundation models
        if self.object_detector is None:
            # For now, use a simple color-based detection as placeholder
            # In practice, this would use Isaac ROS perception models
            hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
            
            # Define range for cup color (e.g., red cup)
            lower_red = np.array([0, 50, 50])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            
            lower_red = np.array([170, 50, 50])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)
            
            mask = mask1 + mask2
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                # Find the largest contour (likely the cup)
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 500:  # Minimum size threshold
                    # Get the center of the contour
                    m = cv2.moments(largest_contour)
                    if m["m00"] != 0:
                        cx = int(m["m10"] / m["m00"])
                        cy = int(m["m01"] / m["m00"])
                        return (cx, cy)  # Pixel coordinates of cup center
        
        return None
    
    def get_object_3d_position(self, pixel_coords, depth_image_cv):
        # Convert 2D pixel coordinates to 3D world coordinates
        if self.camera_intrinsics is None:
            return None
            
        cx, cy = pixel_coords
        depth_value = depth_image_cv[cy, cx]
        
        if depth_value == 0:
            # Invalid depth, try neighboring pixels
            for dy in range(-5, 6):
                for dx in range(-5, 6):
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < depth_image_cv.shape[0] and 0 <= nx < depth_image_cv.shape[1]:
                        depth_val = depth_image_cv[ny, nx]
                        if depth_val != 0:
                            depth_value = depth_val
                            break
                if depth_value != 0:
                    break
        
        if depth_value == 0:
            return None  # Could not determine depth
        
        # Convert pixel to 3D using camera intrinsics
        x = (cx - self.camera_intrinsics['cx']) * depth_value / self.camera_intrinsics['fx']
        y = (cy - self.camera_intrinsics['cy']) * depth_value / self.camera_intrinsics['fy']
        z = depth_value
        
        # Create point in camera frame
        point_camera = Point(x=x, y=y, z=z)
        
        # Transform to robot base frame
        try:
            # Create PoseStamped message for transformation
            pose_camera = PoseStamped()
            pose_camera.header.frame_id = "camera_color_optical_frame"
            pose_camera.header.stamp = self.get_clock().now().to_msg()
            pose_camera.pose.position = point_camera
            pose_camera.pose.orientation.w = 1.0
            
            # Transform to robot base frame
            transform = self.tf_buffer.lookup_transform(
                "base_link", "camera_color_optical_frame",
                rclpy.time.Time(),  # Use latest available transform
                rclpy.duration.Duration(seconds=1.0)
            )
            
            point_base = tf2_geometry_msgs.do_transform_pose(pose_camera, transform)
            return point_base.pose.position
            
        except Exception as e:
            self.get_logger().error(f"Could not transform point: {e}")
            return None
    
    def plan_grasp(self, object_position):
        # Plan grasp trajectory for the detected object
        if self.move_group is None:
            return False
            
        # Set target pose for grasping
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position the gripper slightly above the object
        target_pose.pose.position.x = object_position.x
        target_pose.pose.position.y = object_position.y
        target_pose.pose.position.z = object_position.z + 0.1  # 10cm above object
        
        # Orient gripper to approach from above
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.707  # Looking down
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.707
        
        # Plan the motion
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()
        
        if plan[0]:  # Plan successful
            return True
        else:
            self.get_logger().error("Could not plan grasp trajectory")
            return False
    
    def execute_pick_and_place(self, object_position):
        # Execute complete pick and place operation
        if self.move_group is None:
            return False
            
        try:
            # Plan approach trajectory
            if not self.plan_grasp(object_position):
                return False
            
            # Execute the planned motion
            self.move_group.execute(self.move_group.plan()[1], wait=True)
            
            # Close gripper to grasp the object
            # This would involve commanding the gripper
            # gripper_command_publisher.publish(gripper_close_msg)
            
            # Lift the object
            current_pose = self.move_group.get_current_pose().pose
            lift_pose = PoseStamped()
            lift_pose.header.frame_id = "base_link"
            lift_pose.pose = current_pose
            lift_pose.pose.position.z += 0.1  # Lift 10cm
            
            self.move_group.set_pose_target(lift_pose)
            lift_plan = self.move_group.plan()
            
            if lift_plan[0]:
                self.move_group.execute(lift_plan[1], wait=True)
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error in pick and place: {e}")
            return False
    
    def execute_perception_pipeline(self):
        # Main execution function for the complete pipeline
        if self.latest_rgb_image is None or self.latest_depth_image is None:
            self.get_logger().warn("Waiting for camera data...")
            return False
        
        # Convert ROS images to OpenCV format
        try:
            rgb_cv = self.cv_bridge.imgmsg_to_cv2(self.latest_rgb_image, "bgr8")
            depth_cv = self.cv_bridge.imgmsg_to_cv2(self.latest_depth_image, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Error converting images: {e}")
            return False
        
        # Detect object in the RGB image
        object_pixel = self.detect_object(rgb_cv)
        if object_pixel is None:
            self.get_logger().info("No object detected")
            return False
        
        # Convert 2D detection to 3D world coordinates
        object_3d = self.get_object_3d_position(object_pixel, depth_cv)
        if object_3d is None:
            self.get_logger().error("Could not determine 3D position of object")
            return False
        
        self.get_logger().info(f"Object detected at: ({object_3d.x:.2f}, {object_3d.y:.2f}, {object_3d.z:.2f})")
        
        # Execute pick and place operation
        success = self.execute_pick_and_place(object_3d)
        
        if success:
            self.get_logger().info("Pick and place operation completed successfully")
            return True
        else:
            self.get_logger().error("Pick and place operation failed")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionManipulationPipeline()
    
    # For demonstration, run the pipeline once every 5 seconds
    timer = node.create_timer(5.0, node.execute_perception_pipeline)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## 13.5 End-to-End Demo: Athena Table-Cup Task

Now let's combine navigation and manipulation for the complete demo:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class AthenaNavigationManipulationDemo(Node):
    def __init__(self):
        super().__init__('athena_demo')
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher for demo status
        self.status_pub = self.create_publisher(String, 'demo_status', 10)
        
        # Initialize perception and manipulation pipeline
        self.perception_manipulation = PerceptionManipulationPipeline()
        
        # Demo timer
        self.demo_timer = self.create_timer(10.0, self.run_demo_step)
        self.demo_state = "navigate_to_table"  # State machine for the demo
        self.table_position = self.get_table_position()  # Predefined table location
        
    def get_table_position(self):
        # Return known table position in the environment
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 2.0  # Example coordinates
        pose.pose.position.y = 1.5
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose
    
    def run_demo_step(self):
        if self.demo_state == "navigate_to_table":
            self.navigate_to_table()
        elif self.demo_state == "detect_and_pickup":
            self.detect_and_pickup_object()
        elif self.demo_state == "return_to_base":
            self.return_to_base()
        elif self.demo_state == "complete":
            self.demo_complete()
    
    def navigate_to_table(self):
        self.get_logger().info("Navigating to table...")
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation server not available")
            return
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.table_position
        
        # Send navigation goal
        self.nav_future = self.nav_client.send_goal_async(goal_msg)
        self.nav_future.add_done_callback(self.navigation_done_callback)
        
        self.demo_state = "waiting_for_navigation"
    
    def navigation_done_callback(self, future):
        result = future.result()
        if result.status == 3:  # SUCCEEDED
            self.get_logger().info("Reached table location")
            self.demo_state = "detect_and_pickup"
            self.status_pub.publish(String(data="At table, starting manipulation"))
        else:
            self.get_logger().error("Navigation failed, stopping demo")
            self.demo_state = "complete"
    
    def detect_and_pickup_object(self):
        self.get_logger().info("Detecting and picking up object...")
        
        # Execute perception and manipulation pipeline
        success = self.perception_manipulation.execute_perception_pipeline()
        
        if success:
            self.get_logger().info("Successfully picked up object")
            self.demo_state = "return_to_base"
            self.status_pub.publish(String(data="Picked up object, returning"))
        else:
            self.get_logger().error("Failed to pick up object")
            self.demo_state = "complete"
    
    def return_to_base(self):
        self.get_logger().info("Returning to base...")
        
        # Create goal to return to start position
        base_pose = PoseStamped()
        base_pose.header.frame_id = "map"
        base_pose.pose.position.x = 0.0
        base_pose.pose.position.y = 0.0
        base_pose.pose.position.z = 0.0
        base_pose.pose.orientation.w = 1.0
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation server not available")
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = base_pose
        
        self.nav_future = self.nav_client.send_goal_async(goal_msg)
        self.nav_future.add_done_callback(self.return_done_callback)
    
    def return_done_callback(self, future):
        result = future.result()
        if result.status == 3:  # SUCCEEDED
            self.get_logger().info("Returned to base")
            self.demo_state = "complete"
            self.status_pub.publish(String(data="Demo completed successfully"))
        else:
            self.get_logger().error("Return navigation failed")
            self.demo_state = "complete"
    
    def demo_complete(self):
        self.get_logger().info("Demo completed")
        self.demo_timer.cancel()  # Stop the demo timer

def main(args=None):
    rclpy.init(args=args)
    node = AthenaNavigationManipulationDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## 13.6 Chapter Summary

In this chapter, we've implemented a complete navigation and manipulation pipeline for the "athena" humanoid robot using Nav2 for navigation and MoveIt 2 for manipulation, all integrated within Isaac Sim. We used the SMAC planner for better bipedal path planning and implemented a full perception-to-action pipeline that allows "athena" to navigate to a location, detect an object using RGB-D data, and perform manipulation tasks.

## End-of-Chapter Exercises

1. Configure Nav2 for the "athena" humanoid robot in Isaac Sim
2. Implement the SMAC planner with appropriate parameters for bipedal navigation
3. Create a complete pick-and-place pipeline using MoveIt 2
4. Execute the end-to-end demo where "athena" navigates to a table, detects a cup, and picks it up