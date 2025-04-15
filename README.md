# Realsense Tutorial

This is a repository for all about realsense camera series (D435, D435i, D455 and D457).

Install the libraries:
```bash
# Install librealsense2
sudo apt install ros-humble-librealsense2*

# Install debian realsense2 package
sudo apt install ros-humble-realsense2-*
```
Launch the ROS2 node:
```bash
source /opt/ros/humble/setup.bash 
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true
```
<div align="center">
  <img src="media/rs_rviz.png" width="600">
</div>

## ROS2 topics:
### Intel Realsense D457
List all topics:
```
arghya@arghya:~$ ros2 topic list
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/image_raw/compressed
/camera/camera/aligned_depth_to_color/image_raw/compressedDepth
/camera/camera/aligned_depth_to_color/image_raw/ffmpeg
/camera/camera/aligned_depth_to_color/image_raw/theora
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/image_raw/compressed
/camera/camera/color/image_raw/compressedDepth
/camera/camera/color/image_raw/ffmpeg
/camera/camera/color/image_raw/theora
/camera/camera/color/metadata
/camera/camera/depth/camera_info
/camera/camera/depth/color/points
/camera/camera/depth/image_rect_raw
/camera/camera/depth/image_rect_raw/compressed
/camera/camera/depth/image_rect_raw/compressedDepth
/camera/camera/depth/image_rect_raw/ffmpeg
/camera/camera/depth/image_rect_raw/theora
/camera/camera/depth/metadata
/camera/camera/extrinsics/depth_to_color
/camera/camera/extrinsics/depth_to_depth
/camera/camera/rgbd
/clicked_point
/goal_pose
/initialpose
/parameter_events
/rosout
/tf
/tf_static
```
Echo `/camera/camera/aligned_depth_to_color/image_raw`:

```
arghya@arghya:~$ ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw 
header:
  stamp:
    sec: 1744740840
    nanosec: 995971924
  frame_id: camera_color_optical_frame
height: 480
width: 640
encoding: 16UC1
is_bigendian: 0
step: 1280
data:
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- '...'
---



