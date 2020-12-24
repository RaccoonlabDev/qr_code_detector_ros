# qr_code_detector_ros
The Ros package to get QR code data and position in world.
Written on Python Opencv + zbar + PnP + blur detector.

![IMAGE ALT TEXT](https://habrastorage.org/getpro/habr/upload_files/1d6/c10/759/1d6c107598726b0e59e1aa175a7c58c8)

## Install
Need install lib: http://zbar.sourceforge.net/

**Run:**
```bash
roslaunch qr_pose_estimation_ros qr_pose_estimation.lanch 
```

#### Subscribed Topics:

image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Raw image stream from the camera driver.<br/>
camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)): Camera metadata.<br/>

#### Publisher Topics:

find_qr[qr_pose_estimation_ros_msgs/QrCode](): Position and data of the found object relative "map_id"<br/>
tf2 ([geometry_msgs/TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html)): Tf2 pose of find object<br/> 

#### Parameters:
~rate (float, default: 10)<br/>
&emsp;&emsp;*Frame rate of node. If rate <= 0, work without delay<br/>*
~blur_threshold (int, default: 300)<br/>
&emsp;&emsp;*Filter of blur detector. The higher the value, the more sensitive the filter.<br/>*
~camera_name (string, default: "camera")<br/>
&emsp;&emsp;*Name of camera.<br/>*
~frame_id (string, default: "object")<br/>
&emsp;&emsp;*Name of find object.<br/>*
~map_id (string, default: "map")<br/>
&emsp;&emsp;*frame id relative to which pose_stamped is published .<br/>*
~size_image (float, default: 0.1)<br/>
&emsp;&emsp;*The width of the image in meters.<br/>*
~show_image (bool, default: true)<br/>
&emsp;&emsp;*Uses a known image.<br/>*
~max_dist (float, default: "5.0" )<br/>
&emsp;&emsp;*Maximum distance (meter) of find object (need for filter).<br/>*
