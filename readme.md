## Aim
The manual tf calibration package. Helps approximate tf among various sensors.
For usage, follow roslaunch in serial order.

---

## Steps

### 1 :
Run `roslaunch 1_accumulate_pts.launch` to gather multiple lidar scans into one
big pointloucd and store it as a pcd file.

### 2 :
Using `CloudCompare`, rotate and crop the pointcloud data to required space.
Thereafter, note down approximate values of each sensor's tf in absolute frame.

### 3 :
Key in the approximated tf values in and run `3_static_tf.launch`, which
publishes the pointcloud and respective sensor tf via ROS.
Next, run `rviz -d tf_viz.rviz` to visualize the actual 3D poses of the sensor
along with pointcloud data.

### 4 :
Tune the values back in `3_static_tf.launch`, to ensure that the tf looks good.
Once the values are fixed, run `rosrun tf tf_echo <parent_frame> <child_frame>`
to find the relative tf between desired parent and child frames.
For re-check, user may key in the values in and run `4_static_tf_rel.launch` to
ensure the computed values are actually accurate.
