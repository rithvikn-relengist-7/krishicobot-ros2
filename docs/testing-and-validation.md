# Testing and validation

## Checks completed for this repository draft

- Python source files pass a syntax compilation check (`compileall`).
- Package metadata and entry-point structure have been added.
- README commands and expected topic interfaces are documented.

These checks do not prove that the ROS nodes run correctly in a ROS graph.

## Runtime validation still required

### Navigation node
- [ ] Confirm `/scan` publishes `sensor_msgs/LaserScan`.
- [ ] Confirm `/odom` publishes `nav_msgs/Odometry`.
- [ ] Confirm the drive controller accepts `/cmd_vel`.
- [ ] Replace example waypoints and verify coordinate frame/map origin.
- [ ] Test obstacle detection and clearing behaviour in simulation.
- [ ] Verify stopping behaviour and velocity limits on the actual base.

### RGB-D detector
- [ ] Confirm RGB and depth topic encodings.
- [ ] Confirm depth values are in the units assumed by the node.
- [ ] Verify RGB/depth alignment.
- [ ] Verify camera intrinsics against the active camera.
- [ ] Verify camera-to-target TF exists and has the correct direction.
- [ ] Tune HSV thresholds against representative lighting and fruit images.
- [ ] Evaluate false positives and missed detections.

### UR5 controller
- [ ] Confirm `base_link` and `ee_link` are correct for the robot.
- [ ] Confirm waypoint poses and quaternion values in the robot's frame.
- [ ] Confirm the driver's `/delta_twist_cmds` message contract.
- [ ] Verify workspace limits and emergency stop with a supervisor.
- [ ] Keep `enable_motion:=false` until all checks are complete.
- [ ] Conduct staged simulation and supervised hardware tests.

## Reporting results

Only add accuracy, latency, navigation success rate, repeatability or hardware-demonstration claims after measuring them. Record the test setup, ROS distribution, robot configuration and number of trials.
