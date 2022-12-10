# Example Run
[This link](https://youtu.be/5s21_tOiuP8) contains a video of an early run of this package. Improvements have been made since, specifically for product placement on the AGV and arm planning.

# Coordinate Issues

This package was designed in an environment that seems to be failing every coordinate transform. All through the main c++ file there are functions that attempt to fix this issue.

The main method of fixing the problem is a coordinate remap for the values that are off.
```
double remapValues(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
```
These values were determined by looking at the tf transform output and the robot destination coordinate at the corners of the bin and agv stages.

# Package Information
## Dependencies:

This package depends on the ARIAC 2019 competition environment setup located [here](https://bitbucket.org/osrf/ariac/wiki/2019/Home). Please create a similar workspace directory with the packages relevant packages and tools installed.

## Installation:

### Ros installation:

Make sure that ros noetic was installed into /opt following the installation procedure listed in the [wikis](http://wiki.ros.org/noetic/Installation/Ubuntu).

Then proceed to source ROS:

```
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### Package pre-requirements:

### Installing the package:

Open a terminal in the `/ariac_ws/src` directory (or the work space setup for the 2019 competition). Then perform the following commands:

```
git clone git@github.com:cwru-courses/csds373-f21-t2-ariac.git
```

Then, all the packages can be built after navigating to `ariac_ws` and making:

```
catkin_make
```

```
git clone https://github.com/cwru-eecs-373/cwru_ariac_2019.git
# Install any missing dependencies
rosdep install --from-paths ariac --ignore-src -r -y

git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git
# Install any missing dependencies.
rosdep install --from-paths ecse_373_ariac --ignore-src -r -y
```

# Running this package:

## Launch the competition environment:

```
roslaunch cwru_ecse_373_submission compete.launch
```

## Launch the code that attempts the competition:

```
roslaunch cwru_ecse_373_submission competition.launch
```

# Potential Build Issues

If building gives you an error like the following:

```
[ 50%] Linking CXX executable /ariac_ws/devel/lib/cwru_ecse_373_submission/compete
/usr/bin/ld: cannot find -lur10_kin
collect2: error: ld returned 1 exit status
make[2]: *** [final_project/CMakeFiles/compete.dir/build.make:191: /ariac_ws/devel/lib/cwru_ecse_373_submission/compete] Error 1
make[1]: *** [CMakeFiles/Makefile2:511: final_project/CMakeFiles/compete.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
Invoking "make -j7 -l7" failed
```

Run:
```
export LIBRARY_PATH
LIBRARY_PATH="$LD_LIBRARY_PATH"
```

# Package Design 

## Initialization
The code initializes key componets, including the camera, order, and transform listeners, the different services required for the gripper, starting the competition, telling the AGVs a shipment is ready, etc. 

## Specific Functions, Helpers, & Types
### ArmJointState
ArmJointState is a type alias for std::array<double, 7>. This makes it easier to keep track of joint data that has been converted into the correct order.

### string getBin(osrf_gear::Product p)
This function uses the cached logical camera images to determine and cache the locations of different products.

### string getBinCheat(osrf_gear::Product& p)
This function uses the material locations service to determine the bin in which a certain product can be found. If the item is found on the conveyor belt, that result is rejected. It will only return bin-based locations.

### tf2::Transform getArmTransformFrom(std::string bin)
This function gets a tf transform from the given "bin" frame to the arm1_base_frame.

### geometry_msgs::Pose applyTransform(const tf2::Transform& transform, const geometry_msgs::Pose& pose)
This function takes in a tf transform and a pose and applies the transform. Instead of doing this using tf2::doTransform, this function converts the geometry_msgs::Pose into a tf2 transform and multiplies it with the given tf transform. This function was written in an attempt to fix the numerous transform issues the run environment was running into.

### double poseToYaw(const geometry_msgs::Pose& pose)
This function is designed to return the yaw rotation of a pose in order to rotate the end effector to match the desired pose.

### ArmJointState getKinematicsFromPose(geometry_msgs::Pose &desired, bool isAGVPose = false)
This function converts a pose into an ArmJointState (array of doubles) and returns it. It determines the ArmJointState using ur10 inverse kinematics. It determines the best inverse kinematics solution using a simple technique.

```
int sol_idx = 0;
  for (int i = 0; i < num_sols; ++i) {
    if (output_poses[i][3] > M_PI && output_poses[i][1] > M_PI && output_poses[i][2] < M_PI) {
      sol_idx = i;
      break;
    }
  }
```

This accepts the first solution where the ~shoulder_pan_joint~ and ~elbow_joint~ are both greater than pi radians and the ~shoulder_lift_joint~ is than pi radians.

This method preserves the linear actuator location and the end effector rotation.

### void moveToGoalJointState(ArmJointState goalJointState, ros::Duration duration)
This function takes a goalJointState generated by the getKinematicsFromPose function and a duration and publishes the trajectory using a ROS publisher. A ROS publisher was chosen over an action server because it was more robust during development and didn't pose any significant problems.

### void yawEffector(double rads)
This function takes in a desired yaw angle and normalizes it to be between 0 and 2pi, copies the current joint state and modifies the end effector rotation, and republishes it using

```
moveToGoalJointState(goalRotation, ros::Duration(1));
```

### void moveArm(geometry_msgs::Pose &desired, bool isAGVPose = false, double duration = 3){

This function takes in a desired pose and moves the arm to that pose. It does this by calling both ```getKinematicsFromPose(...)``` and ```moveToGoalJointState(...)```.


### void moveBase(double to, bool absolute = false)

This function takes in a position for the linear actuator, copies the current joint state, changes the linear actuator position to the given value, and republishes the trajectory.

## Listeners
1. Orders that are received are queued into a vector to be read later by the main loop.
2. AGV states are stored into a map to be read in the main loop.
3. The gripper state is listened to and stored to be read in the main loop.
4. The joint states are read and reordered to match the order of the "order" vector.
5. Logical camera images are all stored directly to a map from binId to logical camera image.

## Main Loop
The main loop is designed to handle all of the key components of the ariac. 

It ensures that all of the logical camera images have been loaded before proceeding. It returns if 2 orders have been filled already, though the ARIAC does this itself. It copies and clears the order queue. 

### Robot Poses In Use
There are 3 main poses for the robot used by this package while it moves through the ARIAC. The startPose, leftPose, and rightPose.

#### Start Pose
While moving back to a bin or retreating from an AGV, the robot defaults to the startPose. This pose is a great starting point for reaching into the bins because it avoids the logical cameras. It is hovered slightly above the bins but never runs into them.

![start pose](https://github.com/LiamPierce/ecse373_f22_t3_ariac_final_5/blob/master/Screenshot%202022-12-09%20at%209.34.53%20PM.png)

### Left Pose
While moving to the left AGV, the leftPose is used because it is facing the left AGV. This makes depositing the part easier. It also makes planning the yaw change for the end effector easier.

![left pose](https://github.com/LiamPierce/ecse373_f22_t3_ariac_final_5/blob/master/Screenshot%202022-12-09%20at%209.37.17%20PM.png)

### Right Pose
Similarly, while moving to the right AGV, the rightPose is used because it is facing the right AGV. This makes depositing the part easier and makes planning the yaw change easier.

![right pose](https://github.com/LiamPierce/ecse373_f22_t3_ariac_final_5/blob/master/Screenshot%202022-12-09%20at%209.35.10%20PM.png)

### Per Product Routine
For every order in the copied queue, the main loop will go through every shipment. For each shipment, the loop goes to each product.

For each product, the loop determines which bin it is in, finds the logical camera image from the logical camera image map, and finds the best product to pick from the bin and its pose relative to the camera.

If there are no products in the bin, the loop will skip the product.

The loop then moves to the linear actuator position stored in the binPositions map according to the binId key. It does this movement using  ```moveBase(binPositions[binId], true);```. The position stored is an absolute linear actuator position.

The loop sleeps for half a second to make sure there isn't oscillation in the base position. 

The loop then converts the part pose to the arm's frame.
```
tf2::Transform tf = getArmTransformFrom("logical_camera_"+binId+"_frame");
geometry_msgs::Pose goalPose = applyTransform(tf, im.models.front().pose);
```

Since this conversion seems to be broken in my environment, the main loop then does a transform of its own using the remapValues helper function described above.

```
goalPose.position.x = remapValues(goalPose.position.x, -0.397151, -0.804914, -0.21012, -0.377455);
goalPose.position.y = remapValues(goalPose.position.y, 0.435990, 0.881735, 0.144, 0.358278);
goalPose.position.z += 0.239;
goalPose.orientation.w = 0.707;
goalPose.orientation.x = 0.0;
goalPose.orientation.y = 0.707;
goalPose.orientation.z = 0.0;
```

The vacuum gripper is turned on now before reaching the part.

```
osrf_gear::VacuumGripperControl request;
request.request.enable = true;
bool success = gripperClient.call(request);
if (!success){
  ROS_ERROR("GRIPPER FAILED TO ENABLE");
}
```

The arm is then instructed to move to ```goalPose``` using the ```moveArm(...)``` function.
Though it isn't common, the gripper may fail to pick up the part. There is a failsafe built into the design of this call.

```
while (!gripperState.attached){
  moveArm(goalPose);
  ros::Duration(0.3).sleep();

  if (!gripperState.attached){
    goalPose.position.z += 0.05;
    moveArm(goalPose, false, 0.3);
    goalPose.position.z -= 0.05;
    goalPose.position.y += 0.02; //Move a little... this shouldn't be needed often.
  }
}
```
If the gripper is somehow already holding a part, it will not attempt to pick up the part at all. If the gripper fails to pick up the part, it will back off slightly, readjust the Y coordinate slightly, and try again.

The arm moves to the startPose to avoid the logical camera while moving to the left or right pose depending on which AGV this part is destined for. The arm then moves to that left or right pose then to the correct AGV.

The program sleeps for one second to ensure the robot isn't oscillating.

The desired coordinates for the part on the AGV tray are determined from the part goal pose in the shipment specification. That agv-frame coordinate is then converted using a tf transform.

```
tf = getArmTransformFrom("kit_tray_" + std::to_string(agv));
goalPose= applyTransform(tf, p.pose);
```

The x and y positions are then remapped again depending on which AGV is being used. This is again because of the transformation issue.

```
goalPose.position.x = remapValues(goalPose.position.x, -0.17, 0.18, -0.04, 0.13);
if (agv == 1){
  goalPose.position.y = remapValues(goalPose.position.y, 0.75, 1.10920, 0.28, 0.52);
  while (ros::ok() && agvStates["agv1"] != "ready_to_deliver"){
    ros::Duration(0.1).sleep();
  }
}else{

  goalPose.position.y = remapValues(goalPose.position.y, -1.10920, -0.75, -0.52, -0.28);
  while (ros::ok() && agvStates["agv2"] != "ready_to_deliver"){
    ros::Duration(0.1).sleep();
  }
}
goalPose.position.z = -0.01;
```

The end effector is rotated to the goal yaw determined by the poseToYaw helper explained above.
```
double yaw = poseToYaw(goalPose);
yawEffector(yaw);
```
The arm is then moved to place the part on the AGV.
```
moveArm(goalPose, true);
```

And the vacuum gripper is disabled.

```
request.request.enable = false;
success = gripperClient.call(request);
if (!success){
  ROS_ERROR("GRIPPER FAILED TO DISABLE");
}
```

The arm is then moved back to the startPose.

This same task is repeated for every product.

### Finishing a shipment

Once every product in a shipment has been fulfilled, the shipment's AGV is called.

```
osrf_gear::AGVControl submit;
submit.request.shipment_type = s.shipment_type;

bool success = false;
if (s.agv_id == "agv1"){
  success = agv1Client.call(submit);
}else{
  success = agv2Client.call(submit);
}

if (!success){
  ROS_ERROR("Failed to called AGV Client!");
}else if (!submit.response.success){
  ROS_ERROR("Shipment unsuccessful with message %s", submit.response.message.c_str());
}else{
  ROS_INFO("Successfully submitted shipment.");
}
```

## Key Package Takeaways

There are a few key components that allowed the package to succeed.

1. The use of key poses to allow the robot to skillfully avoid the logical cameras and place itself in a beneficial position based on the current situation.
2. The use of the remapValues function allowed the transforms to be useful. Since the coordinate transforms were completely incorrect in my environment, this package required the use of complicated coordinate remapping to succeed at all. Without this component the project would not be complete.
3. Yawwing the end effector to increase the ARIAC pose score.
4. A well seperated data source and data controller and well defined helper methods. Since the order listening, joint listening, etc was well seperated from the main loop, it was very easy to update the per-product routine.

# General Block Diagram

![block diagram](https://github.com/LiamPierce/ecse373_f22_t3_ariac_final_5/blob/master/Screenshot%202022-12-09%20at%2010.21.28%20PM.png)
