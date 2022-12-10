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
## Main Loop
The main loop is designed to handle all of the key components of the ariac. 

## Listeners
Orders that are received are queued into a vector to be read later by the main loop.
AGV states are stored into a map to be read in the main loop.
The gripper state is listened to and stored to be read in the main loop.
The joint states are read and reordered to match the order of the "order" vector.

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


