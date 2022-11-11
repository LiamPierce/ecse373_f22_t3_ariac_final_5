#include <map>
#include <sstream>

#include "ros/ros.h"
#include "std_srvs/Trigger.h"

#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include "osrf_gear/Order.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"

#include "ur_kinematics/ur_kin.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

using namespace std;

std::vector<osrf_gear::Order> orderQueue;

std::map<string, string> binCache;
ros::ServiceClient materialLocationsService;
std::map<string, ros::Subscriber> logicCameraSubscribers;
std::map<string, osrf_gear::LogicalCameraImage> logicalCameraImages;

sensor_msgs::JointState jointStates;

ros::Publisher armPositionPublisher;

double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];

trajectory_msgs::JointTrajectory desired;

string getBin(osrf_gear::Product p){

  if (binCache.find(p.type) != binCache.end()){
    return binCache[p.type];
  }

  for (int i = 1; i<=6;i++){
    osrf_gear::LogicalCameraImage im = logicalCameraImages["logical_camera_bin"+i];

    if (im.models.size() == 0){
      continue;
    }

    if (im.models[0].type == p.type){
      binCache[p.type] = "bin"+i;
      return binCache[p.type];
    }
  }

  return "nobin";
}

string getBinCheat(osrf_gear::Product& p){
  osrf_gear::GetMaterialLocations c;
  c.request.material_type = (string) p.type;
  if (!materialLocationsService.call(c)){
    ROS_ERROR("Failed to call material locations service.");
    return "nobin";
  }

  if (!c.response.storage_units.size()){
    ROS_INFO("\t\tNOT FOUND IN ANY BIN!");
    return "nobin";
  }

  for (osrf_gear::StorageUnit unit : c.response.storage_units){
    if ((string) unit.unit_id != "belt"){
      return (string) unit.unit_id;
    }
  }

  return "nobin";
}

void logicalCameraHandler(const osrf_gear::LogicalCameraImage::ConstPtr &im, const string &topic){
  //ROS_INFO_STREAM(topic);
  logicalCameraImages[topic] = *im;
}

void orderHandler(const osrf_gear::Order& o){
  ROS_INFO("Received order.");
  orderQueue.push_back(o);
}

void jointStateListener(const sensor_msgs::JointState &js){
  jointStates = js;
  ROS_INFO_STREAM_THROTTLE(10, jointStates);
}

int packets = 0;
void moveArm(geometry_msgs::PoseStamped &desired){
  q_pose[0] = jointStates.position[1];
  q_pose[1] = jointStates.position[2];
  q_pose[2] = jointStates.position[3];
  q_pose[3] = jointStates.position[4];
  q_pose[4] = jointStates.position[5];
  q_pose[5] = jointStates.position[6];

  ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);

  T_des[0][3] = desired.pose.position.x;
  T_des[1][3] = desired.pose.position.y;
  T_des[2][3] = desired.pose.position.z + 0.3; // above part
  T_des[3][3] = 1.0;
  // The orientation of the end effector so that the end effector is down.
  T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
  T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
  T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
  T_des[3][0] = 0.0;  T_des[3][1] = 0.0; T_des[3][2] = 0.0;
  int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);

  // Declare a variable for generating and publishing a trajectory.
  trajectory_msgs::JointTrajectory jointTrajectory;
  // Fill out the joint trajectory header.
  // Each joint trajectory should have an non-monotonically increasing sequence number.
  jointTrajectory.header.seq = packets++;
  jointTrajectory.header.stamp = ros::Time::now(); // When was this message created.
  jointTrajectory.header.frame_id = "/world"; // Frame in which this is specified.
  // Set the names of the joints being used.  All must be present.
  jointTrajectory.joint_names.clear();
  jointTrajectory.joint_names.push_back("linear_arm_actuator_joint");
  jointTrajectory.joint_names.push_back("shoulder_pan_joint");
  jointTrajectory.joint_names.push_back("shoulder_lift_joint");
  jointTrajectory.joint_names.push_back("elbow_joint");
  jointTrajectory.joint_names.push_back("wrist_1_joint");
  jointTrajectory.joint_names.push_back("wrist_2_joint");
  jointTrajectory.joint_names.push_back("wrist_3_joint");

  // Set a start and end point.
  jointTrajectory.points.resize(2);
  // Set the start point to the current position of the joints from joint_states.
  jointTrajectory.points[0].positions.resize(jointTrajectory.joint_names.size());
  for (int indy = 0; indy < jointTrajectory.joint_names.size(); indy++) {
    for (int indz = 0; indz < jointStates.name.size(); indz++) {
      if (jointTrajectory.joint_names[indy] == jointStates.name[indz]) {
        jointTrajectory.points[0].positions[indy] = jointStates.position[indz];
        break;
      }
    }
  }
  // When to start (immediately upon receipt).
  jointTrajectory.points[0].time_from_start = ros::Duration(0.0);
  // Must select which of the num_sols solutions to use.  Just start with the first.
  int q_des_indx = 0;
  // Set the end point for the movement
  jointTrajectory.points[1].positions.resize(jointTrajectory.joint_names.size());
  jointTrajectory.points[1].velocities.resize(jointTrajectory.joint_names.size());
  // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
  jointTrajectory.points[1].positions[0] = jointStates.position[1];
  // The actuators are commanded in an odd order, enter the joint positions in the correct positions
  for (int indy = 0; indy < 6; indy++) {
    jointTrajectory.points[1].positions[indy + 1] = q_des[0][indy];
    jointTrajectory.points[1].velocities[indy + 1] = 0;
  }
  // How long to take for the movement.
  jointTrajectory.points[1].time_from_start = ros::Duration(1.0);

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm/follow_joint_trajectory", true);

  control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
  joint_trajectory_as.action_goal.goal.trajectory = jointTrajectory;
  actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
  ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
}

int main(int argc, char **argv){

  ros::init(argc, argv, "cwru_ecse_373_submission");
  ros::NodeHandle n;

  std_srvs::Trigger beginComp;
  ros::ServiceClient beginClient = n.serviceClient<std_srvs::Trigger>("start_competition");

  ROS_INFO("Sending begin client call.");
  if (!beginClient.call(beginComp)){
    ROS_ERROR("FAILED TO CALL COMPETITION SERVICE!");
  }

  if (!beginComp.response.success){
    ROS_WARN("Competition service returned failure: %s", beginComp.response.message.c_str());
  }else{
    ROS_INFO("Competition service called successfully: %s", \
    beginComp.response.message.c_str());
  }

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  materialLocationsService = n.serviceClient<osrf_gear::GetMaterialLocations>("material_locations");

  //Subscribe to all 10 logic cameras and monitor them by topic.
  string prefix = "logical_camera_bin";
  int dec = 0;
  for (int i = 1;i<=10;i++){
    string topic = prefix+std::to_string(i-dec);

    ros::Subscriber subscriber = n.subscribe<osrf_gear::LogicalCameraImage>(
      topic,
      100,
      boost::bind(logicalCameraHandler, _1, topic)
    );

    logicCameraSubscribers[topic] = subscriber;

    if (i == 6 || i == 8){
      dec=i;
      prefix = i == 6 ? "logical_camera_agv" : "quality_control_sensor_";
    }
  }

  ros::Subscriber orders = n.subscribe("orders", 1, orderHandler);
  ros::Subscriber jointStateSubsriber = n.subscribe("arm1/joint_states", 1, jointStateListener);

  armPositionPublisher = n.advertise<trajectory_msgs::JointTrajectory>("arm/command", 10);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()){
    ros::spinOnce();

    if (logicalCameraImages.size() < 10){
      loop_rate.sleep();
      ++count;
      continue;
    }

    std::vector<osrf_gear::Order> oqc = orderQueue;
    orderQueue.clear();

    for (osrf_gear::Order o : oqc){
      for (osrf_gear::Shipment s : o.shipments) {
        ROS_INFO("SHIPMENT %s", s.shipment_type.c_str());
        ROS_INFO("\tPRODUCTS:");

        for (osrf_gear::Product p : s.products){
          ROS_INFO("\t\tPRODUCT: %s", p.type.c_str());

          string binId = getBinCheat(p);
          ROS_INFO_STREAM("\t\tBIN: "<<binId);

          ROS_INFO_STREAM("\t\tLooking at logical_camera_"+(binId));
          if (logicalCameraImages.find("logical_camera_"+(binId)) == logicalCameraImages.end()){
            ROS_INFO("NO DATA ON THIS BIN");
            continue;
          }

          osrf_gear::LogicalCameraImage im = logicalCameraImages["logical_camera_"+(binId)];

          if (im.models.size() == 0){
            ROS_ERROR("\t\tProduct not found in bin.");
            continue;
          }

          ROS_INFO("\t\tPose: XYZ [%f %f %f] QTRN [%f %f %f %f]",
            im.models[0].pose.position.x,
            im.models[0].pose.position.y,
            im.models[0].pose.position.z,
            im.models[0].pose.orientation.x,
            im.models[0].pose.orientation.y,
            im.models[0].pose.orientation.z,
            im.models[0].pose.orientation.w
          );

          geometry_msgs::TransformStamped tfStamped;
          try{
            tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_"+binId+"_frame", ros::Time(0.0), ros::Duration(1.0));
            ROS_DEBUG(
              "Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
              tfStamped.child_frame_id.c_str()
            );
          }catch(tf2::TransformException &ex){
            ROS_ERROR("%s", ex.what());
          }

          geometry_msgs::PoseStamped partPose, goalPose;
          partPose.pose = im.pose;
          tf2::doTransform(partPose, goalPose, tfStamped);

          /*
          goalPose.pose.position.z += 0.10;
          goalPose.pose.orientation.w = 0.707;
          goalPose.pose.orientation.x = 0.0;
          goalPose.pose.orientation.y = 0.707;
          goalPose.pose.orientation.z = 0.0;
          */

          moveArm(goalPose);

        }
      }
    }


    loop_rate.sleep();
    ++count;
  }


  return 0;
}
