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
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"

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
std::map<string, double> binPositions = {{"bin1", 0}, {"bin2", 0}, {"bin3", 0}, {"bin4", -0.6}, {"bin5", 0.33}, {"bin6", 1.1}};

sensor_msgs::JointState jointStates;

ros::ServiceClient gripperClient;
osrf_gear::VacuumGripperState gripperState;

ros::Publisher armPositionPublisher;

tf2_ros::Buffer tfBuffer;

double current_pose[6], output_poses[8][6];
double T_forward[4][4], T_destination[4][4];


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
  //ROS_INFO_STREAM_THROTTLE(10, jointStates);
}

void gripperStateListener(const osrf_gear::VacuumGripperStateConstPtr& msg) {
	gripperState = *msg;
}

bool armIsMoving() {
  bool s = false;
  for (const double v : jointStates.velocity) {
    if (std::abs(v) > 0.1){
      return true;
    }
  }

  return false;
}

int packets = 0;
void moveArm(geometry_msgs::Pose &desired){

  std::vector<std::string> order = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

  for(int i = 0; i < order.size(); ++i) {
    current_pose[i] = 0.0;
    for(int j = 0; j < jointStates.name.size(); ++j) {

      if(order[i] == jointStates.name[j]) {
        current_pose[i] = jointStates.position[j];
      }
    }
  }

  ur_kinematics::forward((double *)&current_pose, (double *)&T_forward);

  T_forward[0][3] = desired.position.x;
  T_forward[1][3] = desired.position.y;
  T_forward[2][3] = desired.position.z;
  T_forward[3][3] = 1.0;
  // The orientation of the end effector so that the end effector is down.
  T_forward[0][0] = 0.0;
  T_forward[0][1] = -1.0;
  T_forward[0][2] = 0.0;

  T_forward[1][0] = 0.0;
  T_forward[1][1] = 0.0;
  T_forward[1][2] = 1.0;

  T_forward[2][0] = -1.0;
  T_forward[2][1] = 0.0;
  T_forward[2][2] = 0.0;

  T_forward[3][0] = 0.0;
  T_forward[3][1] = 0.0;
  T_forward[3][2] = 0.0;

  int num_sols = ur_kinematics::inverse((double *)&T_forward, (double *)&output_poses, 0.0);

  if (num_sols == 0) {
    ROS_ERROR("No inverse kinematics solutions found!");
    //ros::shutdown();
  }

  for(int i = 0; i < num_sols; ++i) {
    for(int j = 0; j < 6; ++j) {
      std::cout << output_poses[i][j] << " ";
    }
    std::cout << std::endl;
  }

  trajectory_msgs::JointTrajectory jointTrajectory;

  jointTrajectory.header.seq = packets++;
  jointTrajectory.header.stamp = ros::Time::now();
  jointTrajectory.header.frame_id = "/world";

  // Set the names of the joints being used.  All must be present.
  jointTrajectory.joint_names.clear();
  jointTrajectory.joint_names.push_back("linear_arm_actuator_joint");
  jointTrajectory.joint_names.push_back("shoulder_pan_joint");
  jointTrajectory.joint_names.push_back("shoulder_lift_joint");
  jointTrajectory.joint_names.push_back("elbow_joint");
  jointTrajectory.joint_names.push_back("wrist_1_joint");
  jointTrajectory.joint_names.push_back("wrist_2_joint");
  jointTrajectory.joint_names.push_back("wrist_3_joint");

  jointTrajectory.points.resize(2);
  jointTrajectory.points[0].positions.resize(jointTrajectory.joint_names.size());
  jointTrajectory.points[1].positions.resize(jointTrajectory.joint_names.size());

  for (int indy = 0; indy < jointTrajectory.joint_names.size(); indy++) {
    for (int indz = 0; indz < jointStates.name.size(); indz++) {
      if (jointTrajectory.joint_names[indy] == jointStates.name[indz]) {
        jointTrajectory.points[0].positions[indy] = jointStates.position[indz];
        break;
      }
    }
  }

  jointTrajectory.points[0].time_from_start = ros::Duration(0.0);
  jointTrajectory.points[1].time_from_start = ros::Duration(2.0);

  int solution_index = 0;

  jointTrajectory.points[1].positions[0] = jointStates.position[1];

  for (int indy = 0; indy < 6; indy++) {
    jointTrajectory.points[1].positions[indy + 1] = output_poses[solution_index][indy];
  }

  armPositionPublisher.publish(jointTrajectory);
  ROS_INFO("Published new joint trajectory.");

  ros::Duration(2.0).sleep();
  /*while (armIsMoving()){
    ros::Duration(0.1).sleep();
  }*/

}

void moveBase(double to, bool absolute = false){

  trajectory_msgs::JointTrajectory jointTrajectory;

  jointTrajectory.header.seq = packets++;
  jointTrajectory.header.stamp = ros::Time::now();
  jointTrajectory.header.frame_id = "/world";

  // Set the names of the joints being used.  All must be present.
  jointTrajectory.joint_names.clear();
  jointTrajectory.joint_names.push_back("linear_arm_actuator_joint");
  jointTrajectory.joint_names.push_back("shoulder_pan_joint");
  jointTrajectory.joint_names.push_back("shoulder_lift_joint");
  jointTrajectory.joint_names.push_back("elbow_joint");
  jointTrajectory.joint_names.push_back("wrist_1_joint");
  jointTrajectory.joint_names.push_back("wrist_2_joint");
  jointTrajectory.joint_names.push_back("wrist_3_joint");

  jointTrajectory.points.resize(2);
  jointTrajectory.points[0].positions.resize(jointTrajectory.joint_names.size());
  jointTrajectory.points[1].positions.resize(jointTrajectory.joint_names.size());

  for (int indy = 0; indy < jointTrajectory.joint_names.size(); indy++) {
    for (int indz = 0; indz < jointStates.name.size(); indz++) {
      if (jointTrajectory.joint_names[indy] == jointStates.name[indz]) {
        jointTrajectory.points[0].positions[indy] = jointStates.position[indz];
        break;
      }
    }
  }

  jointTrajectory.points[0].time_from_start = ros::Duration(0.0);
  jointTrajectory.points[1].time_from_start = ros::Duration(2.0);

  for (int indy = 1; indy < 7; indy++) {
    jointTrajectory.points[1].positions[indy] = jointTrajectory.points[0].positions[indy];
  }

  jointTrajectory.points[1].positions[0] = (!absolute ? jointStates.position[1] : 0) + to;

  armPositionPublisher.publish(jointTrajectory);
  ROS_INFO("Published new joint trajectory.");

  ros::Duration(5.0).sleep();
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
  ros::Subscriber gripperStateSubscriber = n.subscribe("arm1/gripper/state", 1000, gripperStateListener);

  armPositionPublisher = n.advertise<trajectory_msgs::JointTrajectory>("arm1/arm/command", 10);
  gripperClient = n.serviceClient<osrf_gear::VacuumGripperControl>("arm1/gripper/control");

  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  spinner.start();
  ros::Rate sleeper(20);

  bool start = true;
  geometry_msgs::Pose startPose;
  startPose.position.x = -0.05;
  startPose.position.y = 0.2;
  startPose.position.z = 0.20;

  int count = 0;
  while (ros::ok()){

    if (logicalCameraImages.size() < 10){
      sleeper.sleep();
      ++count;
      continue;
    }

    std::vector<osrf_gear::Order> oqc = orderQueue;
    orderQueue.clear();

    /*for (double i = 0; i<=20;i++){
      moveBase((i-10)/3, true);
      ROS_INFO("%f", (i-10)/3);
      ros::Duration(2).sleep();
    }*/

    for (osrf_gear::Order o : oqc){
      if (start){
        start = false;
        ROS_INFO("Moving arm to home.");
        moveArm(startPose);
        //ros::shutdown();
      }


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

          geometry_msgs::TransformStamped tf;
          try{
            tf = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_"+binId+"_frame", ros::Time(0.0), ros::Duration(1.0));
            ROS_INFO(
              "Transform to [%s] from [%s]", tf.header.frame_id.c_str(),
              tf.child_frame_id.c_str()
            );
          }catch(tf2::TransformException &ex){
            ROS_ERROR("Transform error: %s", ex.what());
          }

          geometry_msgs::PoseStamped partPose, goalPose;
          partPose.pose = im.models[rand() % 5].pose;
          tf2::doTransform(partPose, goalPose, tf);

          goalPose.pose.position.x += 0.46;
          goalPose.pose.position.z += 0.24;
          goalPose.pose.orientation.w = 0.707;
          goalPose.pose.orientation.x = 0.0;
          goalPose.pose.orientation.y = 0.707;
          goalPose.pose.orientation.z = 0.0;

          moveBase(binPositions[binId]);
          moveArm(goalPose.pose);

          ros::Duration(1).sleep();

          osrf_gear::VacuumGripperControl request;
          request.request.enable = true;
		      bool success = gripperClient.call(request);
          if (!success){
            ROS_ERROR("GRIPPER FAILED TO ENABLE");
          }

          moveArm(startPose);
          ros::Duration(1).sleep();
          moveArm(goalPose.pose);

          ros::Duration(1).sleep();

          request.request.enable = false;
		      success = gripperClient.call(request);
          if (!success){
            ROS_ERROR("GRIPPER FAILED TO DISABLE");
          }
        }
      }
      ROS_INFO("Moving arm to home.");
      moveArm(startPose);
    }


    sleeper.sleep();
    ++count;
  }


  return 0;
}
