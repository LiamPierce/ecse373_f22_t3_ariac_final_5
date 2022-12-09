#include <map>
#include <sstream>

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"

#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include "osrf_gear/Order.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"
#include "osrf_gear/AGVControl.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

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
std::map<string, double> binPositions = {{"bin1", 0}, {"bin2", 0}, {"bin3", 0}, {"bin4", -0.3}, {"bin5", 0.35}, {"bin6", 1.27}};
//std::map<string, double> binPositions = {{"bin1", 0}, {"bin2", 0}, {"bin3", 0}, {"bin4", -0.4}, {"bin5", 0.35}, {"bin6", 1.2}};

using ArmJointState = std::array<double, 7>;
std::vector<std::string> order = {"linear_arm_actuator_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
ArmJointState jointState;

ros::ServiceClient gripperClient;
osrf_gear::VacuumGripperState gripperState;

ros::Publisher armPositionPublisher;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;
TrajectoryServer* trajectoryActionServer;

tf2_ros::Buffer tfBuffer;

double current_pose[6], output_poses[8][6];
double T_forward[4][4], T_destination[4][4];


trajectory_msgs::JointTrajectory desired;

std::map<string, string> agvStates = {{"agv1", "ready_to_deliver"}, {"agv2", "ready_to_deliver"}};
ros::ServiceClient agv1Client;
ros::ServiceClient agv2Client;


double remapValues(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

  ArmJointState newJointState;

  for(int i = 0; i < 7; ++i){
    for (int j = 0; j < js.name.size(); ++j){
      if (order[i] == js.name[j]){
        newJointState[i] = js.position[j];
      }
    }
  }

  jointState = newJointState;

  {
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < order.size(); ++i){
      if (i != 0){
        ss << ",";
      }

      ss << jointState[i];
    }

    ss << "]";

    ROS_INFO_STREAM_THROTTLE(10.0, "Current joint state: " << ss.str());
  }
}

void gripperStateListener(const osrf_gear::VacuumGripperStateConstPtr& msg) {
	gripperState = *msg;
}

tf2::Transform getArmTransformFrom(std::string bin){
  geometry_msgs::TransformStamped tf;
  try{
    tf = tfBuffer.lookupTransform("arm1_base_link", bin, ros::Time(0.0), ros::Duration(1.0));
    ROS_INFO(
      "Transform to [%s] from [%s]", tf.header.frame_id.c_str(),
      tf.child_frame_id.c_str()
    );
  }catch(tf2::TransformException &ex){
    ROS_ERROR("Transform error: %s", ex.what());
  }

  tf2::Transform tf_transform;
  tf2::fromMsg(tf.transform, tf_transform);

  return tf_transform;
}

//This is an attempt to fix the tf2 transformations I've been experiencing.
geometry_msgs::Pose applyTransform(const tf2::Transform& transform, const geometry_msgs::Pose& pose){

  tf2::Transform tf_pose;
  tf2::fromMsg(pose, tf_pose);

  tf2::Transform result = transform * tf_pose;

  geometry_msgs::Pose result_pose;
  tf2::toMsg(result, result_pose);

  return result_pose;
}

void agvListener(const std_msgs::StringConstPtr &msg, const string &agv) {
  agvStates[agv] = msg->data;
}

void printPose(const geometry_msgs::Pose &pose) {
  ROS_INFO("%f %f %f", pose.position.x, pose.position.y, pose.position.z);
}

double poseToYaw(const geometry_msgs::Pose& pose){
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Matrix3x3 mat(q);

  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  return yaw;
}

int packets = 0;
int actionServerPackets = 0;

ArmJointState getKinematicsFromPose(geometry_msgs::Pose &desired, bool isAGVPose = false){
  T_forward[0][3] = desired.position.x;
  T_forward[1][3] = desired.position.y;
  T_forward[2][3] = desired.position.z;
  T_forward[3][3] = 1.0;

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

  int num_sols = ur_kinematics::inverse((double *)&T_forward, (double *)&output_poses);

  if (num_sols == 0) {
    ROS_ERROR("No inverse kinematics solutions found!");
    //ros::shutdown();
  }

  int sol_idx = 0;
  if (isAGVPose){
    for (int i = 0; i < num_sols; ++i) {
      if (output_poses[i][3] > M_PI && output_poses[i][1] > M_PI && output_poses[i][2] < M_PI) {
        sol_idx = i;
        break;
      }
    }
  }else{
    for (int i = 0; i < num_sols; ++i) {
      if (output_poses[i][3] > M_PI && output_poses[i][1] > M_PI && output_poses[i][2] < M_PI) {
        sol_idx = i;
        break;
      }
    }
  }

  ArmJointState goalJointState;
  for (int i = 0; i < 6; ++i) {
    goalJointState[i + 1] = output_poses[sol_idx][i];
  }
  goalJointState[0] = jointState[0]; //Keep linear actuator at the correct position.
  goalJointState[6] = jointState[6]; //Keep end effector at the right rotation.

  {
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < order.size(); ++i){
      if (i != 0){
        ss << ",";
      }

      ss << goalJointState[i];
    }

    ss << "]";

    ROS_INFO_STREAM("Goal joint state: " << ss.str());
  }

  return goalJointState;
}
void moveToGoalJointState(ArmJointState goalJointState, ros::Duration duration){
  trajectory_msgs::JointTrajectory jointTrajectory;

  jointTrajectory.header.seq = packets++;
  jointTrajectory.header.stamp = ros::Time::now();
  jointTrajectory.header.frame_id = "/world";

  jointTrajectory.joint_names = order;

  jointTrajectory.points.resize(2);

  jointTrajectory.points[0].time_from_start = ros::Duration(0.1);
  jointTrajectory.points[1].time_from_start = duration;

  for (int i = 0; i < 7; ++i){
    jointTrajectory.points[0].positions.push_back(jointState[i]);
    jointTrajectory.points[1].positions.push_back(goalJointState[i]);
  }

  {
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < order.size(); ++i){
      if (i != 0){
        ss << ",";
      }

      ss << jointTrajectory.points[1].positions[i];
    }

    ss << "]";

    ROS_INFO_STREAM("Trajectory joint output: " << ss.str());
  }

  armPositionPublisher.publish(jointTrajectory);
}

void yawEffector(double rads){
  ArmJointState goalRotation = jointState;

  while (rads > 2 * M_PI || rads < 0){
    rads += (rads > 2 * M_PI ? -1 : 1) * 2 * M_PI;
  }

  goalRotation[6] = rads;

  moveToGoalJointState(goalRotation, ros::Duration(1));
}

void moveArm(geometry_msgs::Pose &desired, bool isAGVPose = false, double duration = 3){

  printPose(desired);

  ArmJointState goalJointState = getKinematicsFromPose(desired, isAGVPose);
  moveToGoalJointState(goalJointState, ros::Duration(duration));

  ros::Duration(ros::Duration(duration + 0.2)).sleep();
}

void moveBase(double to, bool absolute = false){

  trajectory_msgs::JointTrajectory jointTrajectory;

  jointTrajectory.header.seq = packets++;
  jointTrajectory.header.stamp = ros::Time::now();
  jointTrajectory.header.frame_id = "/world";

  jointTrajectory.joint_names = order;

  jointTrajectory.points.resize(2);
  jointTrajectory.points[0].positions.resize(jointTrajectory.joint_names.size());
  jointTrajectory.points[1].positions.resize(jointTrajectory.joint_names.size());

  jointTrajectory.points[0].time_from_start = ros::Duration(0.0);
  jointTrajectory.points[1].time_from_start = ros::Duration(1.6);

  for (int indy = 0; indy < 7; indy++) {
    jointTrajectory.points[1].positions[indy] = jointState[indy];
  }

  jointTrajectory.points[1].positions[0] = (!absolute ? jointState[1] : 0) + to;

  armPositionPublisher.publish(jointTrajectory);

  /*control_msgs::FollowJointTrajectoryAction jointTrajectoryPayload;

  jointTrajectoryPayload.action_goal.goal.trajectory = jointTrajectory;
  jointTrajectoryPayload.action_goal.header.seq = actionServerPackets++;
  jointTrajectoryPayload.action_goal.header.stamp = ros::Time::now();
  jointTrajectoryPayload.action_goal.header.frame_id = "/world";

  jointTrajectoryPayload.action_goal.goal_id.stamp = ros::Time::now();
  jointTrajectoryPayload.action_goal.goal_id.id = std::to_string(actionServerPackets - 1);

  trajectoryActionServer->sendGoal(jointTrajectoryPayload.action_goal.goal);
  trajectoryActionServer->waitForServer();
  bool succeeded = trajectoryActionServer->waitForResult(ros::Duration(20.0));

  if (succeeded){
    actionlib::SimpleClientGoalState state = trajectoryActionServer->getState();
    ROS_INFO("Trajectory finished with state %s with text: %s", state.toString().c_str(), state.getText().c_str());
  }else{
    ROS_ERROR("TRAJECTORY TIMEOUT!");
  }*/
  ros::Duration(4.0).sleep();
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
  ros::Subscriber jointStateubsriber = n.subscribe("arm1/joint_states", 1, jointStateListener);
  ros::Subscriber gripperStateSubscriber = n.subscribe("arm1/gripper/state", 1000, gripperStateListener);

  for (int i = 1; i<=2;i++){
    ros::Subscriber agvSubscriber = n.subscribe<std_msgs::String>(
      "agv/"+std::to_string(i)+"/state",
      32,
      boost::bind(agvListener, _1, "agv"+std::to_string(i))
    );
  }

  agv1Client = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
  agv2Client = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
  agv1Client.waitForExistence(ros::Duration(0.0));
  agv2Client.waitForExistence(ros::Duration(0.0));

  armPositionPublisher = n.advertise<trajectory_msgs::JointTrajectory>("arm1/arm/command", 10);
  gripperClient = n.serviceClient<osrf_gear::VacuumGripperControl>("arm1/gripper/control");

  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  spinner.start();
  ros::Rate sleeper(20);


  trajectoryActionServer = new TrajectoryServer("/ariac/arm1/arm/follow_joint_trajectory", false);
  ROS_INFO("Waiting for trajectory action server");
  trajectoryActionServer->waitForServer();
  ROS_INFO("Trajectory server running");

  bool start = true;
  geometry_msgs::Pose startPose;
  startPose.position.x = -0.05;
  startPose.position.y = 0.20;
  startPose.position.z = 0.12;

  int ordersCompleted = 0;
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

    if (ordersCompleted == 2){
      return 0;
      /*
      osrf_gear::competition_state end;
      end.request.state = osrf_gear::competition_state::STOP;
      if (competitionStateClient.call(end)){
        ROS_INFO("Successfully completed the competition");
      }else{
        ROS_ERROR("Failed to end the competition!");
      }
      */
    }

    for (osrf_gear::Order o : oqc){
      if (start){
        start = false;
        moveArm(startPose);
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

          moveBase(binPositions[binId], true);

          ros::Duration(0.5).sleep(); //Make sure the arm isn't oscillating from the base movement.

          tf2::Transform tf = getArmTransformFrom("logical_camera_"+binId+"_frame");
          geometry_msgs::Pose goalPose = applyTransform(tf, im.models.front().pose);

          ROS_INFO("Part Pose: ");
          printPose(im.models.front().pose);
          ROS_INFO("ARM Frame Pose: ");
          printPose(goalPose);

          goalPose.position.x = remapValues(goalPose.position.x, -0.397151, -0.804914, -0.21012, -0.377455);
          goalPose.position.y = remapValues(goalPose.position.y, 0.435990, 0.881735, 0.144, 0.358278);
          goalPose.position.z += 0.239;
          goalPose.orientation.w = 0.707;
          goalPose.orientation.x = 0.0;
          goalPose.orientation.y = 0.707;
          goalPose.orientation.z = 0.0;

          osrf_gear::VacuumGripperControl request;
          request.request.enable = true;
		      bool success = gripperClient.call(request);
          if (!success){
            ROS_ERROR("GRIPPER FAILED TO ENABLE");
          }

          //Make sure to grab a part.
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

          int agv = (s.agv_id == "agv2" || s.agv_id == "any" ? 2 : 1);
          //agv = 1;

          moveArm(startPose);
          moveBase(agv == 1 ? 2.2 : -2.2, true);

          ros::Duration(1).sleep();

          tf = getArmTransformFrom("kit_tray_" + std::to_string(agv));
          goalPose= applyTransform(tf, p.pose);

          ROS_INFO("AGV Pose Request: ");
          printPose(p.pose);
          ROS_INFO("ARM Frame Pose: ");
          printPose(goalPose);

          goalPose.position.x = remapValues(goalPose.position.x, -0.25, 0.25, -0.04, 0.13);
          if (agv == 1){

            goalPose.position.y = remapValues(goalPose.position.y, 0.5, 1.40920, 0.28, 0.52);
            while (ros::ok() && agvStates["agv1"] != "ready_to_deliver"){
              ros::Duration(0.1).sleep();
            }
          }else{

            goalPose.position.y = remapValues(goalPose.position.y, -1.40920, -0.500, -0.52, -0.28);
            while (ros::ok() && agvStates["agv2"] != "ready_to_deliver"){
              ros::Duration(0.1).sleep();
            }
          }

          goalPose.position.z = -0.01;

          double yaw = poseToYaw(goalPose);
          moveArm(goalPose, true);

          ROS_INFO("Rotating to yaw %f", yaw);
          yawEffector(yaw);

          ros::Duration(1).sleep();

          request.request.enable = false;
		      success = gripperClient.call(request);
          if (!success){
            ROS_ERROR("GRIPPER FAILED TO DISABLE");
          }

          moveArm(startPose);
          ros::Duration(1).sleep();
        }

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
      }
      ordersCompleted++;
    }


    sleeper.sleep();
    ++count;
  }


  return 0;
}
