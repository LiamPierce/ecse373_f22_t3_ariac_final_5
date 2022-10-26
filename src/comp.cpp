#include <map>
#include <sstream>

#include "ros/ros.h"
#include "std_srvs/Trigger.h"

#include "osrf_gear/Order.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"

using namespace std;

std::vector<osrf_gear::Order> orderQueue;

std::map<string, string> binCache;
ros::ServiceClient materialLocationsService;
std::map<string, ros::Subscriber> logicCameraSubscribers;
std::map<string, osrf_gear::LogicalCameraImage> logicalCameraImages;

string * getBin(osrf_gear::Product p){

  if (binCache.find(p.type) != binCache.end()){
    return &binCache[p.type];
  }
  ROS_INFO_STREAM(logicalCameraImages.size());
  for (int i = 1; i<=6;i++){
    osrf_gear::LogicalCameraImage im = logicalCameraImages["logical_camera_bin"+i];

    if (im.models.size() == 0){
      continue;
    }

    if (im.models[0].type == p.type){
      binCache[p.type] = "bin"+i;
      return &binCache[p.type];
    }
  }

  return NULL;
}

string * getBinCheat(osrf_gear::Product p){
  osrf_gear::GetMaterialLocations c;
  c.request.material_type = p.type.c_str();
  materialLocationsService.call(c);

  if (!c.response.storage_units.size()){
    ROS_INFO("\t\tNOT FOUND IN ANY BIN!");
    return NULL;
  }

  ROS_INFO_STREAM(c.response.storage_units.size());

  for (osrf_gear::StorageUnit unit : c.response.storage_units){
    if ((string) unit.unit_id != "belt"){
      static string bin = (string) unit.unit_id;
      return &bin;
    }
  }

  return NULL;
}

void orderHandler(const osrf_gear::Order& o){
  ROS_INFO("Received order.");
  orderQueue.push_back(o);
}

void logicalCameraHandler(const osrf_gear::LogicalCameraImage::ConstPtr &im, const string &topic){
  //ROS_INFO_STREAM(topic);
  logicalCameraImages[topic] = *im;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "final_project_5");
  ros::NodeHandle n;

  std_srvs::Trigger beginComp;
  ros::ServiceClient beginClient = n.serviceClient<std_srvs::Trigger>("start_competition");

  ROS_INFO("Sending begin client call.");
  beginClient.call(beginComp);

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

          string *binId = getBin(p);
          if (binId){
            ROS_INFO_STREAM("\t\tBIN: "<<*binId);
          }

          ROS_INFO_STREAM("\t\tLooking at logical_camera_"+(*binId));
          if (logicalCameraImages.find("logical_camera_"+(*binId)) == logicalCameraImages.end()){
            ROS_INFO("NO DATA ON THIS BIN");
            continue;
          }

          osrf_gear::LogicalCameraImage im = logicalCameraImages["logical_camera_"+(*binId)];

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

          /*geometry_msgs::TransformStamped tfStamped;
          try{
            tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_"+(*binId), ros::Time(0.0), ros::Duration(1.0));
            ROS_DEBUG(
              "Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
              tfStamped.child_frame_id.c_str()
            );
          }catch(tf2::TransformException &ex){
            ROS_ERROR("%s", ex.what());
          }

          geometry_msgs::PoseStamped part_pose, goal_pose;
          part_pose.pose = im.pose;
          tf2::doTransform(part_pose, goal_pose, tfStamped);

          goal_pose.pose.position.z += 0.10;
          goal_pose.pose.orientation.w = 0.707;
          goal_pose.pose.orientation.x = 0.0;
          goal_pose.pose.orientation.y = 0.707;
          goal_pose.pose.orientation.z = 0.0;*/
        }
      }
    }


    loop_rate.sleep();
    ++count;
  }


  return 0;
}
