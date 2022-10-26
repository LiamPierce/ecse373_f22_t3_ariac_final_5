#include <map>
#include <sstream>


#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

#include "osrf_gear/Order.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"

#include "geometry_msgs/Pose.h"

using namespace std;


std::vector<osrf_gear::Order> orderQueue;

ros::ServiceClient materialLocationsService;
std::map<string, ros::Subscriber> logicCameraSubscribers;
std::map<string, osrf_gear::LogicalCameraImage> logicalCameraImages;

string * getBin(osrf_gear::Product p){
  osrf_gear::GetMaterialLocations c;
  c.request.material_type = p.type.c_str();
  materialLocationsService.call(c);

  if (!c.response.storage_units.size()){
    ROS_INFO("\t\tNOT FOUND IN ANY BIN!");
    return NULL;
  }

  static string bin = (string) c.response.storage_units[0].unit_id;

  return &bin;
}

void orderHandler(const osrf_gear::Order& o){
  ROS_INFO("Received order.");
  orderQueue.push_back(o);

  for (osrf_gear::Shipment s : o.shipments) {
    ROS_INFO("SHIPMENT %s", s.shipment_type.c_str());
    ROS_INFO("\tPRODUCTS:");

    for (osrf_gear::Product p : s.products){
      ROS_INFO("\t\tPRODUCT: %s", p.type.c_str());

      string *binId = getBin(p);
      if (binId){
        ROS_INFO_STREAM("\t\tBIN: "<<*binId);
      }

      osrf_gear::LogicalCameraImage im = logicalCameraImages["logical_camera_"+(*binId)];

      ROS_INFO_STREAM(im.models.size());

      if (im.models.size() > 0){
        ROS_INFO("\t\tPose: XYZ [%f %f %f] QTRN [%f %f %f %f]",
          im.models[0].pose.position.x,
          im.models[0].pose.position.y,
          im.models[0].pose.position.z,
          im.models[0].pose.orientation.x,
          im.models[0].pose.orientation.y,
          im.models[0].pose.orientation.z,
          im.models[0].pose.orientation.w
        );
      }else{
        ROS_ERROR("\t\tProduct not found in bin.");
      }
    }
  }
}

void logicalCameraHandler(const osrf_gear::LogicalCameraImage::ConstPtr &im, const string &topic){
  logicalCameraImages.insert(pair<string, osrf_gear::LogicalCameraImage>(topic, *im));
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

  materialLocationsService = n.serviceClient<osrf_gear::GetMaterialLocations>("material_locations");

  //Subscribe to all 10 logic cameras and monitor them by topic.
  string prefix = "logical_camera_bin";
  int dec = 0;
  for (int i = 1;i<=10;i++){
    string topic = prefix+std::to_string(i-dec);

    ros::Subscriber subscriber = n.subscribe<osrf_gear::LogicalCameraImage>(
      topic,
      10,
      boost::bind(logicalCameraHandler, _1, topic)
    );

    logicCameraSubscribers.insert(pair<string, ros::Subscriber>(topic, subscriber));

    if (i == 6 || i == 8){
      dec=i;
      prefix = i == 6 ? "logical_camera_agv" : "quality_control_sensor_";
    }
  }

  ros::Subscriber orders = n.subscribe("orders", 1000, orderHandler);


  ros::Rate loop_rate(100);

  int count = 0;
  while (ros::ok()){
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
