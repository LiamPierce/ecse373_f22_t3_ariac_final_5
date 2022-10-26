#include <map>

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

#include "osrf_gear/Order.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"

using namespace std;


std::vector<osrf_gear::Order> orderQueue;

ros::ServiceClient materialLocationsService;
std::map<std::string, ros::Subscriber> logicCameras;

std::string * getBin(osrf_gear::Product p){
  osrf_gear::GetMaterialLocations c;
  c.request.material_type = p.type.c_str();
  materialLocationsService.call(c);

  if (!c.response.storage_units.size()){
    ROS_INFO("\t\tNOT FOUND IN ANY BIN!");
    return NULL;
  }

  return &c.response.storage_units[0].unit_id;
}

void orderHandler(const osrf_gear::Order& o){
  ROS_INFO("Received order.");
  orderQueue.push_back(o);

  for (osrf_gear::Shipment s : o.shipments) {
    ROS_INFO("SHIPMENT %s", s.shipment_type.c_str());
    ROS_INFO("\tPRODUCTS:");

    for (osrf_gear::Product p : s.products){
      ROS_INFO("\t\tPRODUCT: %s", p.type.c_str());

      std::string* binId = getBin(p);
      if (binId){
        ROS_INFO_STREAM(binId);
      }
    }
  }
}

void logicalCameraHandler(const osrf_gear::LogicalCameraImage&, std::string* bin){

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

  for (int i = 1;i<=6;i++){
    std::string topic = "logical_camera_bin"+std::to_string(i);

    logicCameras.insert(pair<std::string, ros::Subscriber>(
      topic,
      n.subscribe<osrf_gear::LogicalCameraImage>(
        topic,
        10,
        boost::bind(logicalCameraHandler, _1)
      )
    ));
  }

  for (int i = 1;i<=2;i++){
    //logicCameras.insert(pair<std::string, ros::Subscriber>("agv"+std::to_string(i), n.subscribe("logical_camera_agv"+std::to_string(i), 10, handleLogicCameras)));
  }

  for (int i = 1;i<=2;i++){
    //logicCameras.insert(pair<std::string, ros::Subscriber>("qc"+std::to_string(i), n.subscribe("quality_control_sensor_"+std::to_string(i), 10, handleLogicCameras)));
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
