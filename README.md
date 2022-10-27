# Package Design

This package starts the araic competition using the [/araic/start_competition](https://bitbucket.org/osrf/ariac/wiki/2019/competition_interface_documentation) service.

The executable then subscribes to the order topic to listen for new orders. When a new order is received, it pushes the order to the order queue.

It then subscribes to every logical camera using boost::bind and a single handler method. The boost::bind passes the topic to the handler so that the handler knows which logical camera it is receiving data for. The handler then stores this information in a map of topic=>LogicalCameraImage.

The code loops to check for new orders in the queue. If there's a new order, and there's data available from the logical cameras, it will loop through every shipment and every product to find the correct bin and pose using the [material_locations](https://bitbucket.org/osrf/ariac/wiki/2019/competition_interface_documentation) service.

The code also contains a method for finding (and caching) the correct bin location using just the logical camera data.

# Running this package:

## Launch the competition environment:

```
roslaunch final_project_5 competition.launch
```

## Launch the code that attempts the competition:
```
roslaunch final_project_5 compete.launch
```
