# Package Design

This package starts the araic competition using the [/araic/start_competition](https://bitbucket.org/osrf/ariac/wiki/2019/competition_interface_documentation) service.

The executable then subscribes to the order topic to listen for new orders. When a new order is received, it pushes the order to the order queue.

It then subscribes to every logical camera using boost::bind and a single handler method. The boost::bind passes the topic to the handler so that the handler knows which logical camera it is receiving data for. The handler then stores this information in a map of topic=>LogicalCameraImage.

The code loops to check for new orders in the queue. If there's a new order, and there's data available from the logical cameras, it will loop through every shipment and every product to find the correct bin and pose using the [material_locations](https://bitbucket.org/osrf/ariac/wiki/2019/competition_interface_documentation) service.

The code also contains a method for finding (and caching) the correct bin location using just the logical camera data.

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
LIBRARY_PATH="$LD_LIBRARY_PATH"
```
