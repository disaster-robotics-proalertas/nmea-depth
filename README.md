# NMEA depth to ROS (ros_nmea_depth package)
ROS package to parse NMEA GPS and Depth sentences, obtained from a Lowrance SL2 side-scanner, into ROS topics.
The package currently contains one [node](https://github.com/rgmaidana/nmea-depth/blob/master/scripts/nmea_depth_udp.py), which receives the NMEA sentences via UDP.
As there are no standard messages for NMEA depth sentences, two new message types are defined:

* [DepthOfWater.msg](https://github.com/rgmaidana/nmea-depth/blob/master/msg/DepthOfWater.msg): Following the [DPT sentence](https://gpsd.gitlab.io/gpsd/NMEA.html#_dpt_depth_of_water)
* [DepthBelowTransducer.msg](https://github.com/rgmaidana/nmea-depth/blob/master/msg/DepthBelowTransducer.msg): Following the [DBT sentence](https://gpsd.gitlab.io/gpsd/NMEA.html#_dbt_depth_below_transducer)

Partly based on code from [this](https://github.com/rolker/seapath/blob/master/nodes/seapath_nmea_node.py) ROS node.

## Dependencies

* [nmea_msgs](http://wiki.ros.org/nmea_msgs)

## Installation

* Install or resolve dependencies (see above)
* Clone this repository to your catkin workspace src directory (e.g., $HOME/catkin_ws/src)
* From your catkin workspace root directory (e.g., $HOME/catkin_ws), run the *catkin_ws* command

## Usage

You can run the [UDP node](https://github.com/rgmaidana/nmea-depth/blob/master/scripts/nmea_depth_udp.py) by running a *roscore*, and then using the *rosrun* command:

```
roscore &
rosrun ros_nmea_depth nmea_depth_udp.py
```

Alternatively, a [ROS launch file](https://github.com/rgmaidana/nmea-depth/blob/master/launch/lowrance.launch) is provided, where the default parameters (see below) are set as:

* Frame ID: lowrance
* IP Address: None (UDP multicast)
* UDP Port: 12021

## Published Topics

All topics are published relative the device's namespace, referred here by the *frame_id* parameter, and the sensor where the data is obtained.
For example, if running the [lowrance](https://github.com/rgmaidana/nmea-depth/blob/master/launch/lowrance.launch) launch file, the frame_id will be *lowrance*, and thus the GPS time reference topic will be */lowrance/gps/time_reference*.

* <frame_id>/nmea_sentence ([nmea_msgs/Sentence](http://docs.ros.org/api/nmea_msgs/html/msg/Sentence.html)): A message representing a single NMEA0183 sentence
* <frame_id>/gps/fix ([sensor_msgs/NavSatFix](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)): GPS Position Fix information (WGS84)
* <frame_id>/gps/vel ([geometry_msgs/TwistStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html)): GPS reported ground speed (in m/s)
* <frame_id>/gps/time_reference ([sensor_msgs/TimeReference](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html)): Time reference in GPS time (GPST)
* <frame_id>/gps/active_satellites ([nmea_msgs/Gpgsa](http://docs.ros.org/api/nmea_msgs/html/msg/Gpgsa.html)): GPS Dillution of Position (DOP) and active satellite data
* <frame_id>/gps/satellites_in_view ([nmea_msgs/Gpgsv](http://docs.ros.org/api/nmea_msgs/html/msg/Gpgsv.html)): Information on all GPS satellites currently in view, specifically PRN, elevation angle, azimuth angle and signal-to-noise ratio
* <frame_id>/depth/water ([nmea_depth/DepthOfWater](https://github.com/rgmaidana/nmea-depth/blob/master/msg/DepthOfWater.msg)): Depth of water column, following the [DPT](https://gpsd.gitlab.io/gpsd/NMEA.html#_dpt_depth_of_water) NMEA sentence
* <frame_id>/depth/water ([nmea_depth/DepthBelowTransducer](https://github.com/rgmaidana/nmea-depth/blob/master/msg/DepthBelowTransducer.msg)): Depth of water column, following the [DBT](https://gpsd.gitlab.io/gpsd/NMEA.html#_dbt_depth_below_transducer) NMEA sentence

## Parameters

* ~address (string, default: None)

UDP address for source of NMEA sentences. Default is none, as the UDP socket should accept messages from all IP addresses (multicast).

* ~port (int, default: 12021)

UDP port for source of NMEA sentences. Default is a random palindrome number, **you must change this parameter to reflect the inbound UDP connection with your device.** You can do this either by modifying the [ROS launch file](https://github.com/rgmaidana/nmea-depth/blob/master/launch/lowrance.launch) or by specifying this parameter when using *rosrun*:

```
rosrun ros_nmea_depth nmea_depth_udp.py _port:=<your_UDP_port>
```

* ~frame_id (string, default: None)

Name or ID of device which produces the NMEA sentences (e.g., lowrance).

## Contributors

* [Renan Maidana](https://github.com/rgmaidana): ROS package and UDP node
* [Guilherme Heck](https://github.com/heckgui): Sidescanner (Lowrance SL2) interface, debugging and logging
* [Alexandre Amory](https://github.com/amamory): Guidance and logistics
