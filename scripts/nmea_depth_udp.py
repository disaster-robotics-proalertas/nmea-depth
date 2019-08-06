#!/usr/bin/env python

from nmea_depth.msg import DepthBelowTransducer, DepthOfWater
from nmea_msgs.msg import Sentence
import signal
import socket
import rosparam
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TimeReference
from geometry_msgs.msg import TwistStamped
import datetime
import calendar

# UDP socket
udp_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Uncomment this line if too many problems with "address already in use"
# udp_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Callback to handle keyboard interruption
def handle_sigint(sig, frame):
    udp_in.close()      # Close UDP port
    rospy.loginfo("[nmea_depth_udp] Quitting...")
    rospy.signal_shutdown(reason="Interrupted")     # Shutdown ROS node

def nmea_depth_udp():
    # Init node
    rospy.init_node("nmea_depth_udp", anonymous=True)
    rospy.loginfo("[nmea_depth_udp] Initializing node...")
    rate = rospy.Rate(10)   # 10 Hz

    # Parameters
    udp_addr = rospy.get_param('~address', '')
    udp_port = rospy.get_param('~port', 12021)     # Random palindrome port
    device_frame_id = rospy.get_param('~frame_id', "")
    
    # Start UDP socket (defaults to any IP and port 12021)
    udp_in.bind((udp_addr, udp_port))

    # Publishers
    sentence_pub = rospy.Publisher("%s/nmea_sentence" % device_frame_id, Sentence, queue_size=10)
    position_pub = rospy.Publisher("%s/fix" % device_frame_id, NavSatFix, queue_size=10)
    vel_pub = rospy.Publisher("%s/vel" % device_frame_id, TwistStamped, queue_size=10)
    timeref_pub = rospy.Publisher("%s/time_reference" % device_frame_id, TimeReference, queue_size=10)
    depth_below_trans_pub = rospy.Publisher("%s/depth/below_transducer" % device_frame_id, DepthBelowTransducer, queue_size=10)
    depth_water_pub = rospy.Publisher("%s/depth/water" % device_frame_id, DepthOfWater, queue_size=10)
        
    rospy.loginfo("[nmea_depth_udp] Initialization done.")
    rospy.loginfo("[nmea_depth_udp] Published topics:")
    rospy.loginfo("[nmea_depth_udp] Sentence:\t\t\t%s/nmea_sentence" % device_frame_id)
    rospy.loginfo("[nmea_depth_udp] GPS Fix:\t\t\t%s/fix" % device_frame_id)
    rospy.loginfo("[nmea_depth_udp] GPS Velocity:\t\t%s/vel" % device_frame_id)
    rospy.loginfo("[nmea_depth_udp] Time Reference:\t\t%s/time_reference" % device_frame_id)
    rospy.loginfo("[nmea_depth_udp] Depth of Water:\t\t%s/depth/water" % device_frame_id)
    rospy.loginfo("[nmea_depth_udp] Depth below transducer:\t%s/depth/below_transducer" % device_frame_id)
    # Run node
    while not rospy.is_shutdown():
        try:
            nmea_in, _ = udp_in.recvfrom(1024)        
        except socket.error:
            pass
        ros_now = rospy.Time().now()   
        nmea_parts = nmea_in.strip().split(',')
        if len(nmea_parts):
            try:
                # GPS Fix position
                if nmea_parts[0] == '$GPGGA' and len(nmea_parts) >= 10:
                    latitude = int(nmea_parts[2][0:2])+float(nmea_parts[2][2:])/60.0
                    if nmea_parts[3] == 'S':
                        latitude = -latitude
                    longitude = int(nmea_parts[4][0:3])+float(nmea_parts[4][3:])/60.0
                    if nmea_parts[5] == 'W':
                        longitude = -longitude
                    # altitude = float(nmea_parts[9])
                    nsf = NavSatFix()
                    nsf.header.stamp = ros_now
                    nsf.header.frame_id = device_frame_id
                    nsf.latitude = latitude
                    nsf.longitude = longitude
                    # nsf.altitude = altitude
                    position_pub.publish(nsf)
                
                # Velocity
                if nmea_parts[0] == '$GPVTG' and len(nmea_parts) >= 9:
                    vel = TwistStamped()
                    vel.header.frame_id = device_frame_id
                    vel.header.stamp = ros_now
                    vel.twist.linear.x = float(nmea_parts[7]) / 3.6  # Km/h to m/s
                    vel_pub.publish(vel)
                
                # Time reference (GPST)
                if nmea_parts[0] == '$GPZDA' and len(nmea_parts) >= 5:
                    tref = TimeReference()
                    tref.header.frame_id = device_frame_id
                    tref.header.stamp = ros_now
                    hour = int(nmea_parts[1][0:2])
                    minute = int(nmea_parts[1][2:4])
                    second = int(nmea_parts[1][4:6])
                    try:
                        ms = int(float(nmea_parts[1][6:])*1000000)
                    except ValueError:
                        ms = 0
                    day = int(nmea_parts[2])
                    month = int(nmea_parts[3])
                    year = int(nmea_parts[4])
                    zda = datetime.datetime(year,month,day,hour,minute,second,ms)
                    tref.time_ref = rospy.Time(calendar.timegm(zda.timetuple()),zda.microsecond*1000)
                    tref.source = device_frame_id
                    timeref_pub.publish(tref) 
                
                # Depth (DBT - Depth Below Transducer)
                if nmea_parts[0] == '$SDDBT' and len(nmea_parts) >= 7:
                    d = DepthBelowTransducer()
                    d.header.frame_id = device_frame_id
                    d.header.stamp = ros_now
                    try:
                        d.feet = float(nmea_parts[1])  # In feet
                    except ValueError:
                        pass
                    try:
                        d.meters = float(nmea_parts[3])  # In meters
                    except ValueError:
                        pass
                    try:
                        d.fathoms = float(nmea_parts[5])  # In fathoms
                    except ValueError:
                        pass
                    depth_below_trans_pub.publish(d)

                # Depth (DPT - DePTh of water)
                if nmea_parts[0] == '$SDDPT' and len(nmea_parts) >= 4:
                    d = DepthOfWater()
                    d.header.frame_id = device_frame_id
                    d.header.stamp = ros_now
                    try:
                        d.depth = float(nmea_parts[1])  # In meters
                    except ValueError:
                        pass
                    try:
                        d.offset = float(nmea_parts[2])  # In meters
                    except ValueError:
                        pass
                    try:
                        d.range = float(nmea_parts[3])
                    except ValueError:
                        pass
                    depth_water_pub.publish(d)

                #### Other possible parsings (from Heck's provided logs)
                # GPGLL - Geographic Latitude and Longitude (legacy sentence, same info as contained in GPGGA)
                # GPGSA - Gps dillution of position and active SAtellites
                # GPGSV - Gps Satellites in View
                # GPRMC - Recommendec Minimum navigation type C (includes latitude, longitude, speed in knots, date... all info already available on other messages)
                # SDMTW - Mean Temperature of Water
                # SDVHW - Velocity and Heading in Water (Water speed in knots/kilometers-per-hour and heading in magnetic degrees)
                # SDHDG - magnetic HeaDinG (in degrees, with deviation and variation)

            except ValueError:
                pass

            # NMEA Sentence (published regardless of content)
            sentence_msg = Sentence()
            sentence_msg.header.frame_id = device_frame_id
            sentence_msg.header.stamp = ros_now
            sentence_msg.sentence = nmea_in
            sentence_pub.publish(sentence_msg)
        
        # Node sleeps for 10 Hz
        rate.sleep()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handle_sigint)     # Handles keyboard interrupt
    nmea_depth_udp()   # Run node