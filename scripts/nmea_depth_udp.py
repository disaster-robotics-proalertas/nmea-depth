#!/usr/bin/env python

from ros_nmea_depth.msg import DepthBelowTransducer, DepthOfWater
from nmea_msgs.msg import Sentence, Gpgsa, Gpgsv, GpgsvSatellite
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
udp_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Callback to handle keyboard interruption
def handle_sigint(sig, frame):
    udp_in.close()      # Close UDP port
    rospy.loginfo("[nmea_depth_udp] Quitting...")
    rospy.signal_shutdown(reason="Interrupted")     # Shutdown ROS node

# Utility function to avoid "ValueError" everytime we have to cast a float
def cast_float(value):
    try:
        return float(value)
    except ValueError:
        return float('nan')

# Utility function to avoid "ValueError" everytime we have to cast an integer
def cast_int(value):
    try:
        return int(value)
    except ValueError:
        return -1

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

    # NMEA Sentence publisher (to publish NMEA sentence regardless of content)
    sentence_pub = rospy.Publisher("%s/nmea_sentence" % device_frame_id, Sentence, queue_size=10)
    
    # GPS publishers
    # GPGGA - Position
    # GPVTG - Velocity Track Good (ground speed)
    # GPZDA - Time reference (GPST)
    # GPGSA - Active Satellites
    # GPGSV - Satellites in View
    position_pub = rospy.Publisher("%s/gps/fix" % device_frame_id, NavSatFix, queue_size=10)
    vel_pub = rospy.Publisher("%s/gps/vel" % device_frame_id, TwistStamped, queue_size=10)
    timeref_pub = rospy.Publisher("%s/gps/time_reference" % device_frame_id, TimeReference, queue_size=10)
    active_sat_pub = rospy.Publisher("%s/gps/active_satellites" % device_frame_id, Gpgsa, queue_size=10)
    sat_view_pub = rospy.Publisher("%s/gps/satellites_in_view" % device_frame_id, Gpgsv, queue_size=10)

    # Sidescanner publishers
    # SDDBT - Depth Below Transducer
    # SDDPT - Depth of Water
    # SDMTW - Mean Temperature of Water
    # SDVHW - Velocity and heading in Water
    # SDHDG - Magnetic heading
    depth_below_trans_pub = rospy.Publisher("%s/scanner/depth_below_transducer" % device_frame_id, DepthBelowTransducer, queue_size=10)
    depth_water_pub = rospy.Publisher("%s/scanner/depth_of_water" % device_frame_id, DepthOfWater, queue_size=10)
    temp_water_pub = rospy.Publisher("%s/scanner/water_temperature" % device_frame_id, DepthOfWater, queue_size=10)
    water_velocity_pub = rospy.Publisher("%s/scanner/water_velocity" % device_frame_id, DepthOfWater, queue_size=10)
    mag_heading_pub = rospy.Publisher("%s/scanner/magnetic_heading" % device_frame_id, DepthOfWater, queue_size=10)

    rospy.loginfo("[nmea_depth_udp] Initialization done.")
    # rospy.loginfo("[nmea_depth_udp] Published topics:")
    # rospy.loginfo("[nmea_depth_udp] Sentence:\t\t\t%s/nmea_sentence" % device_frame_id)
    # rospy.loginfo("[nmea_depth_udp] GPS Fix:\t\t\t%s/fix" % device_frame_id)
    # rospy.loginfo("[nmea_depth_udp] GPS Velocity:\t\t%s/vel" % device_frame_id)
    # rospy.loginfo("[nmea_depth_udp] Time Reference:\t\t%s/time_reference" % device_frame_id)
    # rospy.loginfo("[nmea_depth_udp] Depth of Water:\t\t%s/depth/water" % device_frame_id)
    # rospy.loginfo("[nmea_depth_udp] Depth below transducer:\t%s/depth/below_transducer" % device_frame_id)
    # Run node
    while not rospy.is_shutdown():
        try:
            nmea_in, _ = udp_in.recvfrom(1024)        
        except socket.error:
            pass
        ros_now = rospy.Time().now()   
        nmea_parts = nmea_in.strip().split(',')

        if len(nmea_parts):
            # GPS Fix position
            if nmea_parts[0] == '$GPGGA' and len(nmea_parts) >= 10:
                latitude = cast_float(nmea_parts[2][0:2])+cast_float(nmea_parts[2][2:])/60.0
                if nmea_parts[3] == 'S':
                    latitude = -latitude
                longitude = cast_float(nmea_parts[4][0:3])+cast_float(nmea_parts[4][3:])/60.0
                if nmea_parts[5] == 'W':
                    longitude = -longitude
                altitude = cast_float(nmea_parts[9])
                nsf = NavSatFix()
                nsf.header.stamp = ros_now
                nsf.header.frame_id = device_frame_id
                nsf.latitude = latitude
                nsf.longitude = longitude
                nsf.altitude = altitude
                position_pub.publish(nsf)
            
            # Velocity
            if nmea_parts[0] == '$GPVTG' and len(nmea_parts) >= 9:
                vel = TwistStamped()
                vel.header.frame_id = device_frame_id
                vel.header.stamp = ros_now
                vel.twist.linear.x = cast_float(nmea_parts[7]) / 3.6  # Km/h to m/s
                vel_pub.publish(vel)
            
            # Time reference (GPST)
            if nmea_parts[0] == '$GPZDA' and len(nmea_parts) >= 5:
                tref = TimeReference()
                tref.header.frame_id = device_frame_id
                tref.header.stamp = ros_now
                hour = cast_int(nmea_parts[1][0:2])
                minute = cast_int(nmea_parts[1][2:4])
                second = cast_int(nmea_parts[1][4:6])
                try:
                    ms = int(float(nmea_parts[1][6:])*1000000)
                except ValueError:
                    ms = 0
                day = cast_int(nmea_parts[2])
                month = cast_int(nmea_parts[3])
                year = cast_int(nmea_parts[4])
                zda = datetime.datetime(year,month,day,hour,minute,second,ms)
                tref.time_ref = rospy.Time(calendar.timegm(zda.timetuple()),zda.microsecond*1000)
                tref.source = device_frame_id
                timeref_pub.publish(tref) 
            
            # GPS DOP and active satellites
            if nmea_parts[0] == '$GPGSA' and len(nmea_parts) >= 18:
                gsa = Gpgsa()
                gsa.header.frame_id = device_frame_id
                gsa.header.stamp = ros_now
                gsa.auto_manual_mode = nmea_parts[1]
                gsa.fix_mode = cast_int(nmea_parts[2])
                satellite_list = []
                for x in nmea_parts[3:14]:
                    try:
                        satellite_list.append(int(x))
                    except ValueError:
                        break
                gsa.sv_ids = satellite_list
                gsa.pdop = cast_float(nmea_parts[15])
                gsa.hdop = cast_float(nmea_parts[16])
                gsa.vdop = cast_float(nmea_parts[17])
                active_sat_pub.publish(gsa)

            # GPS Satellites in View
            if nmea_parts[0] == '$GPGSV' and len(nmea_parts) >= 7:
                # Typically GPGSV messages come in sequences, run and obtain messages from UDP until last message in sequence arrives
                while True:
                    gsv = Gpgsv()
                    gsv.header.frame_id = device_frame_id
                    gsv.header.stamp = ros_now
                    gsv.n_msgs = cast_int(nmea_parts[1])
                    gsv.msg_number = cast_int(nmea_parts[2])
                    gsv.n_satellites = cast_int(nmea_parts[3])
                    i = 4   # Satellite information comes in quadruples until sentence end
                    while i < len(nmea_parts):
                        gsv_sat = GpgsvSatellite()
                        try:
                            gsv_sat.prn = int(nmea_parts[i])
                        except ValueError:
                            pass
                        try:
                            gsv_sat.elevation = int(nmea_parts[i+1])
                        except ValueError:
                            pass
                        try:
                            gsv_sat.azimuth = int(nmea_parts[i+2])
                        except ValueError:
                            pass
                        try:
                            gsv_sat.snr = int(nmea_parts[i+3].split("*")[0])
                        except ValueError:
                            pass

                        gsv.satellites.append(gsv_sat)
                        i += 4

                    sat_view_pub.publish(gsv)

                    # If this message is the last in sequence, break
                    if gsv.n_msgs == gsv.msg_number:
                        break
                    # If not, obtain next NMEA sentence
                    else:
                        try:
                            nmea_in, _ = udp_in.recvfrom(1024)        
                        except socket.error:
                            pass
                        ros_now = rospy.Time().now()   
                        nmea_parts = nmea_in.strip().split(',')
            
            # Depth (DBT - Depth Below Transducer)
            if nmea_parts[0] == '$SDDBT' and len(nmea_parts) >= 7:
                d = DepthBelowTransducer()
                d.header.frame_id = device_frame_id
                d.header.stamp = ros_now
                try:
                    d.feet = cast_float(nmea_parts[1])  # In feet
                except ValueError:
                    pass
                try:
                    d.meters = cast_float(nmea_parts[3])  # In meters
                except ValueError:
                    pass
                try:
                    d.fathoms = cast_float(nmea_parts[5])  # In fathoms
                except ValueError:
                    pass
                depth_below_trans_pub.publish(d)

            # Depth (DPT - DePTh of water)
            if nmea_parts[0] == '$SDDPT' and len(nmea_parts) >= 4:
                d = DepthOfWater()
                d.header.frame_id = device_frame_id
                d.header.stamp = ros_now
                try:
                    d.depth = cast_float(nmea_parts[1])  # In meters
                except ValueError:
                    pass
                try:
                    d.offset = cast_float(nmea_parts[2])  # In meters
                except ValueError:
                    pass
                try:
                    d.range = cast_float(nmea_parts[3])
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