#!/usr/bin/env python

# ROS messages and libraries
from ros_nmea_depth.msg import DepthBelowTransducer, DepthOfWater, WaterHeadingSpeed, MagneticHeading
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import NavSatFix, TimeReference, Temperature
from nmea_msgs.msg import Sentence, Gpgsa, Gpgsv, GpgsvSatellite
from geometry_msgs.msg import TwistStamped, Quaternion
import rosparam
import rospy
from tf.transformations import quaternion_from_euler

# Python utilities
import datetime
import calendar
import signal
import socket
import sys

# TCP socket
tcp_in = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Callback to handle keyboard interruption
def handle_sigint(sig, frame):
    tcp_in.close()      # Close tcp port
    rospy.loginfo("[nmea_depth_tcp] Quitting...")
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

def nmea_depth_tcp():
    # Init node
    system_name = socket.gethostname()
    rospy.init_node("lowrance_sonar", anonymous=True)
    rospy.loginfo("[nmea_depth_tcp] Initializing node...")

    # Parameters
    tcp_addr = rospy.get_param('~address', '127.0.0.1')
    tcp_port = rospy.get_param('~port', 10110)     # Lowrance standard port
    update_rate = rospy.get_param('~update_rate', 40)   # Measurement comm rate for Lowrance (Hz)
    
    # Connect TCP client to destination
    try:
        tcp_in.connect((tcp_addr, tcp_port))
    except IOError as exp:
        rospy.logerr("Socket error: %s" % exp.strerror)
        rospy.signal_shutdown(reason="Socket error: %s" % exp.strerror)
        sys.exit(0)

    # NMEA Sentence publisher (to publish NMEA sentence regardless of content)
    sentence_pub = rospy.Publisher("%s/sonar/nmea_sentence" % system_name, Sentence, queue_size=10)
    
    # GPS publishers
    # GPGGA - Position
    # GPVTG - Velocity Track Good (ground speed)
    # GPZDA - Time reference (GPST)
    # GPGSA - Active Satellites
    # GPGSV - Satellites in View
    position_pub = rospy.Publisher("%s/sonar/gps/fix" % system_name, NavSatFix, queue_size=10)
    vel_pub = rospy.Publisher("%s/sonar/gps/vel" % system_name, TwistStamped, queue_size=10)
    timeref_pub = rospy.Publisher("%s/sonar/gps/time_reference" % system_name, TimeReference, queue_size=10)
    active_sat_pub = rospy.Publisher("%s/sonar/gps/active_satellites" % system_name, Gpgsa, queue_size=10)
    sat_view_pub = rospy.Publisher("%s/sonar/gps/satellites_in_view" % system_name, Gpgsv, queue_size=10)

    # Sidescanner publishers
    # SDDBT - Depth Below Transducer
    # SDDPT - Depth of Water
    # SDMTW - Mean Temperature of Water
    # SDVHW - Velocity and heading in Water
    # SDHDG - Magnetic heading
    depth_below_trans_pub = rospy.Publisher("%s/sonar/scanner/water/depth_below_transducer" % system_name, DepthBelowTransducer, queue_size=10)
    depth_water_pub = rospy.Publisher("%s/sonar/scanner/water/depth" % system_name, DepthOfWater, queue_size=10)
    temp_water_pub = rospy.Publisher("%s/sonar/scanner/water/temperature" % system_name, Temperature, queue_size=10)
    water_heading_speed_pub = rospy.Publisher("%s/sonar/scanner/water/heading_and_speed" % system_name, WaterHeadingSpeed, queue_size=10)
    mag_heading_pub = rospy.Publisher("%s/sonar/scanner/magnetic_heading" % system_name, MagneticHeading, queue_size=10)

    # Diagnostics publisher
    diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)

    rate = rospy.Rate(update_rate)   # Defines the publish rate
    
    rospy.loginfo("[nmea_depth_tcp] Initialization done.")
    # rospy.loginfo("[nmea_depth_tcp] Published topics:")
    # rospy.loginfo("[nmea_depth_tcp] Sentence:\t\t\t%s/nmea_sentence" % system_name)
    # rospy.loginfo("[nmea_depth_tcp] GPS Fix:\t\t\t%s/fix" % system_name)
    # rospy.loginfo("[nmea_depth_tcp] GPS Velocity:\t\t%s/vel" % system_name)
    # rospy.loginfo("[nmea_depth_tcp] Time Reference:\t\t%s/time_reference" % system_name)
    # rospy.loginfo("[nmea_depth_tcp] Depth of Water:\t\t%s/depth/water" % system_name)
    # rospy.loginfo("[nmea_depth_tcp] Depth below transducer:\t%s/depth/below_transducer" % system_name)
    # Run node
    last_update = 0
    while not rospy.is_shutdown():
        try:
            nmea_in = tcp_in.makefile().readline()        
        except socket.error:
            pass
        nmea_parts = nmea_in.strip().split(',')

        ros_now = rospy.Time().now()
        diag_msg = DiagnosticArray()
        diag_msg.status.append(DiagnosticStatus())
        diag_msg.status[0].name = 'sonar'
        diag_msg.status[0].hardware_id = '%s' % system_name
        if len(nmea_parts):
            #### GPS
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
                nsf.header.frame_id = system_name
                nsf.latitude = latitude
                nsf.longitude = longitude
                nsf.altitude = altitude
                position_pub.publish(nsf)
            
            # Velocity
            if nmea_parts[0] == '$GPVTG' and len(nmea_parts) >= 9:
                vel = TwistStamped()
                vel.header.frame_id = system_name
                vel.header.stamp = ros_now
                vel.twist.linear.x = cast_float(nmea_parts[7]) / 3.6  # Km/h to m/s
                vel_pub.publish(vel)
            
            # Time reference (GPST)
            if nmea_parts[0] == '$GPZDA' and len(nmea_parts) >= 5:
                tref = TimeReference()
                tref.header.frame_id = system_name
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
                try:
                    zda = datetime.datetime(year,month,day,hour,minute,second,ms)
                    tref.time_ref = rospy.Time(calendar.timegm(zda.timetuple()),zda.microsecond*1000)
                except ValueError:
                    pass
                
                tref.source = system_name
                timeref_pub.publish(tref) 
            
            # GPS DOP and active satellites
            if nmea_parts[0] == '$GPGSA' and len(nmea_parts) >= 18:
                gsa = Gpgsa()
                gsa.header.frame_id = system_name
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
                gsv = Gpgsv()
                gsv.header.frame_id = system_name
                gsv.header.stamp = ros_now
                gsv.n_msgs = cast_int(nmea_parts[1])
                gsv.msg_number = cast_int(nmea_parts[2])
                gsv.n_satellites = cast_int(nmea_parts[3])
                for i in range(4, len(nmea_parts), 4):
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
                    
                sat_view_pub.publish(gsv)
            
            #### Side-scanner
            # Depth (DBT - Depth Below Transducer)
            if nmea_parts[0] == '$SDDBT' and len(nmea_parts) >= 7:
                d = DepthBelowTransducer()
                d.header.frame_id = system_name
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
                d.header.frame_id = system_name
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

            # SDMTW - Mean Temperature of Water
            if nmea_parts[0] == '$SDMTW' and len(nmea_parts) >= 3:
                tempC = Temperature()
                tempC.header.frame_id = system_name
                tempC.header.stamp = ros_now
                tempC.temperature = cast_float(nmea_parts[1])
                temp_water_pub.publish(tempC)

            # SDVHW - Water Heading and Speed
            if nmea_parts[0] == '$SDVHW' and len(nmea_parts) >= 9:
                whs = WaterHeadingSpeed()
                whs.header.frame_id = system_name
                whs.header.stamp = ros_now
                whs.true_heading = cast_float(nmea_parts[1])
                whs.mag_heading = cast_float(nmea_parts[3])
                whs.knots = cast_float(nmea_parts[5])
                whs.kmph = cast_float(nmea_parts[7])
                whs.mps = whs.kmph / 3.6               # Km/h to m/s
                water_heading_speed_pub.publish(whs)

            # SDHDG - Magnetic heading
            if nmea_parts[0] == '$SDHDG' and len(nmea_parts) >= 6:
                hdg = MagneticHeading()
                hdg.header.frame_id = system_name
                hdg.header.stamp = ros_now
                hdg.heading = cast_float(nmea_parts[1])
                hdg.mag_dev = cast_float(nmea_parts[2])
                hdg.mag_dev_dir = nmea_parts[3]
                hdg.mag_var = cast_float(nmea_parts[4])
                hdg.mag_var_dir = nmea_parts[5].split('*')[0]
                quat = quaternion_from_euler(0.0, 0.0, cast_float(hdg.heading))
                hdg.quaternion.x = quat[0]
                hdg.quaternion.y = quat[1]
                hdg.quaternion.z = quat[2]
                hdg.quaternion.w = quat[3]
                mag_heading_pub.publish(hdg)
                

            # NMEA Sentence (published regardless of content)
            sentence_msg = Sentence()
            sentence_msg.header.frame_id = system_name
            sentence_msg.header.stamp = ros_now
            sentence_msg.sentence = nmea_in
            sentence_pub.publish(sentence_msg)

            diag_msg.status[0].level = DiagnosticStatus.OK
            diag_msg.status[0].message = 'OK'
            diag_msg.status[0].values = [KeyValue(key="Sentence", value=sentence_msg.sentence)]
            last_update = ros_now

        # Check for stale status
        elapsed = ros_now.to_sec() - last_update.to_sec()
        if elapsed > 35:
            diag_msg.status[0].level = DiagnosticStatus.STALE
            diag_msg.status[0].message = 'Stale'
            diag_msg.status[0].values = [KeyValue(key="Update Status", value='Stale'),
                                         KeyValue(key="Time Since Update", value=str(elapsed))]

        # Publish diagnostics message
        diag_pub.publish(diag_msg)

        # Node sleeps for some time
        rate.sleep()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handle_sigint)     # Handles keyboard interrupt
    nmea_depth_tcp()   # Run node
