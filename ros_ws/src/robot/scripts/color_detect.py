#!/usr/bin/python3
import rospy
from std_srvs.srv import Trigger
from color_tracking.srv import SetTarget
from visual_processing.msg import Result

def stop():
    # Wait for the exit service (max 3 seconds)
    rospy.wait_for_service('/color_tracking/exit', timeout=3)
    # Create a service proxy to call the exit service
    exit_srv = rospy.ServiceProxy('/color_tracking/exit', Trigger)
    # Call the service to stop color tracking
    resp = exit_srv()
    print("Called /color_tracking/exit, response:", resp)

def result_callback(msg):
    # Callback function that runs when detection results are received
    # Print detected object's center coordinates and radius (size)
    print("Detected block center_x: {}, center_y: {}, radius: {}".format(
        msg.center_x, msg.center_y, msg.data))

# Initialize ROS node named 'color_detect'
rospy.init_node('color_detect')

# Register shutdown function to properly stop tracking when node exits
rospy.on_shutdown(stop)

# Wait for and call the service to enter (start) color tracking mode
rospy.wait_for_service('/color_tracking/enter')
enter_srv = rospy.ServiceProxy('/color_tracking/enter', Trigger)
enter_srv()

# Wait for and call the service to set the target color
rospy.wait_for_service('/color_tracking/set_target')
set_target_srv = rospy.ServiceProxy('/color_tracking/set_target', SetTarget)
set_target_srv("red")  # Set tracking target to red color

# Subscribe to detection result topic
rospy.Subscriber("/visual_processing/result", Result, result_callback)

# Keep the node running and processing incoming messages
rospy.spin()
