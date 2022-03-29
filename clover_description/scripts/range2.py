import rospy
import threading
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range

a = 0

def callback(data):
    global a
    a = data.range

    

    # u = data

    # print(type(data))

    
    
print(a)

rospy.init_node('flight')

# a = rospy.loginfo(data.range)


def listener():

    rospy.Subscriber("mavros/ultrasonic_sensor_1/range", Range, callback)
    rospy.Subscriber("mavros/ultrasonic_sensor_2/range", Range, callback)
    rospy.Subscriber("mavros/ultrasonic_sensor_3/range", Range, callback)
    rospy.Subscriber("mavros/ultrasonic_sensor_4/range", Range, callback)

    # b = Range().range

    # print(a)
    
    rospy.spin()