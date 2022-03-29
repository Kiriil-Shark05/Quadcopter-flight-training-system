import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn as rc

pubRC = None
msg = None
neutral_speed = 1500

def convert_joy_units(data):
    #This takes the float value from -1.0 to 1.0 and converts it to a value between 1100 and 1900
    return int((data * 400) + neutral_speed)

def joy_callback(data):
    global pubRC, msg
    (yaw, pitch, _, throttle, _, _) = data.axes
    msg = rc()
    msg.channels[0] = neutral_speed
    msg.channels[1] = neutral_speed
    msg.channels[2] = convert_joy_units(pitch)
    msg.channels[3] = convert_joy_units(yaw*-1)
    msg.channels[4] = convert_joy_units(throttle)

def init_joy_control():
    global pubRC
    pubRC = rospy.Publisher('mavros/rc/override', rc, queue_size=10)
    rospy.Subscriber("/joy", Joy, joy_callback)

if __name__ == "__main__":
    init_joy_control()