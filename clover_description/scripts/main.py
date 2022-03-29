import rospy
import time
import math
from clover import srv
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger



get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
land = rospy.ServiceProxy('land', Trigger)

rospy.init_node('flight')

coefficient = 0.02 # Obstacle detection coefficient

desiredDistance = 0.7  # The desired distance that will be between the quadcopter and the obstacle

# Ultrasonic rangefinders

leftRange = 0
rightRange = 0
frontRange = 0
backRange = 0

# Quadcopter pose

poseX = 0
poseY = 0

# Quadcopter mode

mode = ""
firstMode = ""
modeFlag = True

# Counting loop iterations

countingIteration = 0
firstFrontCountingIteration = 0
firstBackCountingIteration = 0
firstRightCountingIteration = 0
firstLeftCountingIteration = 0

# Variables that will be assigned in the loop

firstValueFrontRange = 0
firstValueBackRange = 0
firstValueRightRange = 0
firstValueLeftRange = 0

firstValueFrontPosition = 0
firstValueBackPosition = 0
firstValueRightPosition = 0
firstValueLeftPosition = 0

secondValueFrontRange = 0
secondValueBackRange = 0
secondValueRightRange = 0
secondValueLeftRange = 0

secondValueFrontPosition = 0
secondValueBackPosition = 0
secondValueRightPosition = 0
secondValueLeftPosition = 0

# Flags

firstValueFrontFlag = True
firstValueBackFlag = True
firstValueRightFlag = True
firstValueLeftFlag = True

# Ros subscribers callbacks

def callback(data):
    global leftRange
    leftRange = data.range
        
def callback2(data):
    global rightRange
    rightRange = data.range

def callback3(data):
    global frontRange
    frontRange = data.range
    # print(frontRange)

def callback4(data):
    global backRange
    backRange = data.range

def callback5(data):
    global poseX
    global poseY
    poseX = data.pose.position.x
    poseY = data.pose.position.y

def callback6(data):
    global mode
    mode = data.mode




def main():

    # Ros subscribers

    rospy.Subscriber("ultrasonic_sensor_left/range", Range, callback)
    rospy.Subscriber("ultrasonic_sensor_right/range", Range, callback2)
    rospy.Subscriber("ultrasonic_sensor_front/range", Range, callback3)
    rospy.Subscriber("ultrasonic_sensor_back/range", Range, callback4)

    rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback5)
    rospy.Subscriber("mavros/state", State, callback6)

    # Program loop

    rate = rospy.Rate(100) # Fixed update frequency of 100hz
    while not rospy.is_shutdown():

        global coefficient
        global leftRange
        global rightRange
        global frontRange
        global backRange
        global poseX
        global poseY
        global mode
        global modeFlag
        global firstMode
        global firstValueFrontRange
        global firstValueBackRange
        global firstValueRightRange
        global firstValueLeftRange

        global firstValueFrontPosition
        global firstValueBackPosition
        global firstValueRightPosition
        global firstValueLeftPosition

        global secondValueFrontRange
        global secondValueBackRange
        global secondValueRightRange
        global secondValueLeftRange

        global secondValueFrontPosition
        global secondValueBackPosition
        global secondValueRightPosition
        global secondValueLeftPosition
        
        global firstValueFrontFlag
        global firstValueBackFlag
        global firstValueRightFlag
        global firstValueLeftFlag

        global desiredDistance
        global countingIteration
        global firstFrontCountingIteration
        global firstBackCountingIteration
        global firstRightCountingIteration
        global firstLeftCountingIteration

        countingIteration = countingIteration + 1

        # Determing the current flight mode

        if (modeFlag == True) and (countingIteration > 5):
            firstMode = mode
            modeFlag = False

        # Checking front sensor

        if firstValueFrontFlag == True:
            firstValueFrontPosition = poseX
            firstValueFrontRange = frontRange
            firstFrontCountingIteration = countingIteration
            firstValueFrontFlag = False

        if countingIteration > firstFrontCountingIteration + 10:
            secondValueFrontPosition = poseX
            secondValueFrontRange = frontRange
            firstValueFrontFlag = True


        # Checking back sensor

        if firstValueBackFlag == True:
            firstValueBackPosition = poseX
            firstValueBackRange = frontRange
            firstBackCountingIteration = countingIteration
            firstValueBackFlag = False

        if countingIteration > firstBackCountingIteration + 10:
            secondValueBackPosition = poseX
            secondValueBackRange = backRange
            firstValueBackFlag = True

        # Checking right sensor

        if firstValueRightFlag == True:
            firstValueRightPosition = poseX
            firstValueRightRange = rightRange
            firstRightCountingIteration = countingIteration
            firstValueRightFlag = False

        if countingIteration > firstRightCountingIteration + 10:
            secondValueRightPosition = poseY
            secondValueRightRange = rightRange
            firstValueRightFlag = True

        # Checking left sensor

        if firstValueLeftFlag == True:
            firstValueLeftPosition = poseX
            firstValueLeftRange = leftRange
            firstLeftCountingIteration = countingIteration
            firstValueLeftFlag = False

        if countingIteration > firstLeftCountingIteration + 10:
            secondValueLeftPosition = poseY
            secondValueLeftRange = leftRange
            firstValueLeftFlag = True

        # Detection front obstacles

        if ((abs(secondValueFrontPosition) - abs(firstValueFrontPosition)) > coefficient and (abs(secondValueFrontPosition) - abs(firstValueFrontPosition)) < 0.061) and ((firstValueFrontRange - secondValueFrontRange) > coefficient and (firstValueFrontRange - secondValueFrontRange) < 0.061) and frontRange < desiredDistance + 0.40:
            set_velocity(vx=0, vy=0.0, vz=0, frame_id='body', auto_arm=True)
            rospy.sleep(2)
            print("You almost crashed into the front obstacle. Fly more carefully")
            modeService(custom_mode=firstMode)

        if ((abs(secondValueFrontPosition) - abs(firstValueFrontPosition)) > 0.061) and ((firstValueFrontRange - secondValueFrontRange) > 0.061) and frontRange < desiredDistance + 0.5:
            set_velocity(vx=0, vy=0.0, vz=0, frame_id='body', auto_arm=True)
            rospy.sleep(2)
            print("You almost crashed into the front obstacle. Fly more carefully")
            modeService(custom_mode=firstMode)


         # Detection back obstacles

        if ((abs(secondValueBackPosition) - abs(firstValueBackPosition)) > coefficient and (abs(secondValueBackPosition) - abs(firstValueBackPosition)) < 0.061) and ((firstValueBackRange - secondValueBackRange) > coefficient and (firstValueBackRange - secondValueBackRange) < 0.061) and backRange < desiredDistance + 0.40:
            set_velocity(vx=0, vy=0.0, vz=0, frame_id='body', auto_arm=True)
            rospy.sleep(2)
            print("You almost crashed into the back obstacle. Fly more carefully")
            modeService(custom_mode=firstMode)

        if ((abs(secondValueBackPosition) - abs(firstValueBackPosition)) > 0.061) and ((firstValueBackRange - secondValueBackRange) > 0.061) and backRange < desiredDistance + 0.5:
            set_velocity(vx=0, vy=0.0, vz=0, frame_id='body', auto_arm=True)
            rospy.sleep(2)
            print("You almost crashed into the back obstacle. Fly more carefully")
            modeService(custom_mode=firstMode)

        # Detection right obstacles

        if ((abs(secondValueRightPosition) - abs(firstValueRightPosition)) > coefficient and (abs(secondValueRightPosition) - abs(firstValueRightPosition)) < 0.061) and ((firstValueRightRange - secondValueRightRange) > coefficient and (firstValueRightRange - secondValueRightRange) < 0.061) and rightRange < desiredDistance + 0.40:
            set_velocity(vx=0, vy=0.0, vz=0, frame_id='body', auto_arm=True)
            rospy.sleep(2)
            print("You almost crashed into the right obstacle. Fly more carefully")
            modeService(custom_mode=firstMode)

        if ((abs(secondValueRightPosition) - abs(firstValueRightPosition)) > 0.061) and ((firstValueRightRange - secondValueRightRange) > 0.061) and rightRange < desiredDistance + 0.5:
            set_velocity(vx=0, vy=0.0, vz=0, frame_id='body', auto_arm=True)
            rospy.sleep(2)
            print("You almost crashed into the right obstacle. Fly more carefully")
            modeService(custom_mode=firstMode)

        # Detection left obstacles

        if ((abs(secondValueLeftPosition) - abs(firstValueLeftPosition)) > coefficient and (abs(secondValueLeftPosition) - abs(firstValueLeftPosition)) < 0.061) and ((firstValueLeftRange - secondValueLeftRange) > coefficient and (firstValueLeftRange - secondValueLeftRange) < 0.061) and leftRange < desiredDistance + 0.40:
            set_velocity(vx=0, vy=0.0, vz=0, frame_id='body', auto_arm=True)
            rospy.sleep(2)
            print("You almost crashed into the left obstacle. Fly more carefully")
            modeService(custom_mode=firstMode)

        if ((abs(secondValueLeftPosition) - abs(firstValueLeftPosition)) > 0.061) and ((firstValueLeftRange - secondValueLeftRange) > 0.061) and leftRange < desiredDistance + 0.5:
            set_velocity(vx=0, vy=0.0, vz=0, frame_id='body', auto_arm=True)
            rospy.sleep(2)
            print("You almost crashed into the left obstacle. Fly more carefully")
            modeService(custom_mode=firstMode)

        rate.sleep() 



if __name__ == '__main__':
    main()