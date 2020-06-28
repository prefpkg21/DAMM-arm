#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
from sensor_msgs.msg import Joy
#from dynamixel_workbench_toolbox import dynamixel_workbench
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import MXExt, MX, DynamixelStateList

move_strength0 = 300
move_strength1 = 100
move_strength_linear = 1.0

DXL0_ID = 0
DXL0_MIN = 730
DXL0_MAX = 3360

DXL1_ID = 1
DXL1_MIN = 727
DXL1_MAX = 1400

DXL2_ID = 2
DXL2_MIN = 730
DXL2_MAX = 3360
CUT_LEFT = 800
CUT_RIGHT = 3000


DXL0_HOME = 2040
DXL1_HOME = 2040
CUT_HOME = 1530

# Data Byte Length
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

position_goal = 'Goal_Position'

# Default setting

BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyACM0".encode('utf-8')        # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

output_pins = {
    'JETSON_XAVIER': 18,
    'JETSON_NANO': 33,
    'JETSON_NX': 32,
}
output_pin = output_pins.get(GPIO.model, None)
if output_pin is None:
    raise Exception('PWM not supported on this board')


class MoveArm():
    def __init__(self):
        #self.turret_aim_publisher = rospy.Publisher('/dynamixel_workbench/dynamixel_command', MX, queue_size=2)
        self.cmd_subscriber = rospy.Subscriber('/bluetooth_teleop/joy',Joy, self.handle_joystick, queue_size=2)
        self.servo_position_updater = rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, self.update_current_position, queue_size=1)
        self.dyna_cmd_srvproxy = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command',DynamixelCommand)
        self.ctrl_c= False
        rospy.on_shutdown(self.shutdownhook)

        self.rate = rospy.Rate(1)
        self.dxl0_pos =  9999
	self.dxl1_pos =  9999
        self.dxl2_pos = 9999
        self.prev_dxl0 = 2050
        self.prev_dxl1 = 2040
        self.prev_dxl2 = 1530
        self.dyna_cmd_srvproxy('',0,position_goal, DXL0_HOME)
        self.dyna_cmd_srvproxy('',1,position_goal, DXL1_HOME)
	self.dyna_cmd_srvproxy('',2,position_goal, CUT_HOME)
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')

	#PWM Setup
	self.linsrvo_val = 50
	self.p = GPIO.PWM(output_pin, 50)
	self.p.start(self.linsrvo_val)

    # def publish_once_in_dyna_command(self, cmd):
    #     while not self.ctrl_c:
    #         connections = self.servo_position_updater.get_num_connections()
    #         if connections > 0:
    #             self.turret_aim_publisher.publish(cmd)
    #             rospy.loginfo("Turret Pan Publisher")
    #             break
    #         else:
    #             self.rate.sleep()


    def shutdownhook(self):
	self.p.stop()
        GPIO.cleanup()
        self.ctrl_c = True


    
    def handle_joystick(self, data):

	#check to see if current position read by callback
	if self.dxl0_pos == 9999 or self.dxl1_pos == 9999:
		return

	# Simple check for PS4 driver to set initial value
	#   for the right and left triggers
	if data.axes[2] == 0 and data.axes[4] == 0:
		return
	# Read joystick message values 
        dxl0_scale = data.axes[2] # right thumb horiz
	right = data.axes[4] - 1  #right trigger
	left = data.axes[3] - 1  #left trigger
	dxl1_scale =  left - right

	dxl2_cut = data.axes[6] # normally 0 or 1/-1 pressed left/right d-pad

	linsrvo_chng = data.axes[5]  #right thumb vertical
	val = self.linsrvo_val + (move_strength_linear * linsrvo_chng)
	# PWM on JetsonNX 0 to 100 percent
	if val > 100:
		val = 100
	elif val < 0:
		val = 0
	else:
	    self.linsrvo_val = val
        #rospy.loginfo(val)

	#change linear PWM signal
	self.p.ChangeDutyCycle(self.linsrvo_val)

        #set the new position within the Max/Min boundaries
	new_dxl0 = max(min((self.dxl0_pos + (dxl0_scale * move_strength0)), DXL0_MAX),DXL0_MIN)
        new_dxl1 = max(min(self.dxl1_pos + (dxl1_scale * move_strength1), DXL1_MAX), DXL1_MIN)

        dxl0_diff = abs(self.dxl0_pos - new_dxl0)
        dxl1_diff = abs(self.dxl1_pos - new_dxl1)

        if dxl0_diff >= 10:
            self.dyna_cmd_srvproxy.call('',DXL0_ID, position_goal, new_dxl0)

        if dxl1_diff >= 10:
            self.dyna_cmd_srvproxy.call('',DXL1_ID, position_goal, new_dxl1)
        
        if dxl2_cut == 1:  #left
	    self.dyna_cmd_srvproxy.call('',DXL2_ID, position_goal, CUT_LEFT)
	elif dxl2_cut == -1:  #right
	    self.dyna_cmd_srvproxy.call('',DXL2_ID, position_goal, CUT_RIGHT)
	else:
	    self.dyna_cmd_srvproxy.call('',DXL2_ID, position_goal, CUT_HOME)
	    #rospy.loginfo("Cut Home")
        
        

    def update_current_position(self, msg):
        self.dxl0_pos = msg.dynamixel_state[0].present_position
        #self.pan_vel = msg.dynamixel_state[0].present_velocity
        self.dxl1_pos = msg.dynamixel_state[1].present_position
        #self.tilt_vel = msg.dynamixel_state[1].present_velocity

        #rospy.loginfo("dxl0_pos: %s dxl1_pos: %s", self.dxl0_pos, self.dxl1_pos)
        

if __name__ == "__main__":
    rospy.init_node('turret_aim', anonymous=True)
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    turret = MoveArm()

    while not rospy.is_shutdown():
        rospy.spin()
