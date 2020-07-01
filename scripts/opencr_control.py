#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg  import Float32MultiArray

move_strength0 = 300
move_strength1 = 100
move_strength_linear = 1.0

DXL0_ID = 1
DXL0_MIN = 730
DXL0_MAX = 3360

DXL1_ID = 2
DXL1_MIN = 727
DXL1_MAX = 1400

DXL2_ID = 3
DXL2_MIN = 730
DXL2_MAX = 3360
CUT_LEFT = 800
CUT_RIGHT = 3000


DXL0_HOME = 2040
DXL1_HOME = 2040
CUT_HOME = 1530


# Default setting

BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyACM0".encode('utf-8')        # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"


class MoveArm():
    def __init__(self):
        #self.turret_aim_publisher = rospy.Publisher('/dynamixel_workbench/dynamixel_command', MX, queue_size=2)
        self.cmd_subscriber = rospy.Subscriber('/bluetooth_teleop/joy',Joy, self.handle_joystick, queue_size=2)
	self.goal_pub = rospy.Publisher('damm_goal', Float32MultiArray, queue_size=1)
        #self.servo_position_updater = rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, self.update_current_position, queue_size=1)
        
        self.ctrl_c= False
        rospy.on_shutdown(self.shutdownhook)

        self.rate = rospy.Rate(10)
	self.prev_stamp = 0
        self.dxl0_pos =  9999
	self.dxl1_pos =  9999
        self.dxl2_pos = 9999
        self.prev_dxl0 = 2050
        self.prev_dxl1 = 2040
        self.prev_dxl2 = 1530

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
        self.ctrl_c = True


    
    def handle_joystick(self, data):
	stamp_diff = data.header.seq - self.prev_stamp 
	if stamp_diff <= 5:
		return



	# Simple check for PS4 driver to set initial value
	#   for the right and left triggers
	if data.axes[2] == 0 and data.axes[4] == 0:
		return
	msg = Float32MultiArray()
        #msg.dim.size = 4
	# Read joystick message values 
        dxl0_scale = data.axes[2] # right thumb horiz
	right = data.axes[4] - 1  #right trigger
	left = data.axes[3] - 1  #left trigger
	dxl1_scale =  left - right

	dxl2_cut = data.axes[6] # normally 0 or 1/-1 pressed left/right d-pad

	linsrvo_chng = data.axes[5]  #right thumb vertical
	msg.data = [dxl0_scale, dxl1_scale, dxl2_cut, linsrvo_chng]
	self.goal_pub.publish(msg)

        

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
    #GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    #GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    turret = MoveArm()

    while not rospy.is_shutdown():
        turret.rate.sleep
