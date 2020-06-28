# DAMM-arm

1. Clone this repo inside ~/catkin_ws/src  with `git clone https://github.com/prefpkg21/DAMM-arm.git`
 - do the same for https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
 - cd to catkin_ws and run 
 `rosdep install --from=paths src --ignore-src -y -r`
 
2. Setup PWM for XavierNX
 - visit https://github.com/NVIDIA/jetson-gpio to install python GPIO library
 
 - Visit this link to enable the PWM Pin https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/hw_setup_jetson_io.html
 
 
3. Open the python script inside scripts directory
- Set the correct limits for servos at the top
```python
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
CUT_LEFT = 800 #left cut position
CUT_RIGHT = 3000


DXL0_HOME = 2040
DXL1_HOME = 2040
CUT_HOME = 1530
```

Make sure everything clear of servos!!!
- on one terminal run `roslaunch arm_control_relay teleop.launch`
This will start PS4 driver and the Dynamixel servos and remain troqued in-place
## Check PS4 mappings
run in separate terminal `rostopic echo /bluetooth_teleop/joy`
## Set buttons in code
Down in handle_joystick is where to change buttons  
### Remember that arrays start at 0, not 1
```python
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
```
-open another and run `rosrun arm_control_relay control_arm.py`
This will send the home commands to servos and start moving the servos based on PS4 input


##Troubleshooting
- Unable to open port 
open the launch file in the dynamixel workbench-controllers package and set to either /dev/ttyACM0 or /dev/ttyUSB0 based on device used to connect dynamixels to computer.
-servos stutter
Change the P-value of the dynamixels to around 300 on the Robotis computer program for PID-control.
