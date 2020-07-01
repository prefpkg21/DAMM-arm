/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <DynamixelWorkbench.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <Servo.h>
#include <math.h> 

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

#define BAUDRATE  1000000
#define SRV_COUNT 3
#define DXL_ID_1  1
#define DXL_ID_2  2
#define DXL_ID_3  3
#define DXL1_MAX   4090
#define DXL1_MIN   0
#define DXL2_MAX   3090
#define DXL2_MIN   1030
#define DXL3_MAX   3360
#define DXL3_MIN   0
#define LIN_MAX   180
#define LIN_MIN   0
#define CUT_LEFT  0
#define CUT_RIGHT  3000
#define DXL1_HOME  2040
#define DXL2_HOME  2040
#define CUT_HOME  1530
int move_strength1 = -300;
int move_strength2 = -100;

DynamixelWorkbench dxl_wb;
Servo myservo;

const int led_1 = 13;
const int led_2 = 12;

  uint8_t dxl_id[3] = {DXL_ID_1, DXL_ID_2, DXL_ID_3};

  int32_t present_position[3] = {0, 0, 0};
  int32_t goal_position[3] = {2048, 2048, 1530};
  int32_t home_position[3] = {DXL1_HOME, DXL2_HOME, CUT_HOME};
  int     lin_srvo_goal = 90;
const uint8_t handler_index = 0;
const uint8_t servo_count = SRV_COUNT;
void goal_cb( const std_msgs::Float32MultiArray& goal_msg);
void light_cb( const std_msgs::UInt8MultiArray&);
//ROS Setup
ros::NodeHandle nh;
std_msgs::Float32MultiArray srvo_goal_msg;
std_msgs::Float32MultiArray srvo_pos_msg;
std_msgs::UInt8MultiArray led_msg;

ros::Subscriber<std_msgs::Float32MultiArray> goal_sub("damm_goal",  &goal_cb);
ros::Subscriber<std_msgs::UInt8MultiArray> light_sub("damm_light",  &light_cb);
ros::Publisher  position_pub("damm_position",  &srvo_pos_msg);



bool same_same (int32_t* goal, int32_t* pos, int len_g, int len_p) {
  if (len_g != len_p) return false;

  for (size_t n = 0; n < len_g; n++) if (goal[n]!=pos[n]) return false;

  return true;

}

void setup() 
{
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);

  digitalWrite(led_1, LOW);
  digitalWrite(led_2, LOW);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(position_pub);
  nh.subscribe(goal_sub);
  //Serial.begin(57600);
   //while(!Serial); // Wait for Opening Serial Monitor

  const char *log;
  bool result = false;

  uint16_t model_number = 0;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  
  //Ping all servos to wakeup and assign
  for (int cnt = 0; cnt < SRV_COUNT; cnt++)
  {
    result = dxl_wb.ping(dxl_id[cnt], &model_number, &log);
    
    result = dxl_wb.jointMode(dxl_id[cnt], 0, 0, &log);
   
  } 
  // Sync Handler allows writing to single register for multiple servos
  result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);

  //same for Read
  result = dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Position", &log);
  //Set Home positions
  result = dxl_wb.syncWrite(handler_index, dxl_id, SRV_COUNT, home_position, 1, &log);

  myservo.write(lin_srvo_goal);
}

void loop() 
{  
  const char *log;
  bool result = false;

  //result = dxl_wb.syncWrite(handler_index, &goal_position[0], &log);
  if (!(same_same(goal_position, present_position, SRV_COUNT, SRV_COUNT))) result = dxl_wb.syncWrite(handler_index, dxl_id, SRV_COUNT, goal_position, 1, &log);
    result = dxl_wb.syncRead(handler_index, &log);
    result = dxl_wb.getSyncReadData(handler_index, &present_position[0], &log);

  //write linear Servo
  myservo.write(lin_srvo_goal);
  delay(10);
  nh.spinOnce();
}

void goal_cb( const std_msgs::Float32MultiArray& goal_msg){
  
    float dxl1 = goal_msg.data[0];
    float dxl2 = goal_msg.data[1];
    int dxl3 = goal_msg.data[2];
    float lin_srvo    = goal_msg.data[3];
    // Calculation to set the new positions 
    //Max/min to maintain boundary
    goal_position[0] = fmax(fmin((present_position[0] + (dxl1 * move_strength1)), DXL1_MAX),DXL1_MIN);
    goal_position[1] = fmax(fmin((present_position[1] + (dxl2 * move_strength2)), DXL2_MAX),DXL2_MIN);
    lin_srvo_goal = fmax(fmin(lin_srvo_goal + ceil(lin_srvo), LIN_MIN), LIN_MAX);
    if (dxl3 == 1){
      goal_position[2] = CUT_LEFT;
    }else if (dxl3 == -1){
      goal_position[2] = CUT_RIGHT;
    }else{
      goal_position[2] = CUT_HOME;
    }
}

void light_cb( const std_msgs::UInt8MultiArray& msg){
  int led1 = msg.data[0];
  int led2 = msg.data[1];
  (led1) ? digitalWrite(led_1, HIGH) : digitalWrite(led_1, LOW);
  (led2) ? digitalWrite(led_2, HIGH) : digitalWrite(led_2, LOW); 
}
