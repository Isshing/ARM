#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;
StaticJsonDocument<256> jsonInfoHttp;

#include <SCServo.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <nvs_flash.h>

// functions for oled.
#include "oled_ctrl.h"
// functions for RoArm-M2 ctrl.
#include "RoArm-M2_module.h"
// functions for pneumatic modules and lights ctrl. 
#include "switch_module.h"
// define json cmd.
#include "json_cmd.h"
// functions for editing the files in flash.
#include "files_ctrl.h"
// advance functions for RoArm-M2 ctrl.
#include "RoArm-M2_advance.h"
// functions for wifi ctrl.
#include "wifi_ctrl.h"
// functions for esp-now.
#include "esp_now_ctrl.h"
// functions for uart json ctrl.
#include "uart_ctrl.h"
// functions for http & web server.
#include "http_server.h"
#include "pwmServoCtrl.h"


// void setup() {
//   Serial.begin(115200);
//   Wire.begin(S_SDA, S_SCL);
//   while(!Serial) {}

//   delay(1200);

//   initOLED();
//   screenLine_0 = "RoArm-M2";
//   screenLine_1 = "version: 0.84";
//   screenLine_2 = "starting...";
//   screenLine_3 = "";
//   oled_update();

//   // init the littleFS funcs in files_ctrl.h
//   screenLine_2 = screenLine_3;
//   screenLine_3 = "Initialize LittleFS";
//   oled_update();
//   if(InfoPrint == 1){Serial.println("Initialize LittleFS for Flash files ctrl.");}
//   initFS();

//   // init the funcs in switch_module.h
//   screenLine_2 = screenLine_3;
//   screenLine_3 = "Initialize 12V-switch ctrl";
//   oled_update();
//   if(InfoPrint == 1){Serial.println("Initialize the pins used for 12V-switch ctrl.");}
//   switchPinInit();

//   // servos power up
//   screenLine_2 = screenLine_3;
//   screenLine_3 = "Power up the servos";
//   oled_update();
//   if(InfoPrint == 1){Serial.println("Power up the servos.");}
//   delay(500);
  
//   // init servo ctrl functions.
//   screenLine_2 = screenLine_3;
//   screenLine_3 = "ServoCtrl init UART2TTL...";
//   oled_update();
//   if(InfoPrint == 1){Serial.println("ServoCtrl init UART2TTL...");}
//   RoArmM2_servoInit();

//   // check the status of the servos.
//   screenLine_2 = screenLine_3;
//   screenLine_3 = "Bus servos status check...";
//   oled_update();
//   if(InfoPrint == 1){Serial.println("Bus servos status check...");}
//   RoArmM2_initCheck(false);

//   if(InfoPrint == 1 && RoArmM2_initCheckSucceed){
//     Serial.println("All bus servos status checked.");
//   }
//   if(RoArmM2_initCheckSucceed) {
//     screenLine_2 = "Bus servos: succeed";
//   } else {
//     screenLine_2 = "Bus servos: " + 
//     servoFeedback[BASE_SERVO_ID - 11].status +
//     servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status +
//     servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
//     servoFeedback[ELBOW_SERVO_ID - 11].status +
//     servoFeedback[GRIPPER_SERVO_ID - 11].status;
//   }
//   screenLine_3 = ">>> Moving to init pos...";
//   oled_update();
//   RoArmM2_resetPID();
//   RoArmM2_moveInit();

//   screenLine_3 = "Reset joint torque to ST_TORQUE_MAX";
//   oled_update();
//   if(InfoPrint == 1){Serial.println("Reset joint torque to ST_TORQUE_MAX.");}
//   RoArmM2_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX);

//   // screenLine_3 = "WiFi init";
//   // oled_update();
//   // if(InfoPrint == 1){Serial.println("WiFi init.");}
//   // initWifi();

//   // screenLine_3 = "http & web init";
//   // oled_update();
//   // if(InfoPrint == 1){Serial.println("http & web init.");}
//   // initHttpWebServer();

//   screenLine_3 = "RoArm-M2 started";
//   oled_update();
//   if(InfoPrint == 1){Serial.println("RoArm-M2 started.");}

//   if(InfoPrint == 1){Serial.println("Application initialization settings.");}
//   createMission("boot", "these cmds run automatically at boot.");
//   missionPlay("boot", 1);
// }


// void loop() {
//   serialCtrl();
//   // server.handleClient();
//   char receivedChar = Serial.read();
// // 
//   // if (receivedChar=='1')
//   // {
//   //   RoArmM2_Test_drawSqureYZ(-100, 0, 50, 300);
//   // }

//   unsigned long curr_time = millis();
//   if (curr_time - prev_time >= 10){
//     // constantHandle();
    
//     prev_time = curr_time;
//   }
//   // RoArmM2_allPosAbsBesselCtrl(l3+l2B,0,l2A,M_PI,0.36);
//   // delay(1500);
//   // RoArmM2_allPosAbsBesselCtrl(l3+l2B+10,5,l2A,M_PI,0.36);
//   // RoArmM2_Test_drawSqureYZ(-100, 0, 50, 300);

//   RoArmM2_getPosByServoFeedback();//获取参数

//   if (InfoPrint == 2) {
//     RoArmM2_infoFeedback();
//   }
//   if(runNewJsonCmd) {
//     jsonCmdReceiveHandler();
//     jsonCmdReceive.clear();
//     runNewJsonCmd = false;
//   }
// }



char Process_flag=0;

// 定义任务句柄
TaskHandle_t Task1;
TaskHandle_t Task2;

// 定义任务函数
void Task1code( void * pvParameters ){
  // Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());
  for(;;){
    // Serial.println("This is Task1");

    static char time =0;

    unsigned long curr_time = millis();
    if (curr_time - prev_time >= 10){
      constantHandle();
      prev_time = curr_time;
    }
    // RoArmM2_Test_drawSqureYZ(-100, 0, 50, 300);
    // RoArmM2_singleJointAbsCtrl(SHOULDER_JOINT,M_PI/2,0.36,10);
    // RoArmM2_allJointAbsCtrl();

    if (Process_flag)
    {
      jsonCmdReceiveHandler();
      time =1;
      
      Process_flag=0;
    }


RoArmM2_allPosAbsBesselCtrl(58+Camera_Input_X,Camera_Input_Y,60+Camera_Input_Z+30,initT,0.36);
												// RoArmM2_baseCoordinateCtrl(Shelve_Left_2_inputX_Left_Scan,0,Shelve_Left_2_inputZ_Left_Scan,initT);
												// RoArmM2_goalPosMove();

    RoArmM2_getPosByServoFeedback();//获取参数

    // jsonCmdReceiveHandler();

    // jsonCmdReceive.clear();

    vTaskDelay(2 / portTICK_PERIOD_MS); // 延迟2ms
  }
}

void Task2code( void * pvParameters ){
  // Serial.print("Task2 running on core ");
  // Serial.println(xPortGetCoreID());
  for(;;){
    
    serialCtrl();
    // if (InfoPrint == 2) {
    //   RoArmM2_infoFeedback();
    // }
    // Serial.println("This is Task2");
    // vTaskDelay(50 / portTICK_PERIOD_MS); // 延迟2ms
  }
}



void setup() {

    Serial.begin(115200);
  Wire.begin(S_SDA, S_SCL);
  while(!Serial) {}

  delay(1200);

  initOLED();
  screenLine_0 = "RoArm-M2";
  screenLine_1 = "version: 0.84";
  screenLine_2 = "starting...";
  screenLine_3 = "";
  oled_update();

  // init the littleFS funcs in files_ctrl.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize LittleFS";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize LittleFS for Flash files ctrl.");}
  initFS();

  // init the funcs in switch_module.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize 12V-switch ctrl";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize the pins used for 12V-switch ctrl.");}
  switchPinInit();

  // servos power up
  screenLine_2 = screenLine_3;
  screenLine_3 = "Power up the servos";
  oled_update();
  if(InfoPrint == 1){Serial.println("Power up the servos.");}
  delay(500);
  
  // init servo ctrl functions.
  screenLine_2 = screenLine_3;
  screenLine_3 = "ServoCtrl init UART2TTL...";
  oled_update();
  if(InfoPrint == 1){Serial.println("ServoCtrl init UART2TTL...");}
  RoArmM2_servoInit();
  pwmServoInit();

  // check the status of the servos.
  screenLine_2 = screenLine_3;
  screenLine_3 = "Bus servos status check...";
  oled_update();
  if(InfoPrint == 1){Serial.println("Bus servos status check...");}
  RoArmM2_initCheck(false);

  if(InfoPrint == 1 && RoArmM2_initCheckSucceed){
    Serial.println("All bus servos status checked.");
  }
  if(RoArmM2_initCheckSucceed) {
    screenLine_2 = "Bus servos: succeed";
  } else {
    screenLine_2 = "Bus servos: " + 
    servoFeedback[BASE_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
    servoFeedback[ELBOW_SERVO_ID - 11].status +
    servoFeedback[GRIPPER_SERVO_ID - 11].status;
  }
  screenLine_3 = ">>> Moving to init pos...";
  oled_update();
  RoArmM2_resetPID();
  RoArmM2_moveInit();

  screenLine_3 = "Reset joint torque to ST_TORQUE_MAX";
  oled_update();
  if(InfoPrint == 1){Serial.println("Reset joint torque to ST_TORQUE_MAX.");}
  RoArmM2_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX);

  // screenLine_3 = "WiFi init";
  // oled_update();
  // if(InfoPrint == 1){Serial.println("WiFi init.");}
  // initWifi();

  // screenLine_3 = "http & web init";
  // oled_update();
  // if(InfoPrint == 1){Serial.println("http & web init.");}
  // initHttpWebServer();

  screenLine_3 = "RoArm-M2 started";
  oled_update();
  if(InfoPrint == 1){Serial.println("RoArm-M2 started.");}

  if(InfoPrint == 1){Serial.println("Application initialization settings.");}
  createMission("boot", "these cmds run automatically at boot.");
  missionPlay("boot", 1);
  
  // 创建任务
  xTaskCreatePinnedToCore(
    Task1code, /* 任务函数 */
    "Task1",   /* 任务名字 */
    10000,     /* 栈大小 */
    NULL,      /* 传递给任务函数的参数 */
    1,         /* 优先级 */
    &Task1,    /* 任务句柄 */
    0);        /* 核心编号 */

  xTaskCreatePinnedToCore(
    Task2code,
    "Task2",
    10000,
    NULL,
    1,
    &Task2,
    1);
}


void loop() {
  // 在这里不需要做任何事情，因为所有的工作都在任务中完成
}
