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
#include "my_math.h"

char Process_flag=0;

// 定义任务句柄
TaskHandle_t Task1;
TaskHandle_t Task2;

// 定义任务函数
void Task1code( void * pvParameters ){

  for(;;){

    unsigned long curr_time = millis();

    if (curr_time - prev_time >= 10){ //没有收到串口指令时也要保持控制
      constantHandle();
      prev_time = curr_time;
    }

    RoArmM2_getPosByServoFeedback();//获取参数

    if (Process_flag)
    {
      jsonCmdReceiveHandler();
      jsonCmdReceive.clear();  //保证执行完Cmd后不会继续重复执行
      Process_flag=0;
    }

    

    vTaskDelay(2 / portTICK_PERIOD_MS); // 延迟2ms
  }
}

void Task2code( void * pvParameters ){
  for(;;){
    
    serialCtrl();
    
    String X_Show =(String)goalX;
    String Y_Show =(String)goalY;
    String Z_Show =(String)goalZ;

    String B_Show =(String)BASE_JOINT_RAD;
    String S_Show =(String)SHOULDER_JOINT_RAD;
    String E_Show =(String)ELBOW_JOINT_RAD;
    String W_Show =(String)EOAT_JOINT_RAD_BUFFER;
    String G_Show =(String)GRAB_JOINT_PWM;

    screenLine_0="X:"+ X_Show + "Y:" +Y_Show + "Z:" +Z_Show;
    screenLine_1="B:"+ B_Show + "S:" +S_Show + "E:" +E_Show +"W:" + W_Show +"G" + G_Show;

    // if (InfoPrint == 2) {
    //   RoArmM2_infoFeedback();
    // }
    oled_update();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}



void setup() {

  Serial.begin(115200);
  Wire.begin(S_SDA, S_SCL);
  while(!Serial) {}

  delay(1200);

  initOLED();
  initFS();

  switchPinInit();

  delay(500);

  RoArmM2_servoInit();
  pwmServoInit();

  RoArmM2_initCheck(false);

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

  RoArmM2_resetPID();
  RoArmM2_moveInit();

  RoArmM2_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX);  //自动外力适应

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

