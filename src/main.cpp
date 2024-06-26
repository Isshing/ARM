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

char Process_flag = 0;
char receive_cmd_flag = 0;

// 定义任务句柄
TaskHandle_t Task1;
TaskHandle_t Task2;

int timee = 0;

// 定义任务函数
void Task1code(void *pvParameters)
{
  static s16 SHIFT_L2R_time = 0;
  static char SHIFT_L2R_Flag = 0;

  for (;;)
  {

    unsigned long curr_time = millis();

    if (curr_time - prev_time >= 10)
    { // 没有收到串口指令时也要保持控制
      constantHandle();
      prev_time = curr_time;
    }

    RoArmM2_getPosByServoFeedback(); // 获取参数

    switch (ARM_MODE)
    {
    case OCR_SCAN: // OCR识别状态
      MY_RoArmM2_allPosAbsBesselCtrl(OCR_inputX, 0, OCR_inputZ, 0.25);
      break;
    case CARGO_LEFT: // 左货架抓取状态

      switch (Shelve_Layer) // 回到检视状态
      {
      case 1:
        MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Left_1_inputX_Left_Scan, 0, Shelve_Left_1_inputZ_Left_Scan, 0.25);
        break;
      case 2:
        MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Left_2_inputX_Left_Scan, 0, Shelve_Left_2_inputZ_Left_Scan, 0.25);
        break;
      case 3:
        MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Left_3_inputX_Left_Scan, 0, Shelve_Left_3_inputZ_Left_Scan, 0.25);
        break;
      case 4:
        MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Left_4_inputX_Left_Scan, 0, Shelve_Left_4_inputZ_Left_Scan, 0.25);
      default:
        break;
      }
      if (Process_flag)
      {
        jsonCmdReceiveHandler();
        jsonCmdReceive.clear(); // 保证执行完Cmd后不会继续重复执行
        Process_flag = 0;
        receive_cmd_flag = 0; // 重新准备接收命令

        Serial.print("GF\n"); // 发送抓取完成指令
        Serial.print("GF\n");
        Serial.print("GF\n");
      }
      break;
    case CARGO_RIGHT:       // 右货架抓取状态
      switch (Shelve_Layer) // 回到检视状态
      {
      case 1:
        MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Right_1_inputX_Left_Scan, 0, Shelve_Right_1_inputZ_Left_Scan, 0.25);
        break;
      case 2:
        MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Right_2_inputX_Left_Scan, 0, Shelve_Right_2_inputZ_Left_Scan, 0.25);
        break;
      case 3:
        MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Right_3_inputX_Left_Scan, 0, Shelve_Right_3_inputZ_Left_Scan, 0.25);
        break;
      case 4:
        MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Right_4_inputX_Left_Scan, 0, Shelve_Right_4_inputZ_Left_Scan, 0.25);
      default:
        break;
      }
      if (Process_flag)
      {
        jsonCmdReceiveHandler();
        jsonCmdReceive.clear(); // 保证执行完Cmd后不会继续重复执行
        Process_flag = 0;
        receive_cmd_flag = 0; // 重新准备接收命令

        Serial.print("GF\n"); // 发送抓取完成指令
        Serial.print("GF\n"); // 发送抓取完成指令
        Serial.print("GF\n"); // 发送抓取完成指令
      }
      break;
    case SHIFT_L2R: // 左向右自旋

      if (SHIFT_L2R_Flag == 0)
      {
        SHIFT_L2R_time++;

        if (SHIFT_L2R_time < 800)
        {
          RoArmM2_allJointAbsCtrl(0, 0, M_PI / 10, M_PI, 0.36, 1);
        }
        else if (SHIFT_L2R_time >= 800 && SHIFT_L2R_time < 1600)
        {
          RoArmM2_allJointAbsCtrl(M_PI, 0, M_PI / 10, M_PI, 0.25, 1);
        }
        else
        {
          // RoArmM2_baseJointCtrlRad_Right(1,0,0.35,1);
          // MY_RoArmM2_allPosAbsBesselCtrl_Right(50, 0, Shelve_Right_2_inputZ_Left_Scan, 0.25);
          SHIFT_L2R_time = 0;
          SHIFT_L2R_Flag = 1;
        }
      }
      break;
    case SHIFT_R2L: // 右向左自旋
      break;
    case SHINK_LEFT: // 左边收缩状态
      break;
    case SHINK_RIGHT: // 右边收缩状态
      break;

    default:
      break;
    }

    vTaskDelay(2 / portTICK_PERIOD_MS); // 延迟2ms
  }
}

void Task2code(void *pvParameters)
{
  for (;;)
  {

    if (receive_cmd_flag == 0) // 等待接收命令
    {
      serialCtrl();
    }

    String X_Show = (String)goalX;
    String Y_Show = (String)goalY;
    String Z_Show = (String)goalZ;

    String B_Show = (String)BASE_JOINT_RAD;
    String S_Show = (String)SHOULDER_JOINT_RAD;
    String E_Show = (String)ELBOW_JOINT_RAD;
    String W_Show = (String)EOAT_JOINT_RAD_BUFFER;
    String G_Show = (String)GRAB_JOINT_PWM;

    String X_Show_FeBk = (String)lastX;
    String Y_Show_FeBk = (String)lastY;
    String Z_Show_FeBk = (String)lastZ;

    screenLine_0 = "X:" + X_Show + "Y:" + Y_Show + "Z:" + Z_Show;
    screenLine_1 = "B:" + B_Show + "S:" + S_Show + "E:" + E_Show + "W:" + W_Show + "G:" + G_Show;
    // screenLine_1="XF:"+ X_Show_FeBk + "YF:" +Y_Show_FeBk + "ZF:" +Z_Show_FeBk ;

    // if (InfoPrint == 2) {
    //   RoArmM2_infoFeedback();
    // }
    oled_update();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup()
{

  Serial.begin(115200);
  Wire.begin(S_SDA, S_SCL);
  while (!Serial)
  {
  }

  delay(1200);

  initOLED();
  initFS();

  switchPinInit();

  delay(500);

  RoArmM2_servoInit();
  pwmServoInit();

  RoArmM2_initCheck(false);

  if (RoArmM2_initCheckSucceed)
  {
    screenLine_2 = "Bus servos: succeed";
  }
  else
  {
    screenLine_2 = "Bus servos: " +
                   servoFeedback[BASE_SERVO_ID - 11].status +
                   servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status +
                   servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
                   servoFeedback[ELBOW_SERVO_ID - 11].status +
                   servoFeedback[GRIPPER_SERVO_ID - 11].status;
  }

  RoArmM2_resetPID();
  RoArmM2_moveInit();

  RoArmM2_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX); // 自动外力适应

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

void loop()
{
  // 在这里不需要做任何事情，因为所有的工作都在任务中完成
}
