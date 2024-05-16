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
char test_flag = 0;
u32 test_time = 0;

char Shift_L2R_Record = 0;
char Shift_R2L_Record = 0;

// 定义任务句柄
TaskHandle_t Task1;
TaskHandle_t Task2;

int timee = 0;
char CARGO_LEFT_Flag = 0;
char SHINK_Flag = 0;
char SHIFT_R2L_Flag = 0;
char SHIFT_L2R_Flag = 0;
char Grab_Record = 0; // 补偿抓取后抓手不水平Flag

int Shelve_Layer = 1; // 当前层数

// 定义任务函数
void Task1code(void *pvParameters)
{
  static u32 SHIFT_L2R_time = 0;
  static u32 SHIFT_R2L_time = 0;
  static u32 SHINK_time = 0;
  static u32 GF_time = 0;

  for (;;)
  {
    unsigned long curr_time = millis();

    if (curr_time - prev_time >= 10)
    { // 没有收到串口指令时也要保持控制
      constantHandle();
      prev_time = curr_time;
    }
    test_time++;

    RoArmM2_getPosByServoFeedback(); // 获取参数

    switch (ARM_MODE)
    {
    case CARGO_LEFT: // 左货架抓取状态

      if (CARGO_LEFT_Flag == 0)
      {
        switch (Shelve_Layer) // 回到检视状态
        {
        case 1:
          // MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Left_1_inputX_Left_Scan, 3, Shelve_Left_1_inputZ_Left_Scan, 0.25);

          // delay(1000);
          RoArmM2_allJointAbsCtrl(0.03, 0.82, 2.77, 1.30, 0, 20);
          // RoArmM2_singleJointAbsCtrl(1,0.03,0,0);
          // RoArmM2_singleJointAbsCtrl(2,0.82,0,0);
          // RoArmM2_singleJointAbsCtrl(3,2.77,0,0);
          // RoArmM2_singleJointAbsCtrl(EOAT_JOINT,1.30,0,0);

          break;
        case 2:

          // MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Left_2_inputX_Left_Scan, 0, Shelve_Left_2_inputZ_Left_Scan, 0.25);
          // MY_RoArmM2_allPosAbsBesselCtrl(140, 0, 100, 0.25);
          // delay(1000);
          RoArmM2_allJointAbsCtrl(0, -0.26, 2.92, 2.25, 0, 20); // Rad显示参数和贝塞尔曲线的结果是一致的，坐标不一致

          // MY_RoArmM2_allPosAbsBesselCtrl(120, 0, 120, 0.25);
          break;

        case 3:
          // MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Left_3_inputX_Left_Scan, 0, Shelve_Left_3_inputZ_Left_Scan, 0.25);
          RoArmM2_allJointAbsCtrl(0, -0.26, 2.92, 2.25, 0, 20); // Rad显示参数和贝塞尔曲线的结果是一致的，坐标不一致
          break;
        case 4:
          // MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Left_4_inputX_Left_Scan, 0, Shelve_Left_4_inputZ_Left_Scan, 0.25);

          // delay(1000);
          RoArmM2_allJointAbsCtrl(0, -0.38, 1.45, 3.85-0.1, 0, 20); // Rad显示参数和贝塞尔曲线的结果是一致的，坐标不一致
        default:
          break;
        }
        CARGO_LEFT_Flag = 1;
      }

      if (Grab_Record == 1)
      {
        GF_time++;
        switch (Shelve_Layer) // 回到检视状态
        {
        case 1:
          // RoArmM2_allJointAbsCtrl(0, 0.46, 3.08, 1.44, 0, 10); // Rad显示参数和贝塞尔曲线的结果是一致的，坐标不一致
          // RoArmM2_allJointAbsCtrl(0.05, 0.76, 2.99, 1.24, 0, 10);
          // MY_RoArmM2_allPosAbsBesselCtrl(Shelve_Left_1_inputX_Left_Scan , 3, Shelve_Left_1_inputZ_Left_Scan, 0.25);
          RoArmM2_allJointAbsCtrl(0.03, 0.82, 2.77, 1.30, 0, 20);
          if (GF_time >= 200)
          {
            Grab_Record = 0;
            GF_time = 0;
            Serial.print("GF\n"); // 发送抓取完成指令
            Serial.print("GF\n"); // 发送抓取完成指令
          }

          break;
        case 2:
          RoArmM2_allJointAbsCtrl(0, -0.26, 2.92, 2.25, 400, 20); // Rad显示参数和贝塞尔曲线的结果是一致的，坐标不一致
          if (GF_time >= 200)
          {
            Grab_Record = 0;
            GF_time = 0;
            Serial.print("GF\n"); // 发送抓取完成指令
            Serial.print("GF\n"); // 发送抓取完成指令
          }

          break;

        case 3:
          RoArmM2_allJointAbsCtrl(0, -0.26, 2.92, 2.25, 0, 20); // Rad显示参数和贝塞尔曲线的结果是一致的，坐标不一致
          if (GF_time >= 200)
          {
            Grab_Record = 0;
            GF_time = 0;
            Serial.print("GF\n"); // 发送抓取完成指令
            Serial.print("GF\n"); // 发送抓取完成指令
          }
          break;
        case 4:
          RoArmM2_allJointAbsCtrl(0, -0.38, 1.45, 3.85-0.1, 0, 20); // Rad显示参数和贝塞尔曲线的结果是一致的，坐标不一致
          if (GF_time >= 200)
          {
            Grab_Record = 0;
            GF_time = 0;
            Serial.print("GF\n"); // 发送抓取完成指令
            Serial.print("GF\n"); // 发送抓取完成指令
          }
        default:
          break;
        }
      }

      if (Process_flag)
      {
        jsonCmdReceiveHandler();
        jsonCmdReceive.clear(); // 保证执行完Cmd后不会继续重复执行

        Process_flag = 0;
        receive_cmd_flag = 0; // 重新准备接收命令
      }
      // if (test_time >= 1000)
      // {
      //   if (test_flag == 0)
      //   {
      //     Camera_XYZ(0,0,0,140);
      //     Grab_Cargo_2();
      //     CARGO_LEFT_Flag = 0;
      //     Grab_Record = 1; // 调整Wrist水平
      //     test_flag = 1;
      //   }
      // }
      // if (test_time >= 1000)
      // {

      //   Camera_XYZ(0, 0, 0, 140);
      //   Grab_Cargo_1();
      //   CARGO_LEFT_Flag = 0;
      //   Grab_Record = 1; // 调整Wrist水平
      //   test_time=0;
      // }

      break;
    case SHIFT_L2R: // 左向右自旋
      if (Shift_L2R_Record == 0)
      {
        if (SHIFT_L2R_Flag == 0)
        {
          SHIFT_L2R_time++;
          if (SHIFT_L2R_time < 200)
          {
            RoArmM2_allJointAbsCtrl(0, 0, M_PI / 10, M_PI, 0, 20);//抬高
          }
          else if (SHIFT_L2R_time >= 200 && SHIFT_L2R_time < 400)
          {
            RoArmM2_allJointAbsCtrl(-M_PI + 0.01, 0, M_PI / 10, M_PI, 0, 20); //转向另一侧
          }
          else if (SHIFT_L2R_time >= 400 && SHIFT_L2R_time < 600)
          {
            RoArmM2_allJointAbsCtrl(-M_PI + 0.01, 0, M_PI / 3 + 0.1, 1.4 * M_PI, 0, 20); //下降
          }
          else if (SHIFT_L2R_time >= 600 && SHIFT_L2R_time < 800)
          {
            GRAB_ServoCtrl(30);
            RoArmM2_allJointAbsCtrl(-M_PI + 0.01, M_PI / 10, M_PI / 3 + 0.1, 1.4 * M_PI, 0, 20);
          }
          else
          {
            SHIFT_L2R_time = 0;
            SHIFT_L2R_Flag = 1;
            Serial.print("O\n"); // 发送旋转完成信号
            Serial.print("O\n"); // 发送旋转完成信号

          }
        }

        if (Process_flag)
        {
          jsonCmdReceiveHandler();
          jsonCmdReceive.clear(); // 保证执行完Cmd后不会继续重复执行
          Process_flag = 0;
          receive_cmd_flag = 0; // 处理完成后，重新准备接收命令
        }
      }

      break;
    case SHIFT_R2L: // 右向左自旋

      if (Shift_R2L_Record == 0)
      {
        if (SHIFT_R2L_Flag == 0)
        {
          SHIFT_R2L_time++;

          if (SHIFT_R2L_time < 200)
          {
            GRAB_ServoCtrl(130);
            RoArmM2_allJointAbsCtrl(-M_PI + 0.01, 0, M_PI / 10, M_PI, 0, 10); // 抬高
          }
          else if (SHIFT_R2L_time >= 200 && SHIFT_R2L_time < 400)
          {
            RoArmM2_allJointAbsCtrl(0, 0, M_PI / 10, M_PI, 0, 10); // 基座回去
          }
          else if (SHIFT_R2L_time >= 400 && SHIFT_R2L_time < 700)
          {
            // RoArmM2_allJointAbsCtrl(0, 0.46, 3.08, 1.44, 0, 10); // Rad显示参数和贝塞尔曲线的结果是一致的，坐标不一致
            RoArmM2_allJointAbsCtrl(0, -0.26, 2.92, 2.25, 500, 20); // Rad显示参数和贝塞尔曲线的结果是一致的，坐标不一致
          }
          else if (SHIFT_R2L_time >=700 && SHIFT_R2L_time <1000)
          {
            GRAB_ServoCtrl(30);
            RoArmM2_allJointAbsCtrl(0.03, 0.82, 2.77, 1.30, 500, 20);
          }
          else
          {
            SHIFT_R2L_Flag = 1;
            CARGO_LEFT_Flag = 0;
            Shelve_Layer = 1;
            ARM_MODE = CARGO_LEFT;
            Serial.print("SF\n"); // 发送旋转完成信号
            Serial.print("SF\n"); // 发送旋转完成信号
            SHIFT_R2L_time = 0;
            Shift_R2L_Record =1;
          }
        }
      }

      break;
    case SHINK: // 收缩状态

      if (SHINK_Flag == 0)
      {
        SHINK_time++;
        if (SHIFT_R2L_time <= 400)
        {
          RoArmM2_allJointAbsCtrl(0, -M_PI / 15, M_PI * 0.95, M_PI * 0.4, 0, 10);
        }
        else
        {
          SHINK_Flag = 1;
          SHINK_time = 0;
        }
      }

      if (Process_flag)
      {
        jsonCmdReceiveHandler();
        jsonCmdReceive.clear(); // 保证执行完Cmd后不会继续重复执行
        Process_flag = 0;
        receive_cmd_flag = 0; // 处理完成后，重新准备接收命令
      }

      break;

    default:
      break;
    }

    vTaskDelay(2 / portTICK_PERIOD_MS); // 延迟2ms
  }
}

void Task2code(void *pvParameters)
{

  static s16 time_serial = 0;
  static s16 time_show = 0;
  for (;;)
  {
    time_serial++;
    time_show++;

    if (time_serial == 2) // 10ms
    {
      if (receive_cmd_flag == 0) // 等待接收命令
      {
        serialCtrl();
      }
      time_serial = 0;
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

    // screenLine_0 = "X:" + X_Show + "Y:" + Y_Show + "Z:" + Z_Show;
    screenLine_0 = "B:" + B_Show + "S:" + S_Show + "E:" + E_Show + "W:" + W_Show + "G:" + G_Show;
    screenLine_1 = "ST:" + (String)ARM_MODE + "\t" + "L:" + (String)Shelve_Layer;
    // screenLine_1="XF:"+ X_Show_FeBk + "YF:" +Y_Show_FeBk + "ZF:" +Z_Show_FeBk ;

    // if (InfoPrint == 2) {
    //   RoArmM2_infoFeedback();
    // }
    if (time_show == 10) // 50ms
    {
      oled_update();
      time_show = 0;
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void setup()
{

  Serial.begin(115200);
  Wire.begin(S_SDA, S_SCL);
  while (!Serial)
  {
  }

  delay(500);

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
                   //  servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
                   servoFeedback[ELBOW_SERVO_ID - 11].status +
                   servoFeedback[GRIPPER_SERVO_ID - 11].status;
  }

  RoArmM2_resetPID();
  // RoArmM2_moveInit();

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
