#ifndef __UART_CTRL_H__
#define __UART_CTRL_H__

extern char CARGO_LEFT_Flag;
extern char SHINK_Flag;
extern char SHIFT_R2L_Flag;
extern char SHIFT_L2R_Flag;
extern char Grab_Record;
extern int Shelve_Layer; //当前层数
extern char Shift_L2R_Record;

void jsonCmdReceiveHandler()
{
	int cmdType = jsonCmdReceive["T"].as<int>();
	switch (cmdType)
	{

	case CMD_XYZT_ACQU: // 摄像头坐标
		Serial.print("CMD\n"); // 发送接收完成应答
		Camera_XYZ(
			jsonCmdReceive["x"],
			jsonCmdReceive["y"],
			jsonCmdReceive["z"],
			jsonCmdReceive["g"]);

		Shelve_Layer = jsonCmdReceive["L"]; // 当前层

		if (Shelve_Layer == 1)
		{
			Grab_Cargo_1();
		}
		else if (Shelve_Layer == 2)
		{
			Grab_Cargo_2();
			// calibration();
		}
		else if (Shelve_Layer == 3)
		{
			Grab_Cargo_3();
		}
		else if (Shelve_Layer == 4)
		{
			Grab_Cargo_4();
		}
		CARGO_LEFT_Flag = 0;
		receive_cmd_flag = 1;
		Grab_Record =1; //调整Wrist水平
	
		break;
	case CMD_OCR_FINISH:	   // OCR识别完成
		Serial.print("CMD\n"); // 发送接收完成应答
		Shift_L2R_Record = 1;
		ARM_MODE = SHIFT_R2L;  // 转回去
		SHIFT_R2L_Flag = 0;
		receive_cmd_flag = 1;
		break;

	case CMD_START:			   // 车辆启动，进入OCR状态
		Serial.print("CMD\n"); // 发送接收完成应答
		ARM_MODE = SHIFT_L2R;
		SHIFT_L2R_Flag = 0;
		Shift_L2R_Record = 0;
		receive_cmd_flag = 1;
		break;

	case CMD_OVER:			   // 抓取完成，进入收缩状态
		Serial.print("CMD\n"); // 发送接收完成应答
		ARM_MODE = SHINK;
		receive_cmd_flag = 1;
		SHINK_Flag = 0;
		break;

	case CMD_UP_DOWN:          //调整高度时提前准备好检视状态
		Serial.print("CMD\n"); // 发送接收完成应答
		Shelve_Layer = jsonCmdReceive["L"];
		if (Shelve_Layer==1)
		{
			Serial.print("SF\n");
		}
		ARM_MODE=CARGO_LEFT;
		CARGO_LEFT_Flag =0; 
	}
}

extern char Process_flag;

void serialCtrl()
{
	static String receivedData;

	while (Serial.available() > 0)
	{
		char receivedChar = Serial.read();
		receivedData += receivedChar;

		// Detect the end of the JSON string based on a specific termination character
		if (receivedChar == '\n')
		{
			// Now we have received the complete JSON string
			DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
			if (err == DeserializationError::Ok)
			{
				// if (InfoPrint == 1)
				// {
				// 	Serial.println(receivedData);
				// }

				Process_flag = 1;
				// Serial.print("CMD\n"); // 发送接收完成应答
			}
			else
			{
				// Handle JSON parsing error here
			}
			// Reset the receivedData for the next JSON string
			receivedData = "";
		}
	}
}

void serialCtrl_coordinate(void)
{
	static String receivedData;
}

#endif