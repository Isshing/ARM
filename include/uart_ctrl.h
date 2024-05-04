#ifndef __UART_CTRL_H__
#define __UART_CTRL_H__

extern char CARGO_LEFT_Flag;
extern char SHINK_Flag; 
extern char SHIFT_R2L_Flag; 
extern char SHIFT_L2R_Flag;

void jsonCmdReceiveHandler()
{
	int cmdType = jsonCmdReceive["T"].as<int>();
	switch (cmdType)
	{
	// emergency stop. 紧急停止
	case CMD_EMERGENCY_STOP:
		RoArmM2_emergencyStopFlag = true;
		emergencyStopProcessing();
		break;
	case CMD_RESET_EMERGENCY:
		RoArmM2_emergencyStopFlag = false;
		break;
	// it moves to goal position directly
	// with interpolation.
	case CMD_MOVE_INIT:
		RoArmM2_moveInit();
		break;
	case CMD_SINGLE_JOINT_CTRL:
		RoArmM2_singleJointAbsCtrl(
			jsonCmdReceive["joint"],
			jsonCmdReceive["rad"],
			jsonCmdReceive["spd"],
			jsonCmdReceive["acc"]);
		break;
	case CMD_JOINTS_RAD_CTRL:
		RoArmM2_allJointAbsCtrl(
			jsonCmdReceive["base"],
			jsonCmdReceive["shoulder"],
			jsonCmdReceive["elbow"],
			jsonCmdReceive["hand"],
			jsonCmdReceive["spd"],
			jsonCmdReceive["acc"]);
		break;
	case CMD_SINGLE_AXIS_CTRL:
		RoArmM2_singlePosAbsBesselCtrl(
			jsonCmdReceive["axis"],
			jsonCmdReceive["pos"],
			jsonCmdReceive["spd"]);
		break;
	case CMD_XYZT_GOAL_CTRL:
		RoArmM2_allPosAbsBesselCtrl(
			jsonCmdReceive["x"],
			jsonCmdReceive["y"],
			jsonCmdReceive["z"],
			jsonCmdReceive["t"],
			jsonCmdReceive["spd"]);
		break;

	case CMD_XYZT_ACQU: // 摄像头坐标

		Serial.print("CMD\n"); // 发送接收完成应答
		Serial.print("CMD\n"); // 发送接收完成应答
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
		}
		else if (Shelve_Layer == 3)
		{
			Grab_Cargo_3();
		}

		receive_cmd_flag = 1;
		CARGO_LEFT_Flag = 0;
		break;
	case CMD_OCR_FINISH:	  // OCR识别完成
		ARM_MODE = SHIFT_R2L; // 转回去
		SHIFT_R2L_Flag =0;
		receive_cmd_flag = 1;
		break;

	case CMD_START: // 车辆启动，进入OCR状态
		ARM_MODE = SHIFT_L2R;
		SHIFT_L2R_Flag =0;
		receive_cmd_flag = 1;
		break;

	case CMD_OVER: // 抓取完成，进入收缩状态
		ARM_MODE = SHINK;
		receive_cmd_flag = 1;
		SHINK_Flag =0;
		break;

	case CMD_XYZT_DIRECT_CTRL:
		RoArmM2_baseCoordinateCtrl(
			jsonCmdReceive["x"],
			jsonCmdReceive["y"],
			jsonCmdReceive["z"],
			jsonCmdReceive["t"]);
		RoArmM2_goalPosMove();
		break;
	case CMD_SERVO_RAD_FEEDBACK:
		RoArmM2_getPosByServoFeedback();
		RoArmM2_infoFeedback();
		break;

	case CMD_EOAT_HAND_CTRL:
		RoArmM2_handJointCtrlRad(1,
								 jsonCmdReceive["cmd"],
								 jsonCmdReceive["spd"],
								 jsonCmdReceive["acc"]);
		break;
	case CMD_EOAT_GRAB_TORQUE:
		RoArmM2_handTorqueCtrl(
			jsonCmdReceive["tor"]);
		break;

	case CMD_DELAY_MILLIS:
		RoArmM2_delayMillis(
			jsonCmdReceive["cmd"]);
		break;
	case CMD_SWITCH_OFF:
		switchEmergencyStop();
		break;
	case CMD_SINGLE_JOINT_ANGLE:
		RoArmM2_singleJointAngleCtrl(
			jsonCmdReceive["joint"],
			jsonCmdReceive["angle"],
			jsonCmdReceive["spd"],
			jsonCmdReceive["acc"]);
		break;
	case CMD_JOINTS_ANGLE_CTRL:
		RoArmM2_allJointsAngleCtrl(
			jsonCmdReceive["b"],
			jsonCmdReceive["s"],
			jsonCmdReceive["e"],
			jsonCmdReceive["h"],
			jsonCmdReceive["spd"],
			jsonCmdReceive["acc"]);
		break;
		// constant ctrl
		// m: 0 - angle
		//    1 - xyzt
		// cmd: 0 - stop
		// 		  1 - increase
		// 		  2 - decrease
		// {"T":123,"m":0,"axis":0,"cmd":0,"spd":0}
	case CMD_CONSTANT_CTRL:
		constantCtrl(
			jsonCmdReceive["m"],
			jsonCmdReceive["axis"],
			jsonCmdReceive["cmd"],
			jsonCmdReceive["spd"]);
		break;
	// mission & steps edit & file edit.
	case CMD_SCAN_FILES:
		scanFlashContents();
		break;
	case CMD_CREATE_FILE:
		createFile(
			jsonCmdReceive["name"],
			jsonCmdReceive["content"]);
		break;
	case CMD_READ_FILE:
		readFile(
			jsonCmdReceive["name"]);
		break;
	case CMD_DELETE_FILE:
		deleteFile(
			jsonCmdReceive["name"]);
		break;
	case CMD_APPEND_LINE:
		appendLine(
			jsonCmdReceive["name"],
			jsonCmdReceive["content"]);
		break;
	case CMD_INSERT_LINE:
		insertLine(
			jsonCmdReceive["name"],
			jsonCmdReceive["lineNum"],
			jsonCmdReceive["content"]);
		break;
	case CMD_REPLACE_LINE:
		replaceLine(
			jsonCmdReceive["name"],
			jsonCmdReceive["lineNum"],
			jsonCmdReceive["content"]);
		break;
	case CMD_READ_LINE:
		readSingleLine(
			jsonCmdReceive["name"],
			jsonCmdReceive["lineNum"]);
		break;
	case CMD_DELETE_LINE:
		deleteSingleLine(
			jsonCmdReceive["name"],
			jsonCmdReceive["lineNum"]);
		break;

	case CMD_TORQUE_CTRL:
		servoTorqueCtrl(254,
						jsonCmdReceive["cmd"]);
		break;

	case CMD_CREATE_MISSION:
		createMission(
			jsonCmdReceive["name"],
			jsonCmdReceive["intro"]);
		break;
	case CMD_MISSION_CONTENT:
		missionContent(
			jsonCmdReceive["name"]);
		break;
	case CMD_APPEND_STEP_JSON:
		appendStepJson(
			jsonCmdReceive["name"],
			jsonCmdReceive["step"]);
		break;
	case CMD_APPEND_STEP_FB:
		appendStepFB(
			jsonCmdReceive["name"],
			jsonCmdReceive["spd"]);
		break;
	case CMD_APPEND_DELAY:
		appendDelayCmd(
			jsonCmdReceive["name"],
			jsonCmdReceive["delay"]);
		break;
	case CMD_INSERT_STEP_JSON:
		insertStepJson(
			jsonCmdReceive["name"],
			jsonCmdReceive["stepNum"],
			jsonCmdReceive["step"]);
		break;
	case CMD_INSERT_STEP_FB:
		insertStepFB(
			jsonCmdReceive["name"],
			jsonCmdReceive["stepNum"],
			jsonCmdReceive["spd"]);
		break;
	case CMD_INSERT_DELAY:
		insertDelayCmd(
			jsonCmdReceive["name"],
			jsonCmdReceive["stepNum"],
			jsonCmdReceive["spd"]);
		break;
	case CMD_REPLACE_STEP_JSON:
		replaceStepJson(
			jsonCmdReceive["name"],
			jsonCmdReceive["stepNum"],
			jsonCmdReceive["step"]);
		break;
	case CMD_REPLACE_STEP_FB:
		replaceStepFB(
			jsonCmdReceive["name"],
			jsonCmdReceive["stepNum"],
			jsonCmdReceive["spd"]);
		break;
	case CMD_REPLACE_DELAY:
		replaceDelayCmd(
			jsonCmdReceive["name"],
			jsonCmdReceive["stepNum"],
			jsonCmdReceive["delay"]);
		break;
	case CMD_DELETE_STEP:
		deleteStep(
			jsonCmdReceive["name"],
			jsonCmdReceive["stepNum"]);
		break;

	case CMD_MOVE_TO_STEP:
		moveToStep(
			jsonCmdReceive["name"],
			jsonCmdReceive["stepNum"]);
		break;
	case CMD_MISSION_PLAY:
		missionPlay(
			jsonCmdReceive["name"],
			jsonCmdReceive["times"]);
		break;
	// esp-32 dev ctrl.
	case CMD_REBOOT:
		esp_restart();
		break;
	case CMD_FREE_FLASH_SPACE:
		freeFlashSpace();
		break;
	case CMD_BOOT_MISSION_INFO:
		missionContent("boot");
		break;
	case CMD_RESET_BOOT_MISSION:
		deleteFile("boot.mission");
		break;
		createFile("boot", "these cmds run automatically at boot.");
	case CMD_NVS_CLEAR:
		nvs_flash_erase();
		delay(1000);
		nvs_flash_init();
		break;
	case CMD_INFO_PRINT:
		configInfoPrint(
			jsonCmdReceive["cmd"]);
		break;
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