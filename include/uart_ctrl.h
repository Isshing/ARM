void jsonCmdReceiveHandler(){
	int cmdType = jsonCmdReceive["T"].as<int>();
	switch(cmdType){
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
	case CMD_MOVE_INIT:		RoArmM2_moveInit();break;
	case CMD_SINGLE_JOINT_CTRL: 
												RoArmM2_singleJointAbsCtrl(
												jsonCmdReceive["joint"],
												jsonCmdReceive["rad"],
												jsonCmdReceive["spd"],
												jsonCmdReceive["acc"]
												);break;
	case CMD_JOINTS_RAD_CTRL: 
												RoArmM2_allJointAbsCtrl(
												jsonCmdReceive["base"],
												jsonCmdReceive["shoulder"],
												jsonCmdReceive["elbow"],
												jsonCmdReceive["hand"],
												jsonCmdReceive["spd"],
												jsonCmdReceive["acc"]
												);break;
	case CMD_SINGLE_AXIS_CTRL: 
												RoArmM2_singlePosAbsBesselCtrl(
												jsonCmdReceive["axis"],
												jsonCmdReceive["pos"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_XYZT_GOAL_CTRL: 
												RoArmM2_allPosAbsBesselCtrl(
												jsonCmdReceive["x"],
											  jsonCmdReceive["y"],
											  jsonCmdReceive["z"],
											  jsonCmdReceive["t"],
											  jsonCmdReceive["spd"]
											  );break;
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
												jsonCmdReceive["acc"]
												);break;
	case CMD_EOAT_GRAB_TORQUE:
												RoArmM2_handTorqueCtrl(
												jsonCmdReceive["tor"]
												);break;

	case CMD_SET_JOINT_PID:
												RoArmM2_setJointPID(
												jsonCmdReceive["joint"],
												jsonCmdReceive["p"],
												jsonCmdReceive["i"],
												jsonCmdReceive["d"]
												);break;
	case CMD_RESET_PID:		RoArmM2_resetPID();break;

	// set a new x-axis.
	case CMD_SET_NEW_X: 	setNewAxisX(
												jsonCmdReceive["xAxisAngle"]
												);break;
	case CMD_DELAY_MILLIS:
												RoArmM2_delayMillis(
												jsonCmdReceive["cmd"]
												);break;
	case CMD_DYNAMIC_ADAPTATION: 
												RoArmM2_dynamicAdaptation(
												jsonCmdReceive["mode"],
												jsonCmdReceive["b"],
												jsonCmdReceive["s"],
												jsonCmdReceive["e"],
												jsonCmdReceive["h"]
												);break;
	case CMD_SWITCH_CTRL: switchCtrl(
												jsonCmdReceive["pwm_a"],
												jsonCmdReceive["pwm_b"]
												);break;

	case CMD_SWITCH_OFF:  switchEmergencyStop();break;
	case CMD_SINGLE_JOINT_ANGLE:
												RoArmM2_singleJointAngleCtrl(
												jsonCmdReceive["joint"],
												jsonCmdReceive["angle"],
												jsonCmdReceive["spd"],
												jsonCmdReceive["acc"]
												);break;
	case CMD_JOINTS_ANGLE_CTRL:
												RoArmM2_allJointsAngleCtrl(
												jsonCmdReceive["b"],
												jsonCmdReceive["s"],
												jsonCmdReceive["e"],
												jsonCmdReceive["h"],
												jsonCmdReceive["spd"],
												jsonCmdReceive["acc"]
												);break;
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
												jsonCmdReceive["spd"]
												);break;




	// mission & steps edit & file edit.
	case CMD_SCAN_FILES:  scanFlashContents();
												break;
	case CMD_CREATE_FILE: createFile(
												jsonCmdReceive["name"],
												jsonCmdReceive["content"]
												);break;
	case CMD_READ_FILE:		readFile(
												jsonCmdReceive["name"]
												);break;
	case CMD_DELETE_FILE: deleteFile(
												jsonCmdReceive["name"]
												);break;
	case CMD_APPEND_LINE:	appendLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["content"]
												);break;
	case CMD_INSERT_LINE: insertLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["lineNum"],
												jsonCmdReceive["content"]
												);break;
	case CMD_REPLACE_LINE:
												replaceLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["lineNum"],
												jsonCmdReceive["content"]
												);break;
	case CMD_READ_LINE:   readSingleLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["lineNum"]
												);break;
	case CMD_DELETE_LINE: deleteSingleLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["lineNum"]
												);break;


	case CMD_TORQUE_CTRL: servoTorqueCtrl(254,
												jsonCmdReceive["cmd"]);
												break;


	case CMD_CREATE_MISSION:
												createMission(
												jsonCmdReceive["name"],
												jsonCmdReceive["intro"]
												);break;
	case CMD_MISSION_CONTENT:
												missionContent(
												jsonCmdReceive["name"]
												);break;
	case CMD_APPEND_STEP_JSON: 
												appendStepJson(
												jsonCmdReceive["name"],
												jsonCmdReceive["step"]
												);break;
	case CMD_APPEND_STEP_FB:
												appendStepFB(
												jsonCmdReceive["name"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_APPEND_DELAY:
												appendDelayCmd(
												jsonCmdReceive["name"],
												jsonCmdReceive["delay"]
												);break;
	case CMD_INSERT_STEP_JSON:
												insertStepJson(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["step"]
												);break;
	case CMD_INSERT_STEP_FB:
												insertStepFB(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_INSERT_DELAY:
												insertDelayCmd(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_REPLACE_STEP_JSON:
												replaceStepJson(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["step"]
												);break;
	case CMD_REPLACE_STEP_FB:
												replaceStepFB(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_REPLACE_DELAY:
												replaceDelayCmd(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["delay"]
												);break;
	case CMD_DELETE_STEP: deleteStep(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"]
												);break;

	case CMD_MOVE_TO_STEP:
												moveToStep(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"]
												);break;
	case CMD_MISSION_PLAY:
												missionPlay(
												jsonCmdReceive["name"],
												jsonCmdReceive["times"]
												);break;

	// servo settings.
	case CMD_SET_SERVO_ID:
												changeID(
												jsonCmdReceive["raw"],
												jsonCmdReceive["new"]
												);break;
	case CMD_SET_MIDDLE:  setMiddlePos(
												jsonCmdReceive["id"]
												);break;
	case CMD_SET_SERVO_PID: 
												setServosPID(
												jsonCmdReceive["id"],
												jsonCmdReceive["p"]
												);break;

	// esp-32 dev ctrl.
	case CMD_REBOOT: 			esp_restart();break;
	case CMD_FREE_FLASH_SPACE:
												freeFlashSpace();break;
	case CMD_BOOT_MISSION_INFO:
												missionContent("boot");break;
	case CMD_RESET_BOOT_MISSION:
												deleteFile("boot.mission");break;
												createFile("boot", "these cmds run automatically at boot.");
	case CMD_NVS_CLEAR:		nvs_flash_erase();
												delay(1000);
												nvs_flash_init();
												break;
	case CMD_INFO_PRINT:	configInfoPrint(
												jsonCmdReceive["cmd"]
												);break;
	}
}


void serialCtrl() {
  static String receivedData;

  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    receivedData += receivedChar;

    // Detect the end of the JSON string based on a specific termination character
    if (receivedChar == '\n') {
      // Now we have received the complete JSON string
      DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
      if (err == DeserializationError::Ok) {
  			if (InfoPrint == 1) {
  				Serial.println(receivedData);
  			}
        jsonCmdReceiveHandler();
      } else {
        // Handle JSON parsing error here
      }
      // Reset the receivedData for the next JSON string
      receivedData = "";
    }
  }
}