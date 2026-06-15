// CommandType:
#define EmergencyStop  0
#define TypeAngleCtrl  1
#define TypeTorqueCtrl 2
#define TypeCoordCtrl  3
#define TypeGetInfo    4
#define TypeRecordReplay 5
#define TypeServoConfig  6

#define OpenManual  7
#define RobotConfig 8
#define MainPage    9


// AngeleCtrl - 1
// Command_A: ServoID
// Command_B:
#define AngleCtrlInit 0
#define AngleIncrease 1
#define AngleDecrease 2
#define AngleCtrlStop 3


// TorqueCtrl - 2
// Command_A:
#define TorqueOFF 0
#define TorqueON  1


// CoordCtrl - 3
// Command_A:
#define CoordCtrlInit 0
#define CoordX 1
#define CoordY 2
#define CoordZ 3
#define CoordT 4
#define CoordG 5
// Command_B:
#define CoordIncrease 1
#define CoordDecrease 2
#define CoordCtrlStop 3


// GetInfo - 4
// Command_A:
#define GetDevInfo    1
#define GetAngTorInfo 2


// Record & Replay - 5
// Command_A:
#define RecordStep 1
#define Move2Step  2
#define RemoveStep 3
#define DelayTime  4
#define LoopTimes  5
// Command_B: NumInput


// ServoConfig - 6
// Command_A:
#define ServoConfigStop  0

#define Servo_1_Increase 11
#define Servo_1_Decrease 12
#define Servo_1_Set      10

#define Servo_2_Increase 21
#define Servo_2_Decrease 22
#define Servo_2_Set      20

#define Servo_3_Increase 31
#define Servo_3_Decrease 32
#define Servo_3_Set      30

#define Servo_4_Increase 41
#define Servo_4_Decrease 42
#define Servo_4_Set      40

#define Servo_5_Increase 51
#define Servo_5_Decrease 52
#define Servo_5_Set      50

#define ServoIDSet       1
#define S2AutoConfig     2
#define GetServoID       3

#define AngleCtrlSpeed   4
#define CoordCtrlSpeed   5

// Command_B: ID / Time


// RobotConfig - 8
// Command_A:
#define ESP_NOW_LEADER   1
#define ESP_NOW_FOLLOWER 2

//  1- angleCtrl.
//  3- coordCtrl.
int globalCmdType;

int globalAngleCmdA;
int globalAngleCmdB;

int globalCoordCmdA;
int globalCoordCmdB;

int reverseRange    = 4900;
int torqueThreshold = 300;
int posCheckOffset  = 50;

double speedA = 0.1;

double speedC = 0.1;
double CoordList[6] = {0, 0, 200, 100, 40, 180};

double BasePos[6] = {0, 2047, 2047, 2047, 2047, 2047};

int servoNumDetected = 0;
int servoID_Detected = -1;

int defaultRecordSpeed = 3;
int lastStepNum = 3;
int defaultDelayBetweenSteps = 500;

// void emergencyStop(){
// 	globalStopFlag = 1;
// 	emergencyStopProcessing();
// 	if(DebugMode){Serial.println("EmergencyStop");}
// }


void anglePosInit(){
	AngleList[1] = 180;
	AngleList[2] = 10;
	AngleList[3] = 280;
	AngleList[4] = 135;
	AngleList[5] = 180;
	st.WritePosEx(1, int(angleGenOut(AngleList[1])+0.5), 500, 150);
	st.WritePosEx(2, int(angleGenOut(AngleList[2])+0.5), 1500, 150);
	st.WritePosEx(3, int(angleGenOut(AngleList[3])+0.5), 500, 150);
	st.WritePosEx(4, int(angleGenOut(AngleList[4])+0.5), 500, 150);
	st.WritePosEx(5, int(angleGenOut(AngleList[5])+0.5), 500, 150);
	globalCmdType = TypeAngleCtrl;
	bool posGoal_1 = false;
	bool posGoal_2 = false;
	bool posGoal_3 = false;
	bool posGoal_4 = false;
	bool posGoal_5 = false;
	while(1){
		getFeedBackAll();
		if(feedBackErrorCheck){
			delay(3000);
			return;
		}
		if(posRead[1] > int(angleGenOut(AngleList[1])+0.5) - posCheckOffset && posRead[1] < int(angleGenOut(AngleList[1])+0.5) + posCheckOffset){
			posGoal_1 = true;
		}
		else{
			posGoal_1 = false;
		}
		if(posRead[2] > int(angleGenOut(AngleList[2])+0.5) - posCheckOffset && posRead[2] < int(angleGenOut(AngleList[2])+0.5) + posCheckOffset){
			posGoal_2 = true;
		}
		else{
			posGoal_2 = false;
		}
		if(posRead[3] > int(angleGenOut(AngleList[3])+0.5) - posCheckOffset && posRead[3] < int(angleGenOut(AngleList[3])+0.5) + posCheckOffset){
			posGoal_3 = true;
		}
		else{
			posGoal_3 = false;
		}
		if(posRead[4] > int(angleGenOut(AngleList[4])+0.5) - posCheckOffset && posRead[4] < int(angleGenOut(AngleList[4])+0.5) + posCheckOffset){
			posGoal_4 = true;
		}
		else{
			posGoal_4 = false;
		}
		if(posRead[5] > int(angleGenOut(AngleList[5])+0.5) - posCheckOffset && posRead[5] < int(angleGenOut(AngleList[5])+0.5) + posCheckOffset){
			posGoal_5 = true;
		}
		else{
			posGoal_5 = false;
		}

		if(posGoal_1 && posGoal_2 && posGoal_3 && posGoal_4 && posGoal_5){
			break;
		}
	}
}


void coordPosInit(){
	Serial.println("CoordPosInit");
	CoordList[1] = LEN_B + LEN_D + LEN_E;
	CoordList[2] = LEN_H;
	CoordList[3] = LEN_A + LEN_C - LEN_F;
	CoordList[4] = 90;
	CoordList[5] = 180;
	allAxisCtrl(CoordList[1], CoordList[2], CoordList[3], CoordList[4]);
	st.WritePosEx(5, int(angleGenOut(CoordList[5])+0.5), 500, 150);
	globalCmdType = TypeCoordCtrl;
	bool posGoal_1 = false;
	bool posGoal_2 = false;
	bool posGoal_3 = false;
	bool posGoal_4 = false;
	bool posGoal_5 = false;
	delay(200);
	while(1){
		getFeedBackAll();
		Serial.print("CoordPosInit- posRead 1-5:");
		Serial.println(posGoal_1);
		Serial.println(posGoal_2);
		Serial.println(posGoal_3);
		Serial.println(posGoal_4);
		Serial.println(posGoal_5);

		if(feedBackErrorCheck){
			delay(3000);
			return;
		}
		if(posRead[1] > STPos[0] - posCheckOffset && posRead[1] < STPos[0] + posCheckOffset){
			posGoal_1 = true;
		}
		else{
			posGoal_1 = false;
		}
		if(posRead[2] > STPos[1] - posCheckOffset && posRead[2] < STPos[1] + posCheckOffset){
			posGoal_2 = true;
		}
		else{
			posGoal_2 = false;
		}
		if(posRead[3] > STPos[2] - posCheckOffset && posRead[3] < STPos[2] + posCheckOffset){
			posGoal_3 = true;
		}
		else{
			posGoal_3 = false;
		}
		if(posRead[4] > STPos[3] - posCheckOffset && posRead[4] < STPos[3] + posCheckOffset){
			posGoal_4 = true;
		}
		else{
			posGoal_4 = false;
		}
		if(posRead[5] > int(angleGenOut(CoordList[5])+0.5) - posCheckOffset && posRead[5] < int(angleGenOut(CoordList[5])+0.5) + posCheckOffset){
			posGoal_5 = true;
		}
		else{
			posGoal_5 = false;
		}

		if(posGoal_1 && posGoal_2 && posGoal_3 && posGoal_4 && posGoal_5){
			break;
		}
	}
}

// #continuous change.
void typeAngleCtrl(int inputID, int inputAngleCmd){
	if(globalCmdType != TypeAngleCtrl && inputID != 5){
		globalStopFlag = 1;
		RGBcolor(255,64,0);
		// init
		anglePosInit();
		RGBcolor(0,0,0);
		globalStopFlag = 0;
		globalCmdType = TypeAngleCtrl;
	}
	globalAngleCmdA = inputID;
	globalAngleCmdB = inputAngleCmd;
	if(inputAngleCmd == AngleCtrlStop){
		globalStopFlag = 1;
	}
	else{
		globalStopFlag = 0;
	}
	if(DebugMode){
		Serial.print("AngleCtrl");
		Serial.print(" - ID:");
		Serial.print(inputID);
		Serial.print("  AngleCmd:");
		switch(inputAngleCmd){
			case AngleCtrlInit: Serial.println("AngleCtrlInit"); break;
			case AngleIncrease: Serial.println("Increase");      break;
			case AngleDecrease: Serial.println("Decrease");      break;
			case AngleCtrlStop: Serial.println("AngleCtrlStop"); break;
		}
	}
	switch(inputAngleCmd){
		case AngleCtrlInit: RGBcolor(255,64,0);anglePosInit();RGBcolor(0,0,0); break;
		case AngleCtrlStop: globalStopFlag = 1;
							servoStop(inputID); break;
	}
}


void typeTorqueCtrl(int cmdInput){
	if(DebugMode){
		Serial.print("TorqueCtrl - ");
		switch(cmdInput){
			case TorqueOFF: Serial.println("TorqueOFF"); break;
			case TorqueON:  Serial.println("TorqueON");  break;
		}
	}
	switch(cmdInput){
		case TorqueOFF: torqueCtrlAll(0); break;
		case TorqueON:  torqueCtrlAll(1); break;
	}
}

// #continuous change.
void typeCoordCtrl(int coordSelect, int inputCoordCmd){
	if(globalCmdType != TypeCoordCtrl){
		globalStopFlag = 1;
		RGBcolor(255,64,0);
		// init
		coordPosInit();
		RGBcolor(0,0,0);
		globalStopFlag = 0;
		globalCmdType = TypeCoordCtrl;
	}
	globalCoordCmdA = coordSelect;
	globalCoordCmdB = inputCoordCmd;
	if(inputCoordCmd == CoordCtrlStop){
		globalStopFlag = 1;
	}
	else{
		globalStopFlag = 0;
	}
	if(DebugMode){
		Serial.print("CoordCtrl");
		Serial.print(" - CoordSelect:");
		switch(coordSelect){
			case CoordX: Serial.print('X'); break;
			case CoordY: Serial.print('Y'); break;
			case CoordZ: Serial.print('Z'); break;
			case CoordT: Serial.print('T'); break;
			case CoordG: Serial.print('G'); break;
			case CoordCtrlInit: Serial.println("CoordCtrlInit"); break;
		}
		Serial.print("  CoordCmd:");
		switch(inputCoordCmd){
			case CoordCtrlStop: Serial.println("CoordCtrlStop"); break;
			case CoordIncrease: Serial.println("CoordIncrease"); break;
			case CoordDecrease: Serial.println("CoordDecrease"); break;
		}
	}
	switch(coordSelect){
		case CoordCtrlInit: RGBcolor(255,64,0);coordPosInit();RGBcolor(0,0,0); break;
	}
}


void typeGetInfo(int cmdInput){
	if(DebugMode){
		Serial.print("GetInfo - ");
		switch(cmdInput){
			case GetDevInfo:    Serial.println("Get Device Info");         break;
			case GetAngTorInfo: Serial.println("Get Angle & Torque Info"); break; 
		}
	}
}


void typeRecordReplay(int cmdA, int cmdB){
	if(DebugMode){
		Serial.print("Record & Replay - ");
		switch(cmdA){
			case RecordStep: Serial.print("RecordSetp  StepNum:"); break;
			case Move2Step:  Serial.print("Move2Step  StepNum:");  break;
			case RemoveStep: Serial.print("RemoveStep  StepNum:"); break;
			case DelayTime:  Serial.print("DelayTime:");           break;
			case LoopTimes:  Serial.print("LoopTimes:");           break;
		}
		Serial.println(cmdB);
	}

	switch(cmdA){
		case RecordStep: 
						recordPos(cmdB, defaultRecordSpeed);
						lastStepNum = cmdB;
						break;
		case Move2Step:  
						movetoStep(cmdB);
						break;
		case RemoveStep: 
						removeStep(cmdB);
						break;
		case DelayTime: 
						recordDelay(lastStepNum, cmdB);
						break;
		case LoopTimes: 
						replay(defaultDelayBetweenSteps, cmdB);
						break;
	}
}


void servoIDSet(int InputSetID){
	Serial.println("setting");
	if(servoNumDetected == 1){
		setID(servoID_Detected, InputSetID);
		Serial.println("SetID");
		int PingStatus;
		PingStatus = st.Ping(InputSetID);
		if(PingStatus!=-1){
			Serial.print("New ID set successfully:");
			Serial.println(InputSetID);
		}
	}
	else{
		Serial.println("Make sure there is ONLY ONE servo connected!");
	}
}


void searchServo(){
	servoNumDetected = 0;
	servoID_Detected = -1;
	for(int i = 0; i <= 6; i++){
		int PingStatus;
		PingStatus = st.Ping(i);
		if(PingStatus!=-1){
			servoNumDetected++;
			servoID_Detected = i;
			Serial.print("servoNumDetected:");Serial.println(servoNumDetected);
			Serial.print("servoID_Detected:");Serial.println(servoID_Detected);
		}
	}
}


void S2AutoConfigFunc(){
	AngleList[1] = 180;
	AngleList[3] = 270;
	AngleList[4] =  90;
	AngleList[5] = 180;
	st.WritePosEx(1, int(angleGenOut(AngleList[1])+0.5), 100, 150);
	st.WritePosEx(3, int(angleGenOut(AngleList[3])+0.5), 100, 150);
	st.WritePosEx(4, int(angleGenOut(AngleList[4])+0.5), 100, 150);
	st.WritePosEx(5, int(angleGenOut(AngleList[5])+0.5), 100, 150);

	setMode(2, 3);
	st.WritePosEx(2, -20000, 500, 50);
	delay(1000);
	while(1){
		getFeedBack(2);
		Serial.println(loadRead[2]);
		if(loadRead[2] > torqueThreshold){
			Serial.println("Reaching the edge.");
			servoStop(2);
			st.WritePosEx(2, reverseRange, 1500, 150);
			RGBcolor(255, 32, 0);
			delay(11000);
			setMiddle(2);
			AngleList[2] = 180;
			RGBcolor(0, 0, 0);
			setMode(2, 0);
			break;
		}
	}
	setMode(2, 0);
	anglePosInit();
}


void bootPosCheck(){
  	delay(100);  
	getFeedBackAll();
  	delay(100);
	if(ID_Stu[0] && ID_Stu[1] && ID_Stu[2] && ID_Stu[3] && ID_Stu[4]){
		Serial.println("Servo Status:ok");
	}
	else{
		Serial.println("Failed to ping.");
		Serial.println(ID_Stu[0]);
		Serial.println(ID_Stu[1]);
		Serial.println(ID_Stu[2]);
		Serial.println(ID_Stu[3]);
		Serial.println(ID_Stu[4]);
		return;
	}

	AngleList[1] = 180;
	AngleList[2] = 10;
	AngleList[3] = 270;
	AngleList[4] =  90;
	AngleList[5] = 180;
	st.WritePosEx(1, int(angleGenOut(AngleList[1])+0.5), 100, 150);
	st.WritePosEx(2, int(angleGenOut(AngleList[2])+0.5), 500, 150);
	st.WritePosEx(3, int(angleGenOut(AngleList[3])+0.5), 100, 150);
	st.WritePosEx(4, int(angleGenOut(AngleList[4])+0.5), 100, 150);
	st.WritePosEx(5, int(angleGenOut(AngleList[5])+0.5), 100, 150);

	delay(1000);
	while(1){
		getFeedBack(2);
		if(loadRead[2] > torqueThreshold){
			Serial.println("Reaching the edge.");
			servoStop(2);
			setMode(2, 3);
			st.WritePosEx(2, reverseRange, 1500, 150);
			RGBcolor(255, 32, 0);
			delay(6000);
			setMiddle(2);
			RGBcolor(0, 0, 0);
			setMode(2, 0);
			break;
		}
		else if(posRead[2] > int(angleGenOut(AngleList[2])+0.5) - posCheckOffset && posRead[2] < int(angleGenOut(AngleList[2])+0.5) + posCheckOffset){
			break;
		}
	}
	delay(50);
	anglePosInit();
}


void typeServoConfig(int cmdAInput, int cmdBInput){
	Serial.print("ServoConfig - ");
	switch(cmdAInput){
		case ServoConfigStop:   Serial.println("ServoConfigStop");       break;

		case Servo_1_Increase:  Serial.println("Servo_1_Increase");
							    BasePos[1] += 1;
							    st.WritePosEx(1, BasePos[1], 3400, 150); break;
		case Servo_1_Decrease:  Serial.println("Servo_1_Decrease"); 
							    BasePos[1] -= 1; 
							    st.WritePosEx(1, BasePos[1], 3400, 150); break;
		case Servo_1_Set:       Serial.println("Servo_1_Set");
							    setMiddle(1);
							    BasePos[1] = 2047;AngleList[1] = 180;    break;

		case Servo_2_Increase:  Serial.println("Servo_2_Increase");
							    BasePos[2] += 1; 
							    st.WritePosEx(2, BasePos[2], 3400, 150); break;
		case Servo_2_Decrease:  Serial.println("Servo_2_Decrease");
							    BasePos[2] -= 1; 
							    st.WritePosEx(2, BasePos[2], 3400, 150); break;
		case Servo_2_Set:       Serial.println("Servo_2_Set");
							    setMiddle(2);
							    BasePos[2] = 2047;AngleList[2] = 180;    break;

		case Servo_3_Increase:  Serial.println("Servo_3_Increase");
							    BasePos[3] += 1; 
							    st.WritePosEx(3, BasePos[3], 3400, 150); break;
		case Servo_3_Decrease:  Serial.println("Servo_3_Decrease");
							    BasePos[3] -= 1; 
							    st.WritePosEx(3, BasePos[3], 3400, 150); break;
		case Servo_3_Set:       Serial.println("Servo_3_Set");
							    setMiddle(3);
							    BasePos[3] = 2047;AngleList[3] = 180;    break;

		case Servo_4_Increase:  Serial.println("Servo_4_Increase");
							    BasePos[4] += 1; 
							    st.WritePosEx(4, BasePos[4], 3400, 150); break;
		case Servo_4_Decrease:  Serial.println("Servo_4_Decrease");
							    BasePos[4] -= 1; 
							    st.WritePosEx(4, BasePos[4], 3400, 150); break;
		case Servo_4_Set:       Serial.println("Servo_4_Set");
							    setMiddle(4);
							    BasePos[4] = 2047;AngleList[4] = 180;    break;

		case Servo_5_Increase:  Serial.println("Servo_5_Increase");
							    BasePos[5] += 1; 
							    st.WritePosEx(5, BasePos[5], 3400, 150); break;
		case Servo_5_Decrease:  Serial.println("Servo_5_Decrease");
							    BasePos[5] -= 1; 
							    st.WritePosEx(5, BasePos[5], 3400, 150); break;
		case Servo_5_Set:       Serial.println("Servo_5_Set");
							    setMiddle(5);
							    BasePos[5] = 2047;AngleList[5] = 180;    break;

		case ServoIDSet:        Serial.print("ServoIDSet");
							    Serial.println(cmdBInput);
							    servoIDSet(cmdBInput);          break;
		case S2AutoConfig:      Serial.println("S2AutoConfig");
								S2AutoConfigFunc();                 break;
		case GetServoID:        Serial.println("GetServoID");searchServo(); break;

		case AngleCtrlSpeed:    Serial.println("SetAngleCtrlSpeed");speedA=cmdBInput; break;
		case CoordCtrlSpeed:    Serial.println("SetCoordCtrlSpeed");speedC=cmdBInput; break;
	}
}


void openManual(){
	if(DebugMode){Serial.println("OpenManual");}
}


void robotConfig(int cmdAInput){
	if(DebugMode){Serial.println("RobotConfig");}
	espNowMode = cmdAInput;
}


void mainPage(){
	if(DebugMode){Serial.println("MainPage");}
}


void commandProcessing(int cmdInputType, int cmdInputA, int cmdInputB){
	CtrlModeSelect = CTRL_VIA_WEB;
	switch(cmdInputType){
		case EmergencyStop:    emergencyStop();processType = 0;      	  	break;
		case TypeAngleCtrl:    typeAngleCtrl(cmdInputA, cmdInputB);        	break;
		case TypeTorqueCtrl:   typeTorqueCtrl(cmdInputA);                  	break;
		case TypeCoordCtrl:    typeCoordCtrl(cmdInputA, cmdInputB);    		break;
		case TypeGetInfo:      typeGetInfo(cmdInputA);                 		break;
		case TypeRecordReplay: typeRecordReplay(cmdInputA, cmdInputB); 		break;
		case TypeServoConfig:  typeServoConfig(cmdInputA, cmdInputB);  		break;
		case OpenManual:       openManual();                           		break;
		case RobotConfig:      robotConfig(cmdInputA);                     	break;
		case MainPage:         mainPage();                             		break;
	}
}


void cmdThreading(){
	if(espNowMode == 2){
		return;
	}
	if(globalStopFlag || CtrlModeSelect != CTRL_VIA_WEB){
		return;
	}
	else{
		// RGBcolor(0,16,255);
		if(globalCmdType == TypeAngleCtrl && globalAngleCmdA != 5){
			if(globalAngleCmdB == AngleIncrease){
				AngleList[globalAngleCmdA] += speedA;
				if(AngleList[globalAngleCmdA] > AngleMax[globalAngleCmdA]){
					AngleList[globalAngleCmdA] = AngleMax[globalAngleCmdA];
				}
			}
			else if(globalAngleCmdB == AngleDecrease){
				AngleList[globalAngleCmdA] -= speedA;
				if(AngleList[globalAngleCmdA] < AngleMin[globalAngleCmdA]){
					AngleList[globalAngleCmdA] = AngleMin[globalAngleCmdA];
				}
			}
			int ServoPosGenOut = int(angleGenOut(AngleList[globalAngleCmdA])+0.5);
			BasePos[globalAngleCmdA] = ServoPosGenOut;
			st.WritePosEx(globalAngleCmdA, ServoPosGenOut, 3400, 150);
			return;
			// Serial.println("Status: 1");
		}
		if(globalCmdType == TypeCoordCtrl && globalCoordCmdA != CoordCtrlInit){
			if(globalCoordCmdB == CoordIncrease){
				singleAxisWebCtrl((globalCoordCmdA-1), speedC);
				if(!nanCtrlIK){
					CoordList[globalCoordCmdA] = goalPosition[globalCoordCmdA-1];
				}
			}
			else if(globalCoordCmdB == CoordDecrease){
				singleAxisWebCtrl((globalCoordCmdA-1), -speedC);
				if(!nanCtrlIK){
					CoordList[globalCoordCmdA] = goalPosition[globalCoordCmdA-1];
				}
			}
			CoordList[5] = AngleList[5];
			// return;
			// Serial.println("Status: 2");
		}
		if(globalAngleCmdA == 5 && globalAngleCmdB != AngleCtrlStop){
			if(globalAngleCmdB == AngleIncrease){
				AngleList[globalAngleCmdA] += speedA;
				if(AngleList[globalAngleCmdA] > AngleMax[globalAngleCmdA]){
					AngleList[globalAngleCmdA] = AngleMax[globalAngleCmdA];
				}
			}
			else if(globalAngleCmdB == AngleDecrease){
				AngleList[globalAngleCmdA] -= speedA;
				if(AngleList[globalAngleCmdA] < AngleMin[globalAngleCmdA]){
					AngleList[globalAngleCmdA] = AngleMin[globalAngleCmdA];
				}
			}
			int ServoPosGenOut = int(angleGenOut(AngleList[globalAngleCmdA])+0.5);
			BasePos[globalAngleCmdA] = ServoPosGenOut;
			st.WritePosEx(globalAngleCmdA, ServoPosGenOut, 3400, 150);
			return;
			// Serial.println("Status: 3");
		}
		// RGBcolor(0,0,0);
	}
}