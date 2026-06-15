#include <ArduinoJson.h>

StaticJsonDocument<400> jsonCmdReceive;
StaticJsonDocument<300> jsonInfoBufferSend;
StaticJsonDocument<200> jsonCmdSend;

StaticJsonDocument<300> jsonAngleSend;
StaticJsonDocument<300> jsonCoordSend;
StaticJsonDocument<300> jsonStPosSend;
StaticJsonDocument<300> jsonDevInfoSend;
StaticJsonDocument<300> jsonStInfoSend;
StaticJsonDocument<300> jsonRecord;

DynamicJsonDocument jsonStepMove(1024);

// EMERGENCY_STOP: {"T":0}

// ANGLE_CTRL: T:cmdType, P1-5:angleInput, S:speedInput, A:accInput
// {"T":1,"P1":180,"P2":0,"P3":90,"P4":90,"P5":180,"S1":200,"S2":200,"S3":200,"S4":200,"S5":200,"A1":60,"A2":60,"A3":60,"A4":60,"A5":60}

// ANGLE_CTRL_INIT: move to angleCtrl initPos.
// {"T":11}

// COORD_CTRL: T:cmdType, P1-3:coordInput, P4:thetaAngle, P5:grabberAngle, S1:stepDelay, S5:grabberSpeed
// {"T":2,"P1":277.5104065,"P2":-13.75,"P3":276.5822754,"P4":90,"P5":180,"S1":10,"S5":200}

// COORD_CTRL_INIT: move to coordCtrl initPos.
// {"T":12}

// ST_POS_CTRL: T:cmdType, P1-5:posInput, S1-5:speedInput, A1-5:accInput.
// {"T":3,"P1":2047,"P2":0,"P3":3500,"P4":1500,"P5":2047,"S1":200,"S2":200,"S3":200,"S4":200,"S5":200,"A1":60,"A2":60,"A3":60,"A4":60,"A5":60}

// ST_POS_CTRL_INIT: move to posCtrl initPos.
// {"T":13}


// GET_DEV_INFO: get device info.
// {"T":4}

// GET_ANGTOR_INFO: get the angle $ torque info of every servo.
// {"T":5}

// GET_INFO_BUFFER: get the buffer of IK.
// {"T":6}

// PLACEHOLDER: Null.
// {"T":7}


// === RECORD_REPLAY ===
// ALL_TORQUE_OFF: {"T":8,"P1":0}
//  ALL_TORQUE_ON: {"T":8,"P1":1}
//  GET_STEP_INFO: {"T":8,"P1":2,"P2":stepNum}
//   MOVE_TO_STEP: {"T":8,"P1":3,"P2":stepNum}
//    REMOVE_STEP: {"T":8,"P1":4,"P2":stepNum}
//     RECORD_POS: {"T":8,"P1":5,"P2":stepNum,"P3":speed(delay every step - 5)}
//   RECORD_DELAY: {"T":8,"P1":15,"P2":stepNum,"P3":delayInput - ms}
//         REPLAY: {"T":8,"P1":6,"P2":stepDelay(delayTime between steps),"P3":loopTime(-1 -forever)}
// NVS_SPACE_LEFT: {"T":8,"P1":7}
//      NVS_CLEAR: {"T":8,"P1":777}


// === ROARM_M1_CONFIG ===
//       CONFIG_ALL_INIT: {"T":9,"P1":6} all servos move to pos2047.
// CONFIG_TORQUE_ALL_OFF: {"T":9,"P1":7}
//  CONFIG_TORQUE_ALL_ON: {"T":9,"P1":8}
//     CONFIG_TORQUE_OFF: {"T":9,"P1":servoNum,"P2":0}
//      CONFIG_TORQUE_ON: {"T":9,"P1":servoNum,"P2":1}
//           CONFIG_MOVE: {"T":9,"P1":servoNum,"P2":2,"P3":PosInput}
//     CONFIG_SET_MIDDLE: {"T":9,"P1":servoNum,"P2":10}

// === HELP ===
// HELP:{"T":10}

// JSON STR: T C1 C2 C3 ...
// T: command types.
#define EMERGENCY_STOP    0 // {"T":0}
#define ANGLE_CTRL        1 // {"T":1,"A1":angleInput}
#define ANGLE_CTRL_INIT  11 // {"T":11}
#define COORD_CTRL        2
#define COORD_CTRL_INIT  12
#define ST_POS_CTRL       3
#define ST_POS_CTRL_INIT 13

#define GET_DEV_INFO     4
#define GET_ANGTOR_INFO  5
#define GET_INFO_BUFFER  6

#define PLACE_HOLDER     7

#define RECORD_REPLAY    8
#define ROARM_M1_CONFIG  9
#define HELP             10


// === === === === === === ===
// record and replay.
// P1
#define ALL_TORQUE_OFF 0
#define ALL_TORQUE_ON  1
#define GET_STEP_INFO  2 // P2:stepNumber.
#define MOVE_TO_STEP   3 // P2:stepNumber.
#define REMOVE_STEP    4 // P2:stepNumber.
#define RECORD_POS     5 // P2:stepNumber.P3:speed.
#define RECORD_DELAY  15 // P2:stepNumber.P3:speed.
#define REPLAY         6 // P2:replaySpeed.
#define NVS_SPACE_LEFT 7
#define NVS_CLEAR    777
// P2 stepNumber./replay speed.
// === === === === === === ===


// === === === === === === ===
// config RoArm-M1
// P1 1-5:ID
#define CONFIG_ALL_INIT       6
#define CONFIG_TORQUE_ALL_OFF 7
#define CONFIG_TORQUE_ALL_ON  8
// P2
#define CONFIG_TORQUE_OFF 0
#define CONFIG_TORQUE_ON  1
#define CONFIG_MOVE       2
#define CONFIG_SET_MIDDLE 10
//P3:move to position.
// === === === === === === ===


byte jsonID[5]       = {1, 2, 3, 4, 5};
s16 jsonPosCtrl[5]   = {(int)(angleGenOut(STDirection[0]*180) + 0.5), 
                       (int)(angleGenOut(STDirection[0]*0+15)*3 + 0.5), 
                       (int)(angleGenOut(STDirection[0]*90+180) + 0.5), 
                       (int)(angleGenOut(STDirection[0]*90+180) + 0.5), 
                       (int)(angleGenOut(STDirection[0]*180) + 0.5)};
u16 jsonSpdCtrl[5]   = {200, 200, 200, 200, 200};
byte jsonAccCtrl[5]  = {60, 60, 60, 60, 60};
s16 jsonMiddlePos[5] = {2047, 2047, 2047, 2047, 2047};
u16 jsonMiddleSpd[5] = {200, 200, 200, 200, 200};

s16 jsonGoalPos[5]   = {(int)(angleGenOut(STDirection[0]*180) + 0.5), 
                       (int)(angleGenOut(STDirection[0]*0+15)*3 + 0.5), 
                       (int)(angleGenOut(STDirection[0]*90+180) + 0.5), 
                       (int)(angleGenOut(STDirection[0]*90+180) + 0.5), 
                       (int)(angleGenOut(STDirection[0]*180) + 0.5)};

s16 jsonLastPos[5]   = {(int)(angleGenOut(STDirection[0]*180) + 0.5), 
                       (int)(angleGenOut(STDirection[0]*0+15)*3 + 0.5), 
                       (int)(angleGenOut(STDirection[0]*90+180) + 0.5), 
                       (int)(angleGenOut(STDirection[0]*90+180) + 0.5), 
                       (int)(angleGenOut(STDirection[0]*180) + 0.5)}; 

s16 stepsCtrlGoal[5] = {2047, 2047, 2047, 2047, 2047};
s16 stepsCtrlLast[5] = {2047, 2047, 2047, 2047, 2047};
s16 stepsCtrlBuffer[5] = {2047, 2047, 2047, 2047, 2047};
u16 stepsCtrlSpeed[5]  = {0, 0, 0, 0, 0};
byte stepsCtrlAcc[5] = {60, 60, 60, 60, 60};
s16 jsonPosList[5] = {2047, 2047, 2047, 2047, 2047};


int jsonCtrlDelay = 10;

int configID = 0;
int configType = 0;


void torqueAll(bool torqueCmd){
  if(torqueCmd){
    st.EnableTorque(1, 1);
    st.EnableTorque(2, 1);
    st.EnableTorque(3, 1);
    st.EnableTorque(4, 1);
    st.EnableTorque(5, 1);
  }
  else{
    st.EnableTorque(1, 0);
    st.EnableTorque(2, 0);
    st.EnableTorque(3, 0);
    st.EnableTorque(4, 0);
    st.EnableTorque(5, 0);
  }
}


void emergencyStop(){
  globalStopFlag = 1;
  RGBcolor(255, 64, 0);
  Serial.println("=== === === Stop! === === ===");
  Serial.println("---    Torque Lock OFF    ---");
  Serial.println("---    HELP: {\"T\":10}     ---");
  torqueAll(false);
  RGBcolor(0, 0, 0);
  for(int i=0;i<29;i++){
    RGBcolor(255, 64, 0);
    Serial.print(">");
    RGBcolor(0, 0, 0);
    delay(200);
  }
  torqueAll(true);
  Serial.println("");
  Serial.println("---    Torque Lock ON     ---");
}


void getAngTorInfo(){
  if(st.FeedBack(1)!=-1){
    jsonStInfoSend["A1"] = angleGenOutReverse(st.ReadPos(-1));
    jsonStInfoSend["T1"] = st.ReadLoad(-1);
  }
  if(st.FeedBack(2)!=-1){
    jsonStInfoSend["A2"] = angleGenOutReverse(st.ReadPos(-1))/3-15;
    jsonStInfoSend["T2"] = st.ReadLoad(-1);
  }
  if(st.FeedBack(3)!=-1){
    jsonStInfoSend["A3"] = angleGenOutReverse(st.ReadPos(-1))-180;
    jsonStInfoSend["T3"] = st.ReadLoad(-1);
  }
  if(st.FeedBack(4)!=-1){
    jsonStInfoSend["A4"] = STDirection[3]*(angleGenOutReverse(st.ReadPos(-1))-180);
    jsonStInfoSend["T4"] = st.ReadLoad(-1);
  }
  if(st.FeedBack(5)!=-1){
    jsonStInfoSend["A5"] = angleGenOutReverse(st.ReadPos(-1));
    jsonStInfoSend["T5"] = st.ReadLoad(-1);
  }

  char angleInfoJson[300];
  serializeJson(jsonStInfoSend, angleInfoJson);
  Serial.println(angleInfoJson);
}


void getInfoBuffer(){
  jsonInfoBufferSend["Static"]     = InfoBuffer[0];
  jsonInfoBufferSend["Angle_1"]    = InfoBuffer[1];
  jsonInfoBufferSend["Angle_2"]    = InfoBuffer[2];
  jsonInfoBufferSend["Angle_3"]    = InfoBuffer[3];
  jsonInfoBufferSend["Angle_4"]    = InfoBuffer[4];
  jsonInfoBufferSend["Angle_IKE"]  = InfoBuffer[5];
  jsonInfoBufferSend["Angle_EoAT"] = InfoBuffer[6];
  jsonInfoBufferSend["EoAT-a"]     = InfoBuffer[7];
  jsonInfoBufferSend["EoAT-b"]     = InfoBuffer[8];
  jsonInfoBufferSend["LengthXY"]   = InfoBuffer[9];

  char getInfoJson[300];
  serializeJson(jsonInfoBufferSend, getInfoJson);
  Serial.println(getInfoJson);
}


void getStPos(){
  if(st.FeedBack(1)!=-1){jsonPosList[0] = st.ReadPos(-1);}
  else{jsonPosList[1] = 2047;}
  if(st.FeedBack(2)!=-1){jsonPosList[1] = st.ReadPos(-1);}
  else{jsonPosList[1] = 2047;}
  if(st.FeedBack(3)!=-1){jsonPosList[2] = st.ReadPos(-1);}
  else{jsonPosList[1] = 2047;}
  if(st.FeedBack(4)!=-1){jsonPosList[3] = st.ReadPos(-1);}
  else{jsonPosList[1] = 2047;}
  if(st.FeedBack(5)!=-1){jsonPosList[4] = st.ReadPos(-1);}
  else{jsonPosList[1] = 2047;}
}


void jsonAngleCtrl(bool setInit){
  if(!setInit){
    jsonPosCtrl[0] = (int)(angleGenOut(STDirection[0]*jsonCmdReceive["P1"].as<float>()) + 0.5);
    jsonPosCtrl[1] = (int)(angleGenOut(STDirection[1]*jsonCmdReceive["P2"].as<float>()+15)*3 + 0.5);
    jsonPosCtrl[2] = (int)(angleGenOut(STDirection[2]*jsonCmdReceive["P3"].as<float>()+180) + 0.5);
    jsonPosCtrl[3] = (int)(angleGenOut(STDirection[3]*jsonCmdReceive["P4"].as<float>()+180) + 0.5);
    jsonPosCtrl[4] = (int)(angleGenOut(STDirection[4]*jsonCmdReceive["P5"].as<float>()) + 0.5);

    jsonSpdCtrl[0] = jsonCmdReceive["S1"].as<int>();
    jsonSpdCtrl[1] = jsonCmdReceive["S2"].as<int>();
    jsonSpdCtrl[2] = jsonCmdReceive["S3"].as<int>();
    jsonSpdCtrl[3] = jsonCmdReceive["S4"].as<int>();
    jsonSpdCtrl[4] = jsonCmdReceive["S5"].as<int>();

    jsonAccCtrl[0] = jsonCmdReceive["A1"].as<int>();
    jsonAccCtrl[1] = jsonCmdReceive["A2"].as<int>();
    jsonAccCtrl[2] = jsonCmdReceive["A3"].as<int>();
    jsonAccCtrl[3] = jsonCmdReceive["A4"].as<int>();
    jsonAccCtrl[4] = jsonCmdReceive["A5"].as<int>();
  }
  else{
    jsonPosCtrl[0] = (int)(angleGenOut(STDirection[0]*180));
    jsonPosCtrl[1] = (int)(angleGenOut(STDirection[1]*0+15)*3);
    jsonPosCtrl[2] = (int)(angleGenOut(STDirection[2]*90+180));
    jsonPosCtrl[3] = (int)(angleGenOut(STDirection[3]*90+180));
    jsonPosCtrl[4] = (int)(angleGenOut(STDirection[4]*180));

    jsonSpdCtrl[0] = 200;
    jsonSpdCtrl[1] = 200;
    jsonSpdCtrl[2] = 200;
    jsonSpdCtrl[3] = 200;
    jsonSpdCtrl[4] = 200;

    jsonAccCtrl[0] = 60;
    jsonAccCtrl[1] = 60;
    jsonAccCtrl[2] = 60;
    jsonAccCtrl[3] = 60;
    jsonAccCtrl[4] = 60;
  }
  st.SyncWritePosEx(jsonID, 5, jsonPosCtrl, jsonSpdCtrl, jsonAccCtrl);

  jsonAngleSend["T"]  = 1;
  jsonAngleSend["P1"] = angleGenOutReverse(jsonPosCtrl[0]);
  jsonAngleSend["P2"] = angleGenOutReverse(jsonPosCtrl[1])/3-15;
  jsonAngleSend["P3"] = angleGenOutReverse(jsonPosCtrl[2])-180;
  jsonAngleSend["P4"] = STDirection[3]*(angleGenOutReverse(jsonPosCtrl[3])-180);
  jsonAngleSend["P5"] = angleGenOutReverse(jsonPosCtrl[4]);

  jsonAngleSend["S1"] = jsonSpdCtrl[0];
  jsonAngleSend["S2"] = jsonSpdCtrl[1];
  jsonAngleSend["S3"] = jsonSpdCtrl[2];
  jsonAngleSend["S4"] = jsonSpdCtrl[3];
  jsonAngleSend["S5"] = jsonSpdCtrl[4];

  jsonAngleSend["A1"] = jsonAccCtrl[0];
  jsonAngleSend["A2"] = jsonAccCtrl[1];
  jsonAngleSend["A3"] = jsonAccCtrl[2];
  jsonAngleSend["A4"] = jsonAccCtrl[3];
  jsonAngleSend["A5"] = jsonAccCtrl[4];

  char angleInfoJson[300];
  serializeJson(jsonAngleSend, angleInfoJson);
  Serial.println(angleInfoJson);
}


void jsonCoordCtrl(bool setInit){
  if(!setInit){
    goalPosition[0] = jsonCmdReceive["P1"].as<float>();
    goalPosition[1] = jsonCmdReceive["P2"].as<float>();
    goalPosition[2] = jsonCmdReceive["P3"].as<float>();
    goalPosition[3] = jsonCmdReceive["P4"].as<float>()*rateTZ;
    jsonPosCtrl[4]  = (int)(angleGenOut(STDirection[4]*jsonCmdReceive["P5"].as<float>())  + 0.5);
    jsonCtrlDelay   = (int)(jsonCmdReceive["S1"].as<int>()+0.5);
    jsonSpdCtrl[4]  = jsonCmdReceive["S5"].as<int>();
  }
  else{
    goalPosition[0] = initPosX;
    goalPosition[1] = initPosY;
    goalPosition[2] = initPosZ;
    goalPosition[3] = initPosT*rateTZ;
    jsonPosCtrl[4]  = (int)(angleGenOut(STDirection[4]*180) + 0.5);
    jsonCtrlDelay   = 10;
    jsonSpdCtrl[4]  = 200;
  }
  st.WritePosEx(5, jsonPosCtrl[4], jsonSpdCtrl[4], 50);
  float deltaSteps = maxNumInArray();
  for(float i=0;i<=1;i+=1/deltaSteps){
    if(processType == EMERGENCY_STOP){
      emergencyStop();
      lastPosition[0] = linearCtrl(lastPosition[0], goalPosition[0], (i-1/deltaSteps));
      lastPosition[1] = linearCtrl(lastPosition[1], goalPosition[1], (i-1/deltaSteps));
      lastPosition[2] = linearCtrl(lastPosition[2], goalPosition[2], (i-1/deltaSteps));
      lastPosition[3] = linearCtrl(lastPosition[3], goalPosition[3], (i-1/deltaSteps));
      return;
    }
    ctrlIK(linearCtrl(lastPosition[0], goalPosition[0], i), 
           linearCtrl(lastPosition[1], goalPosition[1], i), 
           linearCtrl(lastPosition[2], goalPosition[2], i), 
           linearCtrl(lastPosition[3], goalPosition[3], i));
    ctrlMove();
    delay(jsonCtrlDelay);
  }
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  ctrlMove();
  lastPositionUpdate();

  jsonCoordSend["T"]  = 2;
  jsonCoordSend["P1"] = goalPosition[0];
  jsonCoordSend["P2"] = goalPosition[1];
  jsonCoordSend["P3"] = goalPosition[2];
  jsonCoordSend["P4"] = goalPosition[3]/rateTZ;
  jsonCoordSend["P5"] = angleGenOutReverse(jsonPosCtrl[4]);
  jsonCoordSend["S1"] = jsonCtrlDelay;
  jsonCoordSend["S5"] = jsonSpdCtrl[4];

  char coordInfoJson[300];
  serializeJson(jsonCoordSend, coordInfoJson);
  Serial.println(coordInfoJson);
}


void jsonStPosCtrl(bool setInit){
  if(!setInit){
    jsonPosCtrl[0] = jsonCmdReceive["P1"].as<int>();
    jsonPosCtrl[1] = jsonCmdReceive["P2"].as<int>();
    jsonPosCtrl[2] = jsonCmdReceive["P3"].as<int>();
    jsonPosCtrl[3] = jsonCmdReceive["P4"].as<int>();
    jsonPosCtrl[4] = jsonCmdReceive["P5"].as<int>();

    jsonSpdCtrl[0] = jsonCmdReceive["S1"].as<int>();
    jsonSpdCtrl[1] = jsonCmdReceive["S2"].as<int>();
    jsonSpdCtrl[2] = jsonCmdReceive["S3"].as<int>();
    jsonSpdCtrl[3] = jsonCmdReceive["S4"].as<int>();
    jsonSpdCtrl[4] = jsonCmdReceive["S5"].as<int>();

    jsonAccCtrl[0] = jsonCmdReceive["A1"].as<int>();
    jsonAccCtrl[1] = jsonCmdReceive["A2"].as<int>();
    jsonAccCtrl[2] = jsonCmdReceive["A3"].as<int>();
    jsonAccCtrl[3] = jsonCmdReceive["A4"].as<int>();
    jsonAccCtrl[4] = jsonCmdReceive["A5"].as<int>();
  }
  else{
    jsonPosCtrl[0] = 2047;
    jsonPosCtrl[1] = 0;
    jsonPosCtrl[2] = 3500;
    jsonPosCtrl[3] = 1500;
    jsonPosCtrl[4] = 2047;

    jsonSpdCtrl[0] = 200;
    jsonSpdCtrl[1] = 200;
    jsonSpdCtrl[2] = 200;
    jsonSpdCtrl[3] = 200;
    jsonSpdCtrl[4] = 200;

    jsonAccCtrl[0] = 60;
    jsonAccCtrl[1] = 60;
    jsonAccCtrl[2] = 60;
    jsonAccCtrl[3] = 60;
    jsonAccCtrl[4] = 60;
  }

  st.SyncWritePosEx(jsonID, 5, jsonPosCtrl, jsonSpdCtrl, jsonAccCtrl);

  jsonStPosSend["T"]  = 3;
  jsonStPosSend["P1"] = jsonPosCtrl[0];
  jsonStPosSend["P2"] = jsonPosCtrl[1];
  jsonStPosSend["P3"] = jsonPosCtrl[2];
  jsonStPosSend["P4"] = jsonPosCtrl[3];
  jsonStPosSend["P5"] = jsonPosCtrl[4];

  jsonStPosSend["S1"] = jsonSpdCtrl[0];
  jsonStPosSend["S2"] = jsonSpdCtrl[1];
  jsonStPosSend["S3"] = jsonSpdCtrl[2];
  jsonStPosSend["S4"] = jsonSpdCtrl[3];
  jsonStPosSend["S5"] = jsonSpdCtrl[4];

  jsonStPosSend["A1"] = jsonAccCtrl[0];
  jsonStPosSend["A2"] = jsonAccCtrl[1];
  jsonStPosSend["A3"] = jsonAccCtrl[2];
  jsonStPosSend["A4"] = jsonAccCtrl[3];
  jsonStPosSend["A5"] = jsonAccCtrl[4];

  char stPosInfoJson[300];
  serializeJson(jsonStPosSend, stPosInfoJson);
  Serial.println(stPosInfoJson);
}


void getHelp(){
  Serial.println(jsonCtrlInstructions);
}


void getStepInfo(int stepSelect){
  Serial.println("P1: GET_STEP_INFO");
  String stepString  = String(stepSelect);
  String setpInfoGet = preferences.getString(stepString.c_str(), "{\"S\":0}");
  Serial.println(setpInfoGet);
}


int maxDeltaInArray(){
  int deltaPos[4] = {abs(stepsCtrlGoal[0] - stepsCtrlLast[0]),
                     abs(stepsCtrlGoal[1] - stepsCtrlLast[1]),
                     abs(stepsCtrlGoal[2] - stepsCtrlLast[2]),
                     abs(stepsCtrlGoal[3] - stepsCtrlLast[3])};
  int maxVal = deltaPos[0];
  for(int i = 0; i < (sizeof(deltaPos) / sizeof(deltaPos[0])); i++){
    maxVal = max(deltaPos[i],maxVal);
  }
  return maxVal;
}


// stepInfo: {"S":stepStatic,"P1-5":pos,"T":stepStride}
// stepStatic:0 -NULL
// stepStatic:1 -POS
// stepStatic:2 -DELAY {"T":8,"P1":15,"P2":stepNum,"P3":delayInput}
// {"S":stepStatic,"T":stepStride}
void movetoStep(int stepSelect){
  Serial.print("P1: MOVE_TO_STEP:");Serial.println(stepSelect);
  String stepString  = String(stepSelect);
  String setpInfoGet = preferences.getString(stepString.c_str(), "{\"S\":0}");
  Serial.println(setpInfoGet);
  deserializeJson(jsonStepMove, setpInfoGet);
  JsonObject obj = jsonStepMove.as<JsonObject>();
  int stepStatic = jsonStepMove["S"].as<int>();

  if(stepStatic == 1){
    stepsCtrlGoal[0] = jsonStepMove["P1"].as<int>();
    stepsCtrlGoal[1] = jsonStepMove["P2"].as<int>();
    stepsCtrlGoal[2] = jsonStepMove["P3"].as<int>();
    stepsCtrlGoal[3] = jsonStepMove["P4"].as<int>();
    stepsCtrlGoal[4] = jsonStepMove["P5"].as<int>();
    int stepT  = jsonStepMove["T"].as<int>();
    double totalMoveSteps = maxDeltaInArray();
    for(double i=0;i<=1;i+=1/totalMoveSteps){
      if(processType == EMERGENCY_STOP){
        emergencyStop();
        stepsCtrlLast[0] = int(besselCtrl(stepsCtrlLast[0], stepsCtrlGoal[0], (i-1/totalMoveSteps))+0.5);
        stepsCtrlLast[1] = int(besselCtrl(stepsCtrlLast[1], stepsCtrlGoal[1], (i-1/totalMoveSteps))+0.5);
        stepsCtrlLast[2] = int(besselCtrl(stepsCtrlLast[2], stepsCtrlGoal[2], (i-1/totalMoveSteps))+0.5);
        stepsCtrlLast[3] = int(besselCtrl(stepsCtrlLast[3], stepsCtrlGoal[3], (i-1/totalMoveSteps))+0.5);
        stepsCtrlLast[4] = int(besselCtrl(stepsCtrlLast[4], stepsCtrlGoal[4], (i-1/totalMoveSteps))+0.5);
        return;
      }
      stepsCtrlBuffer[0] = int(besselCtrl(stepsCtrlLast[0], stepsCtrlGoal[0], i)+0.5);
      stepsCtrlBuffer[1] = int(besselCtrl(stepsCtrlLast[1], stepsCtrlGoal[1], i)+0.5);
      stepsCtrlBuffer[2] = int(besselCtrl(stepsCtrlLast[2], stepsCtrlGoal[2], i)+0.5);
      stepsCtrlBuffer[3] = int(besselCtrl(stepsCtrlLast[3], stepsCtrlGoal[3], i)+0.5);
      stepsCtrlBuffer[4] = int(besselCtrl(stepsCtrlLast[4], stepsCtrlGoal[4], i)+0.5);
      st.SyncWritePosEx(jsonID, 5, stepsCtrlBuffer, stepsCtrlSpeed, stepsCtrlAcc);
      delay(stepT);
    }
    st.SyncWritePosEx(jsonID, 5, stepsCtrlGoal, stepsCtrlSpeed, stepsCtrlAcc);
    stepsCtrlLast[0] = stepsCtrlGoal[0];
    stepsCtrlLast[1] = stepsCtrlGoal[1];
    stepsCtrlLast[2] = stepsCtrlGoal[2];
    stepsCtrlLast[3] = stepsCtrlGoal[3];
    stepsCtrlLast[4] = stepsCtrlGoal[4];
  }
  else if(stepStatic == 2){
    Serial.println("P1: DELAY_TIME");
    // delay(jsonStepMove["T"].as<int>());
    for(int i=0;i<jsonStepMove["T"].as<int>();i++){
      if(processType == EMERGENCY_STOP){
        emergencyStop();
        return;
      }
      delay(1);
    }
  }
}


void removeStep(int stepSelect){
  Serial.println("P1: REMOVE_STEP");
  String stepString  = String(stepSelect);
  preferences.putString(stepString.c_str(), "{\"S\":0}");
}


void recordPos(int stepSelect, int strideInput){
  Serial.println("P1: RECORD_POS");
  if(stepSelect>stepNumMax){preferences.putInt("stepNumMax",stepSelect);stepNumMax = stepSelect;}
  String stepString  = String(stepSelect);
  getStPos();
  jsonRecord["S"]  = 1;
  jsonRecord["P1"] = jsonPosList[0];
  jsonRecord["P2"] = jsonPosList[1];
  jsonRecord["P3"] = jsonPosList[2];
  jsonRecord["P4"] = jsonPosList[3];
  jsonRecord["P5"] = jsonPosList[4];
  jsonRecord["T"]  = strideInput;
  stepsCtrlLast[0] = jsonPosList[0];
  stepsCtrlLast[1] = jsonPosList[1];
  stepsCtrlLast[2] = jsonPosList[2];
  stepsCtrlLast[3] = jsonPosList[3];
  stepsCtrlLast[4] = jsonPosList[4];
  char recordJsonBuffer[300];
  serializeJson(jsonRecord, recordJsonBuffer);
  preferences.putString(stepString.c_str(), String(recordJsonBuffer));
}


void recordDelay(int stepSelect, int delayInput){
  Serial.println("P1: RECORD_DELAY");
  if(stepSelect>stepNumMax){preferences.putInt("stepNumMax",stepSelect);stepNumMax = stepSelect;}
  String stepString  = String(stepSelect);
  jsonRecord["S"]    = 2;
  jsonRecord["T"]    = delayInput;
  char recordJsonBuffer[300];
  serializeJson(jsonRecord, recordJsonBuffer);
  preferences.putString(stepString.c_str(), String(recordJsonBuffer));
}


void replay(int stepDelay, int loopTime){
  Serial.println("P1: REPLAY");
  Serial.print("stepNumMax:");Serial.println(stepNumMax);
  if(loopTime == -1){
    while(processType!=EMERGENCY_STOP){
      for(int i=0;i<=stepNumMax;i++){
        if(processType == EMERGENCY_STOP){emergencyStop();break;return;}
        movetoStep(i);
        for(int i=0;i<stepDelay;i++){
          if(processType == EMERGENCY_STOP){emergencyStop();break;return;}
          delay(1);
        }
      }
    }
  }
  else{
    for(int t=0;t<loopTime;t++){
      for(int i=0;i<=stepNumMax;i++){
        if(processType == EMERGENCY_STOP){emergencyStop();break;return;}
        movetoStep(i);
        for(int i=0;i<stepDelay;i++){
          if(processType == EMERGENCY_STOP){emergencyStop();break;return;}
          delay(1);
        }
      }
    }
  }
}


void recordReplay(){
  switch(jsonCmdReceive["P1"].as<int>()){
    case ALL_TORQUE_OFF:torqueAll(false);break;
    case  ALL_TORQUE_ON:torqueAll(true);break;
    case  GET_STEP_INFO:getStepInfo(jsonCmdReceive["P2"].as<int>());break;
    case   MOVE_TO_STEP:movetoStep(jsonCmdReceive["P2"].as<int>());break;
    case    REMOVE_STEP:removeStep(jsonCmdReceive["P2"].as<int>());break;
    case     RECORD_POS:recordPos(jsonCmdReceive["P2"].as<int>(), jsonCmdReceive["P3"].as<int>());break;
    case   RECORD_DELAY:recordDelay(jsonCmdReceive["P2"].as<int>(), jsonCmdReceive["P3"].as<int>());break;
    case         REPLAY:replay(jsonCmdReceive["P2"].as<int>(), jsonCmdReceive["P3"].as<int>());break;
    case NVS_SPACE_LEFT:getFreeNvsSpace();break;
    case      NVS_CLEAR:nvsClear();break;
  }
}


void configRoArm(){
  configID   = jsonCmdReceive["P1"].as<int>();
  configType = jsonCmdReceive["P2"].as<int>();
  int configMove = jsonCmdReceive["P3"].as<int>();

  if(configID < 6){
    if(configType == CONFIG_TORQUE_OFF){st.EnableTorque(configID, 0);}
    else if(configType == CONFIG_TORQUE_ON){st.EnableTorque(configID, 1);}
    else if(configType == CONFIG_MOVE){st.WritePosEx(configID, configMove, 500, 50);}
    else if(configType == CONFIG_SET_MIDDLE){st.CalibrationOfs(configID);}
  }
  else{
    if(configID == CONFIG_ALL_INIT){st.SyncWritePosEx(jsonID, 5, jsonMiddlePos, jsonMiddleSpd, jsonAccCtrl);}
    else if(configID == CONFIG_TORQUE_ALL_OFF){torqueAll(false);}
    else if(configID == CONFIG_TORQUE_ALL_ON){torqueAll(true);}
  }
}


void cmdHandler(){
  if(CtrlModeSelect != CTRL_VIA_SERIAL){
    return;
  }
  int cmdType = jsonCmdReceive["T"].as<int>();
  Serial.print("T:");Serial.print(cmdType);Serial.print(" ");
  switch(cmdType){
    case   EMERGENCY_STOP:emergencyStop();Serial.println("EMERGENCY_STOP");break;
    case       ANGLE_CTRL:Serial.println("ANGLE_CTRL");break;
    case  ANGLE_CTRL_INIT:Serial.println("ANGLE_CTRL_INIT");break;
    case       COORD_CTRL:Serial.println("COORD_CTRL");break;
    case  COORD_CTRL_INIT:Serial.println("COORD_CTRL_INIT");break;
    case      ST_POS_CTRL:Serial.println("ST_POS_CTRL");break;
    case     GET_DEV_INFO:Serial.println("GET_DEV_INFO");break;
    case  GET_ANGTOR_INFO:Serial.println("GET_ANGTOR_INFO");break;
    case ST_POS_CTRL_INIT:Serial.println("ST_POS_CTRL_INIT");break;
    case  GET_INFO_BUFFER:Serial.println("GET_INFO_BUFFER");break;
    case    RECORD_REPLAY:Serial.println("RECORD_REPLAY");break;
    case  ROARM_M1_CONFIG:Serial.println("ROARM_M1_CONFIG");break;
    case             HELP:Serial.println("HELP");break;
  }

  processType = cmdType;
  // switch(cmdType){
  //   case   EMERGENCY_STOP:emergencyStop();break;

  //   case       ANGLE_CTRL:jsonAngleCtrl(false);break;
  //   case  ANGLE_CTRL_INIT:jsonAngleCtrl(true);break;

  //   case       COORD_CTRL:jsonCoordCtrl(false);break;
  //   case  COORD_CTRL_INIT:jsonCoordCtrl(true);break;

  //   case      ST_POS_CTRL:jsonStPosCtrl(false);break;
  //   case ST_POS_CTRL_INIT:jsonStPosCtrl(true);break;

  //   case     GET_DEV_INFO:getDevInfo();break;
  //   case  GET_ANGTOR_INFO:getAngTorInfo();break;
  //   case  GET_INFO_BUFFER:getInfoBuffer();break;

  //   case    RECORD_REPLAY:recordReplay();break;
  //   case  ROARM_M1_CONFIG:configRoArm();break;
  //   case             HELP:getHelp();break;
  // }
}