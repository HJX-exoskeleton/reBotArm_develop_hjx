#include <SCServo.h>

#define SMS_STS_MAX_TORQUE 16
#define MAX_TORQUE_LIMIT   48
#define SET_MAX_TORQUE     200

// === ST Servo === TypeNum:9
SMS_STS st;
float ServoDigitalRange_ST  = 4095.0;
float ServoAngleRange_ST    = 360.0;
float ServoDigitalMiddle_ST = 2047.0;
#define ServoInitACC_ST      100
#define ServoMaxSpeed_ST     4000
#define MaxSpeed_X_ST        4000
#define ServoInitSpeed_ST    2000

bool ID_Stu[5] = {false, false, false, false, false};

// set serial feedback.
bool serialFeedback = true;
bool feedBackErrorCheck = false;

// set the servo ID list.
byte ID_List[253];
bool Torque_List[253];
int  ServoType[253];

// []: the ID of the servo.
// the buffer of the information read from the active servo.
s16  loadRead[253];
s16  speedRead[253];
byte voltageRead[253];
int  currentRead[253];
s16  posRead[253];
s16  modeRead[253];
s16  temperRead[253];

// []: the num of the active servo.
// use listID[activeNumInList] to get the ID of the active servo.
byte listID[253];
byte searchNum = 0;
bool searchedStatus = false;
bool searchFinished = false;
bool searchCmd      = false;
byte activeNumInList = 0;
s16 activeServoSpeed = 100;
byte servotoSet = 0;

// linkageBuffer to save the angle.
float linkageBuffer[50];

// the buffer of the bytes read from USB-C and servos. 
int usbRead;
int stsRead;


bool getFeedBack(byte servoID){
  if(st.FeedBack(servoID)!=-1){
    posRead[servoID] = st.ReadPos(-1);
    speedRead[servoID] = st.ReadSpeed(-1);
    loadRead[servoID] = st.ReadLoad(-1);
    voltageRead[servoID] = st.ReadVoltage(-1);
    currentRead[servoID] = st.ReadCurrent(-1);
    temperRead[servoID] = st.ReadTemper(-1);
    modeRead[servoID] = st.ReadMode(servoID);
    feedBackErrorCheck = false;
    return true;
  }
  else{
    if(serialFeedback){
      Serial.println("FeedBack err");
    }
    feedBackErrorCheck = true;
    return false;
  }
}


void getFeedBackAll(){
  ID_Stu[0] = getFeedBack(1);
  ID_Stu[1] = getFeedBack(2);
  ID_Stu[2] = getFeedBack(3);
  ID_Stu[3] = getFeedBack(4);
  ID_Stu[4] = getFeedBack(5);
}


void servoInit(){
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  while(!Serial1) {}
  for (int i = 0; i < 6; i++){
    Torque_List[i] = true;
    ServoType[i]   = -1;
  }
}


void setMiddle(byte InputID){
  st.CalibrationOfs(InputID);
  if(InputID == 5){
    st.unLockEprom(5);
    st.writeWord(5, SMS_STS_MAX_TORQUE, SET_MAX_TORQUE);
    st.writeWord(5, MAX_TORQUE_LIMIT, SET_MAX_TORQUE);
    st.LockEprom(InputID);
  }
}


void setMode(byte InputID, byte InputMode){
  if(InputMode == 0){
    st.unLockEprom(InputID);
    st.writeWord(InputID, 11, 4095);
    st.writeByte(InputID, SMS_STS_MODE, InputMode);
    st.LockEprom(InputID);
  }
  else if(InputMode == 3){
    st.unLockEprom(InputID);
    st.writeByte(InputID, SMS_STS_MODE, InputMode);
    st.writeWord(InputID, 11, 0);
    st.LockEprom(InputID);
  }
}


void setID(byte ID_select, byte ID_set){
  st.unLockEprom(ID_select);
  st.writeByte(ID_select, SMS_STS_ID, ID_set);
  st.LockEprom(ID_set);
}


void servoStop(byte servoID){
  st.EnableTorque(servoID, 0);
  delay(10);
  st.EnableTorque(servoID, 1);
}


void emergencyStopProcessing(){
  st.EnableTorque(1, 0);
  st.EnableTorque(2, 0);
  st.EnableTorque(3, 0);
  st.EnableTorque(4, 0);
  st.EnableTorque(5, 0);
  for(int i=0; i<10; i++){
    RGBcolor(255,25,0);
    delay(500);
    RGBcolor(0,0,0);
    delay(500);
  }
  st.EnableTorque(1, 1);
  st.EnableTorque(2, 1);
  st.EnableTorque(3, 1);
  st.EnableTorque(4, 1);
  st.EnableTorque(5, 1);
}


void servoTorque(byte servoID, u8 enableCMD){
  st.EnableTorque(servoID, enableCMD);
}