#include <math.h>


#define M_PI 3.1415926535897932384626433832795

// Set the type of EoAT(End-of-Arm Tooling) here:
// 0-> nothing
// 1-> suction cup
// 2-> grapper
// 3-> electromagnet
int EoAT = 1;

// int args
s16   MiddlePos       = 2047;
float MiddleAngle     = 180;
float ServoPosRange   = 4096;
float ServoAngleRange = 360;

// processing time calculation.
unsigned long timeBefore;
unsigned long timeAfter;
unsigned long timeProce;


// --- --- --- --- --- --- --- --- ---
//
// ^
// |
// Z
// |
// 0===X===>
//               O<-S3
//              / \
//             /   \
//            C     D
//           /       \
//          /         \S4
// X---B---O<-S2       O--E--X- - - -
// |                         |\ /
// A                         F-G(angle)
// |                         |
// O<-S1                     X
//
// --- --- --- --- --- --- --- --- ---
//
// ^
// |
// Y
// |
// O===X===>
//
// X-------------------------X
// |
// H
// |
// O<-S1
//
// --- --- --- --- --- --- --- --- ---
// O: the joints
// X: the structure
float LEN_A = 115.432;
float LEN_B = 41.0513;
float LEN_C = 168.8579;
float LEN_D = 127.9234;
float LEN_E = 108.5357;
float LEN_F = 7.7076;
// float LEN_E = 35;
// float LEN_F = 35;

float LEN_G = 90.0;
float LEN_H = -13.75;

float initPosX = LEN_B + LEN_D + LEN_E;
float initPosY = LEN_H;
float initPosZ = LEN_A + LEN_C - LEN_F;
float initPosT = 90; //90
float rateTZ   = 2;
float rateBASE = 2;
float rateTRO  = 0.8;

float goalPosition[4]   = {initPosX, initPosY, initPosZ, initPosT};
float procPosition[4]   = {initPosX, initPosY, initPosZ, initPosT};
float lastPosition[4]   = {initPosX, initPosY, initPosZ, initPosT};

double AngleList[6] = {0, 180.0, 180.0, 180.0, 180.0, 180.0};
double AngleMax[6]  = {0, 360.0, 360.0, 360.0, 285.0, 280.0};
double AngleMin[6]  = {0,     0,     0,     0,  60.0, 160.0};

s16 posMax[6] = {0,    4095, 4095, 3800, 3200, 4095};
s16 posMin[6] = {0,       0,    0,  600,  750,    0};

s16  posESP[5] = {2047, 2047, 2047, 2047, 2047};
u16  spdESP[5] = {500, 500, 500, 500, 500};
byte accESP[5] = {10, 10, 10, 10, 10};
byte ID_ESP[5] = {1, 2, 3, 4, 5};

float InfoBuffer[20];
// the num of the info in InfoBuffer[].
// you can refer to the doc for more detal info.
uint8_t ModeNum  = 0; // placeHolder.
uint8_t angle_1  = 1; // the angle of servoID.1.
uint8_t angle_2  = 2; // the angle of servoID.2.
uint8_t angle_3  = 3; // the angle of servoID.3.
uint8_t angle_4  = 4; // the angle of servoID.4.
uint8_t angle_IKE   = 5; // the angle of the simpleIK end.
uint8_t angle_EoAT  = 6; // the angle of the EoAT end.
uint8_t len_a    = 7; // the length of EoAT-a.
uint8_t len_b    = 8; // the length of EoAT-b.
uint8_t len_totalXY = 9; // the length of B-C-D-E-F-G(X-Y).

bool nanWiggleIK = false;
bool nanEoAT_IK  = false;
bool nanSimleLIK = false;
bool nanCtrlIK   = false;

bool angleLimitCheckFlag = false;

// set servo ID here.
byte STID[4]    = {1, 2, 3, 4};
s16 STPos[4]    = {MiddlePos, MiddlePos, MiddlePos, MiddlePos};
u16 STSpdNow[4] = {3000, 3000, 3400, 3400};
byte STAcNow[4] = {60, 60, 120, 255};
u16 STSpd[4]    = {0, 0, 0, 0};
byte STAc[4]    = {50, 50, 50, 50};
int STDirection[5] = {1, 1, 1, -1, 1};


// the position 
s16 servoPosRead[5]  = {MiddlePos, MiddlePos, MiddlePos, MiddlePos, MiddlePos};


void torqueCtrlAll(u8 enableCMD){
  st.EnableTorque(1, enableCMD);
  st.EnableTorque(2, enableCMD);
  st.EnableTorque(3, enableCMD);
  st.EnableTorque(4, enableCMD);
  st.EnableTorque(5, enableCMD);
  if(enableCMD){
    torqueLockStatus = true;
  }
  else{
    torqueLockStatus = false;
  }
}


void SetEoAT(int numInput){
  EoAT = numInput;
}


float angleGenOut(float angleInput){
  float posOutput = angleInput*ServoPosRange/ServoAngleRange;
  return posOutput;
}


float angleGenOutReverse(float posInput){
  float angleOutput = posInput*ServoAngleRange/ServoPosRange;
  return angleOutput;
}


// Simple Linkage IK:
// input the position of the end and return angle.
//   O----O
//  /
// O
// ---------------------------------------------------
// |       /beta           /delta                    |
//        O----LB---------X------                    |
// |     /       omega.   |       \LB                |
//      LA        .                < ----------------|
// |alpha     .          bIn         \LB -EP  <delta |
//    /psi.                           \LB -EP        |
// | /.   lambda          |                          |
// O- - - - - aIn - - - - X -                        |
// ---------------------------------------------------
// alpha, beta > 0 ; delta <= 0 ; aIn, bIn > 0
void simpleLinkageIK(double LA, double LB, double aIn, double bIn){
  double psi;
  double alpha;
  double omega;
  double beta;
  double L2C;
  double LC;
  double lambda;
  double delta;
  if(bIn == 0){
    psi   = acos((LA * LA + aIn * aIn - LB * LB)/(2 * LA * aIn)) * 180 / M_PI;
    alpha = 90 - psi;
    omega = acos((aIn * aIn + LB * LB - LA * LA)/(2 * aIn * LB)) * 180 / M_PI;
    beta  = psi + omega;
  }
  else{
    L2C = aIn * aIn + bIn * bIn;
    LC  = sqrt(L2C);
    lambda = atan(bIn/aIn) * 180 / M_PI;
    psi    = acos((LA * LA + L2C - LB * LB)/(2 * LA * LC)) * 180 / M_PI;
    alpha = 90 - lambda - psi;
    omega = acos((LB * LB + L2C - LA * LA)/(2 * LC * LB)) * 180 / M_PI;
    beta  = psi + omega;
  }
  delta = 90 - alpha - beta;

  if(!isnan(alpha) && !isnan(beta) && !isnan(delta)){
    InfoBuffer[angle_2]   = alpha;
    InfoBuffer[angle_3]   = beta;
    InfoBuffer[angle_IKE] = delta;
    nanSimleLIK = false;
  }
  else{
    nanSimleLIK = true;
    Serial.println("nanSimleLIK...");
    Serial.println(isnan(alpha));
    Serial.println(isnan(beta));
    Serial.println(isnan(delta));
  }
}


// Input the angle of EoAT_IK.
// ---E---X- - - - - -
//         \      /
//          F - angleInput
//           \
//            X
//
// Output the angle of the EoAT_IK end.
// Output the relative position of the EoAT_IK end.
// X - - - - - - - - - -
//  \     /           |
//   E - betaGenOut
//    \               |
//     X              b
//     |              |
//     F
//     |              |
// --a-X- - - - - - - - 
void EoAT_IK(float angleInput){
  // float betaGenOut = angleInput - LEN_G;
  // float betaRad    = betaGenOut*M_PI/180;
  // float angleRad   = angleInput*M_PI/180;
  float betaGenOut;
  float betaRad;
  float angleRad;
  float aGenOut;
  float bGenOut;

  if(angleInput == 90){
    betaGenOut = angleInput - LEN_G;
    betaRad    = betaGenOut*M_PI/180;
    angleRad   = angleInput*M_PI/180;
    aGenOut = LEN_E;
    bGenOut = LEN_F;
    // Serial.println("---=90---");
  }
  else if(angleInput < 90){
    betaGenOut = 90 - angleInput;
    betaRad    = betaGenOut*M_PI/180;
    angleRad   = angleInput*M_PI/180;
    aGenOut = cos(angleRad)*LEN_F + cos(betaRad)*LEN_E;
    bGenOut = sin(angleRad)*LEN_F - sin(betaRad)*LEN_E;
    betaGenOut = -betaGenOut;
    // Serial.println("---<90---");
  }
  else if(angleInput > 90){
    betaGenOut = LEN_G - (180 -angleInput);
    betaRad    = betaGenOut*M_PI/180;
    angleRad   = angleInput*M_PI/180;
    aGenOut = - cos(M_PI-angleRad)*LEN_F + cos(betaRad)*LEN_E;
    bGenOut = sin(M_PI-angleRad)*LEN_F + sin(betaRad)*LEN_E;
    // Serial.println("--->90---");
  }
  if(!isnan(angle_EoAT)){
    InfoBuffer[angle_EoAT] = betaGenOut;
    InfoBuffer[len_a] = aGenOut;
    InfoBuffer[len_b] = bGenOut;
    nanEoAT_IK = false;
  }
  else{
    nanEoAT_IK = true;
    Serial.println("nanEoAT_IK...");
  }
}


// Wiggle Plane IK:
// input the position of the end and return angle.
// O-----X
//       |
//       |
//       O
// ------------------------------
//     X                        |
//    /    .                    
//  LA         .                |
//  /alpha         .LB         
// O- - - - - - - - - -.- - - -X|
//                         .  bIn
// ------------aIn-------------X|
// ------------------------------
// alpha, aIn, bIn > 0
// wigglePlaneIK(Linkage_W, zPos, yPos, wiggleAlpha, wiggleLen);
void wigglePlaneIK(double LA, double aIn, double bIn){
  double LB;
  double L2C;
  double LC;
  double alpha;
  double beta;
  double lambda;
  double psi;
  bIn = -bIn;
  if(bIn > 0){
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * 180 / M_PI;
    psi = acos(LA/LC) * 180 / M_PI;
    LB = sqrt(L2C - LA * LA);
    alpha = psi + lambda - 90;
  }
  else if(bIn == 0){
    alpha = 90+ asin(LA/aIn) * 180 / M_PI;
    L2C = aIn * aIn + bIn * bIn;
    LB = sqrt(L2C);
  }
  else if(bIn < 0){
    bIn = -bIn;
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * 180 / M_PI;
    psi = acos(LA/LC) * 180 / M_PI;
    LB = sqrt(L2C - LA * LA);
    alpha = 90 - lambda + psi;
  }
  if(!isnan(alpha)){
    InfoBuffer[angle_1] = alpha + 90;
    InfoBuffer[len_totalXY] = LB - LEN_B;
    nanWiggleIK = false;
  }
  else{
    nanWiggleIK = true;
    Serial.println("nanWiggleIK...");
  }
}


// bool angleLimitCheck(){
//   bool angleLimit_1 = false;
//   bool angleLimit_2 = false;
//   bool angleLimit_3 = false;
//   bool angleLimit_4 = false;
//   bool angleLimit_5 = false;

//   if(STDirection[0]*InfoBuffer[angle_1] < AngleMin[1] || STDirection[0]*InfoBuffer[angle_1] > AngleMax[1]){
//     angleLimit_1 = true;
//   }
//   else{
//     angleLimit_1 = false;
//   }
//   if(STDirection[1]*InfoBuffer[angle_2] < AngleMin[2] || STDirection[1]*InfoBuffer[angle_2] > AngleMax[2]){
//     angleLimit_2 = true;
//   }
//   else{
//     angleLimit_2 = false;
//   }
//   if(STDirection[2]*InfoBuffer[angle_3] < AngleMin[3] || STDirection[2]*InfoBuffer[angle_3] > AngleMax[3]){
//     angleLimit_3 = true;
//   }
//   else{
//     angleLimit_3 = false;
//   }
//   if(STDirection[3]*InfoBuffer[angle_4] < AngleMin[4] || STDirection[3]*InfoBuffer[angle_4] > AngleMax[4]){
//     angleLimit_4 = true;
//   }
//   else{
//     angleLimit_4 = false;
//   }

//   if(angleLimit_1 || angleLimit_2 || angleLimit_3 || angleLimit_4){
//     angleLimitCheckFlag = true;
//     return true;
//   }
//   else{
//     angleLimitCheckFlag = false;
//     return false;
//   }
// }


void ctrlMoveCheck(){
  STPos[0] = (int)(angleGenOut(STDirection[0]*InfoBuffer[angle_1]) + 0.5);
  STPos[1] = (int)(angleGenOut(STDirection[1]*InfoBuffer[angle_2]+15)*3 + 0.5);
  STPos[2] = (int)(angleGenOut(STDirection[2]*InfoBuffer[angle_3]+180)  + 0.5);
  STPos[3] = (int)(angleGenOut(STDirection[3]*InfoBuffer[angle_4]+180)  + 0.5);
}


bool angleLimitCheck(){
  bool angleLimit_1 = false;
  bool angleLimit_2 = false;
  bool angleLimit_3 = false;
  bool angleLimit_4 = false;
  bool angleLimit_5 = false;

  ctrlMoveCheck();

  if(STPos[0] < posMin[1] || STPos[0] > posMax[1]){
    angleLimit_1 = true;
  }
  else{
    angleLimit_1 = false;
  }
  if(STPos[1] < posMin[2] || STPos[1] > posMax[2]){
    angleLimit_2 = true;
  }
  else{
    angleLimit_2 = false;
  }
  if(STPos[2] < posMin[3] || STPos[2] > posMax[3]){
    angleLimit_3 = true;
  }
  else{
    angleLimit_3 = false;
  }
  if(STPos[3] < posMin[4] || STPos[3] > posMax[4]){
    angleLimit_4 = true;
  }
  else{
    angleLimit_4 = false;
  }

  if(angleLimit_1 || angleLimit_2 || angleLimit_3 || angleLimit_4){
    angleLimitCheckFlag = true;
    return true;
  }
  else{
    angleLimitCheckFlag = false;
    return false;
  }
} 



// top
// |   left
// |  /
// Z Y
// |/
// O---X---fornt
// input the XYZ position and the theta(the EoAT angle).
void ctrlIK(float InputX, float InputY, float InputZ, float InputTheta){
  // timeBefore = millis();
  InputTheta = InputTheta/rateTZ;
  wigglePlaneIK(LEN_H, InputX, InputY);
  EoAT_IK(InputTheta);
  simpleLinkageIK(LEN_C, LEN_D, (InfoBuffer[len_totalXY]-InfoBuffer[len_a]), (InputZ - LEN_A + InfoBuffer[len_b]));
  if(nanSimleLIK || nanWiggleIK || nanEoAT_IK){
    nanCtrlIK = true;
    Serial.println("nanCtrlIK...");
  }
  else{
    InfoBuffer[angle_4] = InfoBuffer[angle_IKE] + InfoBuffer[angle_EoAT];
    nanCtrlIK = false;
  }
}


// use ctrlIK() to get the angle and use ctrlMove() to move the servo.
// Input the speed of each servo.
void ctrlMove(){
  STPos[0] = (int)(angleGenOut(STDirection[0]*InfoBuffer[angle_1]) + 0.5);
  STPos[1] = (int)(angleGenOut(STDirection[1]*InfoBuffer[angle_2]+15)*3 + 0.5);
  STPos[2] = (int)(angleGenOut(STDirection[2]*InfoBuffer[angle_3]+180)  + 0.5);
  STPos[3] = (int)(angleGenOut(STDirection[3]*InfoBuffer[angle_4]+180)  + 0.5);
  st.SyncWritePosEx(STID, 4, STPos, STSpd, STAc);
}


// update new positions in lastPos[].
void lastPositionUpdate(){
  lastPosition[0] = goalPosition[0];
  lastPosition[1] = goalPosition[1];
  lastPosition[2] = goalPosition[2];
  lastPosition[3] = goalPosition[3];
}


// set the move speed.
void ctrlSpeed(u16 Spd_1, u16 Spd_2, u16 Spd_3, u16 Spd_4){
  STSpd[0] = Spd_1;
  STSpd[1] = Spd_2;
  STSpd[2] = Spd_3;
  STSpd[3] = Spd_4;
}


// key frames ctrl.
// 0 <= rateInput <= 1
// use it fuction to ctrl a number change from numStart to numEnd.
float linearCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*rateInput + numStart;
  return numOut;
}


// linearCtrl() function is a simple example, which shows how besselCtrl works.
float besselCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*((cos(rateInput*M_PI-M_PI)+1)/2) + numStart;
  return numOut;
}


// cmdNum: 0->X, 1->Y, 2->Z, 3->Theta
// 0<speedInput<1, 1 can be devided by speedInput.
void simpleBesselCtrl(int cmdNum, float changeInput, float speedInput){
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  if(nanCtrlIK){
    return;
  }
  if(cmdNum == 3){changeInput *= rateTZ;}
  goalPosition[cmdNum] += changeInput;
  for(float i=0;i<=1;i+=speedInput){
    ctrlIK(besselCtrl(lastPosition[0], goalPosition[0], i), 
           besselCtrl(lastPosition[1], goalPosition[1], i), 
           besselCtrl(lastPosition[2], goalPosition[2], i), 
           besselCtrl(lastPosition[3], goalPosition[3], i));
    if(nanCtrlIK){
      return;
    }
    ctrlMove();
    delay(2);
  }
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  ctrlMove();
  lastPositionUpdate();
}


void simpleLinearCtrl(int cmdNum, float changeInput, float speedInput){
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  if(nanCtrlIK){
    return;
  }
  if(cmdNum == 3){changeInput *= rateTZ;}
  goalPosition[cmdNum] += changeInput;
  for(float i=0;i<=1;i+=speedInput){
    ctrlIK(linearCtrl(lastPosition[0], goalPosition[0], i), 
           linearCtrl(lastPosition[1], goalPosition[1], i), 
           linearCtrl(lastPosition[2], goalPosition[2], i), 
           linearCtrl(lastPosition[3], goalPosition[3], i));
    if(nanCtrlIK){
      return;
    }
    ctrlMove();
    delay(10);
  }
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  ctrlMove();
  lastPositionUpdate();
}


// ctrl the robot arm to move to the init position.
void moveInitPos(float speedInput){
  goalPosition[0] = initPosX;
  goalPosition[1] = initPosY;
  goalPosition[2] = initPosZ;
  goalPosition[3] = initPosT*rateTZ;
  for(float i=0;i<=1;i+=speedInput){
    ctrlIK(besselCtrl(lastPosition[0], goalPosition[0], i), 
           besselCtrl(lastPosition[1], goalPosition[1], i), 
           besselCtrl(lastPosition[2], goalPosition[2], i), 
           besselCtrl(lastPosition[3], goalPosition[3], i));
    if(nanCtrlIK){
      return;
    }
    ctrlMove();
  }
  lastPositionUpdate();
}


float maxNumInArray(){
  float deltaPos[4] = {abs(goalPosition[0] - lastPosition[0]),
                       abs(goalPosition[1] - lastPosition[1]),
                       abs(goalPosition[2] - lastPosition[2]),
                       abs(goalPosition[3] - lastPosition[3])};
  float maxVal = deltaPos[0];
  for(int i = 0; i < (sizeof(deltaPos) / sizeof(deltaPos[0])); i++){
    maxVal = max(deltaPos[i],maxVal);
  }
  return maxVal;
}


// cmdNum: 0->X, 1->Y, 2->Z, 3->Theta
// X,Y,Z: mm
// Theta: degree
void singleAxisCtrl(int cmdNum, float changeInput){
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  if(nanCtrlIK){
    return;
  }
  if(cmdNum == 3){changeInput *= rateTZ;}
  goalPosition[cmdNum] += changeInput;
  float deltaSteps = maxNumInArray();
  for(float i=0;i<=1;i+=1/deltaSteps){
    ctrlIK(linearCtrl(lastPosition[0], goalPosition[0], i), 
           linearCtrl(lastPosition[1], goalPosition[1], i), 
           linearCtrl(lastPosition[2], goalPosition[2], i), 
           linearCtrl(lastPosition[3], goalPosition[3], i));
    if(nanCtrlIK){
      return;
    }
    ctrlMove();
    delay(10);
  }
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  ctrlMove();
  lastPositionUpdate();
}


void singleAxisWebCtrl(int cmdNum, float changeInput){
  if(cmdNum == 1){changeInput *= rateBASE;}
  if(cmdNum == 3){changeInput *= rateTRO;}
  goalPosition[cmdNum] += changeInput;
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  if(nanCtrlIK || angleLimitCheck()){
    goalPosition[0] = lastPosition[0];
    goalPosition[1] = lastPosition[1];
    goalPosition[2] = lastPosition[2];
    goalPosition[3] = lastPosition[3];
    RGBcolor(255,0,0);
    return;
  }
  RGBcolor(0,0,0);
  ctrlMove();
  lastPositionUpdate();
}


// X,Y,Z: mm
// Theta: degree
void allAxisCtrl(float cmdX, float cmdY, float cmdZ, float cmdTheta){
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  if(nanCtrlIK){
    Serial.println("Out of Range...");
    return;
  }
  goalPosition[0] = cmdX;
  goalPosition[1] = cmdY;
  goalPosition[2] = cmdZ;
  goalPosition[3] = cmdTheta*rateTZ;
  float deltaSteps = maxNumInArray();
  for(float i=0;i<=1;i+=1/deltaSteps){
    ctrlIK(linearCtrl(lastPosition[0], goalPosition[0], i), 
           linearCtrl(lastPosition[1], goalPosition[1], i), 
           linearCtrl(lastPosition[2], goalPosition[2], i), 
           linearCtrl(lastPosition[3], goalPosition[3], i));
    if(nanCtrlIK){
      return;
    }

    // Serial.print("1: ");Serial.println(InfoBuffer[angle_1]);
    // Serial.print("2: ");Serial.println(InfoBuffer[angle_2]);
    // Serial.print("3: ");Serial.println(InfoBuffer[angle_3]);
    // Serial.print("4: ");Serial.println(InfoBuffer[angle_4]);

    // if(isnan(InfoBuffer[2])){
    //   Serial.println("beyond the edge!");
    // }

    ctrlMove();
    // if(deltaSteps != )
    delay(10);
  }
  ctrlIK(goalPosition[0], goalPosition[1], goalPosition[2], goalPosition[3]);
  ctrlMove();
  lastPositionUpdate();
}


// get the position by moving the servos.
void allPosGet(){
  if(st.FeedBack(1)!=-1){servoPosRead[0] = st.ReadPos(-1);}
  if(st.FeedBack(2)!=-1){servoPosRead[1] = st.ReadPos(-1);}
  if(st.FeedBack(3)!=-1){servoPosRead[2] = st.ReadPos(-1);}
  if(st.FeedBack(4)!=-1){servoPosRead[3] = st.ReadPos(-1);}
}


// float angleGenOut(float angleInput){
//   float angleOutput = angleInput*ServoPosRange/ServoAngleRange;
//   return angleOutput;
// }

// void ctrlMove(){
//   STPos[0] = (int)(angleGenOut(InfoBuffer[angle_1])  + 0.5);
//   STPos[1] = (int)(angleGenOut(InfoBuffer[angle_2]+15)*3 + 0.5);
//   STPos[2] = (int)(angleGenOut(InfoBuffer[angle_3]+180)  + 0.5);
//   STPos[3] = (int)(angleGenOut(InfoBuffer[angle_4]+180)  + 0.5);
//   st.SyncWritePosEx(STID, 4, STPos, STSpd, STAc);
// }