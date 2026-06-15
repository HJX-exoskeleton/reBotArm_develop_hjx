#include <SCServo.h>

// ====================== 串口引脚设置 ======================
// 根据你的 ESP32 实际接线修改
#define S_RXD 18
#define S_TXD 19

// ====================== 舵机 ID 范围设置 ======================
// 读取 ID = 1 ~ 7 的所有舵机
#define SERVO_ID_START 1
#define SERVO_ID_END   7

// ====================== 串口波特率 ======================
#define SERVO_BAUDRATE 1000000
#define USB_BAUDRATE   115200

// ====================== 反馈打印间隔 ======================
#define FEEDBACK_INTERVAL 100   // 单位 ms，100ms 打印一次

// ====================== ST 舵机参数 ======================
// ST/SMS_STS 通常 0~4095 对应 0~360°
float ServoDigitalRange_ST = 4095.0;
float ServoAngleRange_ST   = 360.0;

// ====================== 创建舵机对象 ======================
SMS_STS st;

// ====================== 反馈数据缓存 ======================
// ID 最大一般不超过 252
#define MAX_SERVO_ID 252

s16  loadRead[MAX_SERVO_ID + 1];
s16  speedRead[MAX_SERVO_ID + 1];
byte voltageRead[MAX_SERVO_ID + 1];
int  currentRead[MAX_SERVO_ID + 1];
s16  posRead[MAX_SERVO_ID + 1];
s16  modeRead[MAX_SERVO_ID + 1];
s16  temperRead[MAX_SERVO_ID + 1];

// ====================== 时间变量 ======================
unsigned long lastFeedbackTime = 0;


// ====================== 初始化舵机串口 ======================
void servoInit() {
  Serial1.begin(SERVO_BAUDRATE, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(500);

  Serial.println("Servo Serial1 initialized.");
}


// ====================== 检查 ID 是否有效 ======================
bool isValidServoID(byte servoID) {
  return servoID <= MAX_SERVO_ID;
}


// ====================== 读取单个舵机反馈 ======================
bool getFeedBack(byte servoID) {
  if (!isValidServoID(servoID)) {
    Serial.print("Invalid servo ID: ");
    Serial.println(servoID);
    return false;
  }

  if (st.FeedBack(servoID) != -1) {
    posRead[servoID]     = st.ReadPos(-1);
    speedRead[servoID]   = st.ReadSpeed(-1);
    loadRead[servoID]    = st.ReadLoad(-1);
    voltageRead[servoID] = st.ReadVoltage(-1);
    currentRead[servoID] = st.ReadCurrent(-1);
    temperRead[servoID]  = st.ReadTemper(-1);

    // 部分库版本 ReadMode(-1) 不一定稳定，因此这里保留 servoID 方式
    modeRead[servoID]    = st.ReadMode(servoID);

    return true;
  } 
  else {
    Serial.print("ID=");
    Serial.print(servoID);
    Serial.println(" feedback failed!");
    return false;
  }
}


// ====================== 打印表头 ======================
void printTableHeader() {
  Serial.println("Time(ms) | ID | Pos | Angle(deg) | Speed | Load | Voltage(V) | Current | Temp(C) | Mode");
  Serial.println("------------------------------------------------------------------------------------------");
}


// ====================== 打印单个舵机状态 ======================
void printServoStatus(byte servoID) {
  if (getFeedBack(servoID)) {
    float angle = posRead[servoID] / ServoDigitalRange_ST * ServoAngleRange_ST;
    float voltage = voltageRead[servoID] / 10.0;

    Serial.print(millis());
    Serial.print(" | ");

    Serial.print(servoID);
    Serial.print(" | ");

    Serial.print(posRead[servoID]);
    Serial.print(" | ");

    Serial.print(angle, 2);
    Serial.print(" | ");

    Serial.print(speedRead[servoID]);
    Serial.print(" | ");

    Serial.print(loadRead[servoID]);
    Serial.print(" | ");

    Serial.print(voltage, 1);
    Serial.print(" | ");

    Serial.print(currentRead[servoID]);
    Serial.print(" | ");

    Serial.print(temperRead[servoID]);
    Serial.print(" | ");

    Serial.println(modeRead[servoID]);
  }
}


// ====================== 打印所有舵机状态 ======================
void printAllServoStatus() {
  printTableHeader();

  for (byte id = SERVO_ID_START; id <= SERVO_ID_END; id++) {
    printServoStatus(id);
    delay(5);  // 给总线一点间隔，避免连续读取过快
  }

  Serial.println("------------------------------------------------------------------------------------------");
  Serial.println();
}


// ====================== 释放单个舵机力矩 ======================
void releaseSingleServoTorque(byte servoID) {
  Serial.print("Releasing torque of servo ID=");
  Serial.println(servoID);

  st.EnableTorque(servoID, 0);
  delay(100);
}


// ====================== 释放所有舵机力矩 ======================
void releaseAllServoTorque() {
  Serial.println("Releasing all servo torque...");

  for (byte id = SERVO_ID_START; id <= SERVO_ID_END; id++) {
    releaseSingleServoTorque(id);
  }

  Serial.println("All torque disabled. You can rotate the servos by hand now.");
}


// ====================== setup ======================
void setup() {
  Serial.begin(USB_BAUDRATE);
  delay(1000);

  Serial.println();
  Serial.println("====================================================");
  Serial.println("ESP32 ST/SMS_STS Servo Feedback Monitor");
  Serial.println("Servo ID Range: 1 ~ 7");
  Serial.println("No motion command will be sent.");
  Serial.println("Only torque release command will be sent at startup.");
  Serial.println("Rotate the servos by hand to observe feedback data.");
  Serial.println("====================================================");

  servoInit();

  // 上电后关闭 1~7 号舵机力矩，保证舵机不主动运动、不锁死
  releaseAllServoTorque();

  // 上电先读一次状态
  Serial.println("Initial feedback:");
  printAllServoStatus();
}


// ====================== loop ======================
void loop() {
  unsigned long now = millis();

  if (now - lastFeedbackTime >= FEEDBACK_INTERVAL) {
    lastFeedbackTime = now;

    // 不发送任何运动指令，只读取 ID=1~7 的反馈
    printAllServoStatus();
  }
}
