#include <SCServo.h>

// ====================== 串口引脚设置 ======================
// 按照你的 ESP32 实际接线修改
#define S_RXD 18
#define S_TXD 19

// ====================== 修改这里 ======================
// 当前舵机 ID
#define OLD_ID 6

// 想要设置的新 ID
#define NEW_ID 7

// ====================== 串口波特率 ======================
#define SERVO_BAUDRATE 1000000
#define USB_BAUDRATE   115200

SMS_STS st;


// ====================== 初始化舵机串口 ======================
void servoInit() {
  Serial1.begin(SERVO_BAUDRATE, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(500);

  Serial.println("Servo serial initialized.");
}


// ====================== 读取舵机反馈，检查是否在线 ======================
bool checkServo(byte servoID) {
  if (st.FeedBack(servoID) != -1) {
    int pos = st.ReadPos(-1);
    int speed = st.ReadSpeed(-1);
    int voltage = st.ReadVoltage(-1);
    int temp = st.ReadTemper(-1);

    Serial.print("Servo ID ");
    Serial.print(servoID);
    Serial.println(" found.");

    Serial.print("Pos=");
    Serial.print(pos);

    Serial.print(" | Speed=");
    Serial.print(speed);

    Serial.print(" | Voltage=");
    Serial.print(voltage / 10.0);
    Serial.print(" V");

    Serial.print(" | Temp=");
    Serial.print(temp);
    Serial.println(" C");

    return true;
  } 
  else {
    Serial.print("Servo ID ");
    Serial.print(servoID);
    Serial.println(" not found.");
    return false;
  }
}


// ====================== 修改舵机 ID ======================
void setServoID(byte oldID, byte newID) {
  Serial.println();
  Serial.println("Start changing servo ID...");
  Serial.print("Old ID: ");
  Serial.println(oldID);
  Serial.print("New ID: ");
  Serial.println(newID);

  // 先检查旧 ID 是否存在
  if (!checkServo(oldID)) {
    Serial.println("Old ID servo not found. ID change failed.");
    return;
  }

  delay(500);

  // 解锁 EEPROM
  st.unLockEprom(oldID);
  delay(100);

  // 写入新 ID
  st.writeByte(oldID, SMS_STS_ID, newID);
  delay(300);

  // 注意：写入 ID 后，舵机已经变成 newID 了
  // 所以后面锁 EEPROM 要用 newID
  st.LockEprom(newID);
  delay(500);

  Serial.println("ID write command finished.");
  Serial.println("Now checking new ID...");

  // 检查新 ID 是否生效
  if (checkServo(newID)) {
    Serial.println("ID changed successfully!");
  } 
  else {
    Serial.println("New ID feedback failed. Please power cycle the servo and check again.");
  }

  Serial.println();
  Serial.println("Important: After success, remove this ID-changing code or do not input SET again.");
}


// ====================== setup ======================
void setup() {
  Serial.begin(USB_BAUDRATE);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("ST/SMS_STS Servo ID Setting Program");
  Serial.println("======================================");

  servoInit();

  Serial.println();
  Serial.println("Before changing ID, make sure:");
  Serial.println("1. Only ONE servo is connected to the bus.");
  Serial.println("2. Servo power supply is stable.");
  Serial.println("3. OLD_ID and NEW_ID are correctly set in code.");
  Serial.println();

  Serial.print("Current OLD_ID = ");
  Serial.println(OLD_ID);

  Serial.print("Target  NEW_ID = ");
  Serial.println(NEW_ID);

  Serial.println();
  Serial.println("Checking old ID...");
  checkServo(OLD_ID);

  Serial.println();
  Serial.println("If everything is correct, type SET in Serial Monitor and press Enter.");
}


// ====================== loop ======================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "SET") {
      setServoID(OLD_ID, NEW_ID);
    } 
    else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      Serial.println("Type SET to change servo ID.");
    }
  }
}