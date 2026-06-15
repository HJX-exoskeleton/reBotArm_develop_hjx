// ==========================================
// ESP32 缓冲型串口透传程序 (完美解决高速丢包)
// ==========================================

#define S_RXD 18
#define S_TXD 19

void setup() {
  // 1. 【降速求稳】电脑连 ESP32 用 115200，绝不丢包
  Serial.begin(115200); 
  
  // 2. 【保持极速】ESP32连舵机，必须符合出厂 1M 波特率
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
}

void loop() {
  // 核心优化：使用“块搬运”代替“单字节搬运”，防止数据包被撕裂
  
  // 电脑 -> 舵机
  if (Serial.available()) {
    uint8_t buf[128]; // 创建一个推车
    size_t len = Serial.available(); // 看看有多少货
    Serial.readBytes(buf, len);      // 一次性装车
    Serial1.write(buf, len);         // 一次性发给舵机！
  }
  
  // 舵机 -> 电脑
  if (Serial1.available()) {
    uint8_t buf[128];
    size_t len = Serial1.available();
    Serial1.readBytes(buf, len);
    Serial.write(buf, len);
  }
}
