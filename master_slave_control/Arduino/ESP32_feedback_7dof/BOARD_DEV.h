#include <Wire.h>
TaskHandle_t ScreenUpdateHandle;
TaskHandle_t ServerCmdHandle;
TaskHandle_t EspNowHandle;

// SSD1306: 0x3C
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels, 32 as default.
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// OLED initialization.
void InitScreen(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}


// information update on OLED.
void screenUpdate(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  // Row1.
  display.print(F("MAC:"));display.println(MAC_ADDRESS);
  // Row2.
  display.print(F("V:"));display.print(float(voltageRead[1])/10);display.print(F(" "));display.println(IP_ADDRESS);
  // Row3.
  display.print(F("MODE:"));

  if(DEV_ROLE == 1){display.print(F("L"));}
  else if(DEV_ROLE == 2){display.print(F("F"));}
  else if(DEV_ROLE == 0){display.print(F("N"));}

  if(WIFI_MODE == 1){display.print(F(" AP "));display.println(AP_SSID);}
  else if(WIFI_MODE == 2){display.print(F(" STA "));display.print(F("RSSI"));display.println(WIFI_RSSI);}
  else if(WIFI_MODE == 3){display.print(F(" TRY:"));display.print(STA_SSID);display.println(F(""));}

  // Row4.
  display.println(F("WAVESHARE.NET"));

  display.display();
}


void debugInfo(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  display.print(F("S0:"));display.print(servoPosRead[0]);
  display.print(F("  S1:"));display.println(servoPosRead[1]);
  display.print(F("S2:"));display.print(servoPosRead[2]);
  display.print(F("  S3:"));display.println(servoPosRead[3]);

  display.display();
}


void boardDevInit(){
  Wire.begin(S_SDA, S_SCL);
  InitScreen();
  InitRGB();
}


void InfoUpdateThreading(void *pvParameter){
  while(1){
    screenUpdate();
    delay(10000);
  }
}


void serverThreading(void *pvParameter){
  while(1){
    cmdThreading();
    cmdProcess();
    // espNowProcess();
    delay(2);
  }
}


void espNowThreading(void *pvParameter){
  while(1){
    if(espNowMode ==1 ){
      espNowProcess();
      if(torqueLockStatus){
        delay(50);
      }
      else{
        delay(25);
      }
    }
    else{
      delay(3000);
    }
  }
}


void threadInit(){
  xTaskCreatePinnedToCore(&InfoUpdateThreading, "InfoUpdate", 4000, NULL, 5, &ScreenUpdateHandle, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(&serverThreading, "InfoUpdate", 4000, NULL, 5, &ServerCmdHandle, ARDUINO_RUNNING_CORE);
  // xTaskCreate(&serverThreading, "Server", 4000, NULL, 5, &ServerCmdHandle);
  xTaskCreate(&espNowThreading, "ESP-NOW", 4000, NULL, 5, &EspNowHandle);
}