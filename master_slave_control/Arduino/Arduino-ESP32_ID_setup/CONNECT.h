// https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/


void handleRoot(){
  server.send(200, "text/html", index_html); //Send web page
}


void handleStop(){
  emergencyStop();
}


void handleDataFeedBack(){
  char response[1024];
  DynamicJsonDocument jsonResponse(1024);
  if(globalCmdType == 1){
    if(torqueLockStatus){
      jsonResponse["A1"] = AngleList[1];
      jsonResponse["A2"] = AngleList[2];
      jsonResponse["A3"] = AngleList[3];
      jsonResponse["A4"] = AngleList[4];
      jsonResponse["A5"] = AngleList[5];
    }
    else{
      getFeedBackAll();
      jsonResponse["A1"] = posRead[1];
      jsonResponse["A2"] = posRead[2];
      jsonResponse["A3"] = posRead[3];
      jsonResponse["A4"] = posRead[4];
      jsonResponse["A5"] = posRead[5];
    }
    jsonResponse["C1"] = "None";
    jsonResponse["C2"] = "None";
    jsonResponse["C3"] = "None";
    jsonResponse["C4"] = "None";
    jsonResponse["C5"] = "None";
  }
  else if(globalCmdType == 3){
    jsonResponse["A1"] = "None";
    jsonResponse["A2"] = "None";
    jsonResponse["A3"] = "None";
    jsonResponse["A4"] = "None";
    jsonResponse["A5"] = "None";
    jsonResponse["C1"] = CoordList[1];
    jsonResponse["C2"] = CoordList[2];
    jsonResponse["C3"] = CoordList[3];
    jsonResponse["C4"] = CoordList[4];
    jsonResponse["C5"] = AngleList[5];
  }
  serializeJson(jsonResponse, response);
  server.send(200, "text/plane", response);
}


void handleGetDevInfo(){
  char response[1024];
  DynamicJsonDocument jsonResponse(1024);
  jsonResponse["DeviceName"]  = DEV_NAME;
  jsonResponse["CompanyName"] = COMPANY_NAME;
  jsonResponse["MoreInfo"]    = MORE_INFO;
  jsonResponse["GithubLink"]  = GITHUB_LINK;
  jsonResponse["IP"] = WiFi.localIP().toString();

  if(DEFAULT_WIFI_MODE == 1){
    jsonResponse["DefaultWifiMode"] = "AP";
    jsonResponse["AP_SSID"] = AP_SSID;
    jsonResponse["AP_PWD"]  = AP_PWD;
  }
  else if(DEFAULT_WIFI_MODE == 2){
    jsonResponse["DefaultWifiMode"] = "STA";
    jsonResponse["STA_SSID"] = WiFi.SSID();
    jsonResponse["STA_PWD"]  = STA_PWD;
  }

  if(DEFAULT_ROLE == 0){
    jsonResponse["DefaultRole"] = "Normal";
  }
  else if(DEFAULT_ROLE == 1){
    jsonResponse["DefaultRole"] = "Leader";
  }
  else if(DEFAULT_ROLE == 2){
    jsonResponse["DefaultRole"] = "Follower";
  }

  serializeJson(jsonResponse, response);
  server.send(200, "text/plane", response);
}


void handleGetAngTorInfo(){
  if(espNowMode == 3){
    getFeedBackAll();
    char response[1024];
    DynamicJsonDocument jsonResponse(1024);
    jsonResponse["A1"] = posRead[1];
    jsonResponse["A2"] = posRead[2];
    jsonResponse["A3"] = posRead[3];
    jsonResponse["A4"] = posRead[4];
    jsonResponse["A5"] = posRead[5];
    jsonResponse["T1"] = loadRead[1];
    jsonResponse["T2"] = loadRead[2];
    jsonResponse["T3"] = loadRead[3];
    jsonResponse["T4"] = loadRead[4];
    jsonResponse["T5"] = loadRead[5];
    serializeJson(jsonResponse, response);
    server.send(200, "text/plane", response);
  }
}


void handleConfig(){
  server.send(200, "text/html", setting_html);
}


void handleHelp(){
  server.send(200, "text/html", help_html);
}


void webCtrlServer(){
  server.on("/",       handleRoot);
  server.on("/config", handleConfig);
  server.on("/stop",   handleStop);
  server.on("/help",   handleHelp);

  server.on("/readData",      handleDataFeedBack);
  server.on("/getDevInfo",    handleGetDevInfo);
  server.on("/getAngTorInfo", handleGetAngTorInfo);

  server.on("/cmd", [](){
    int cmdT = server.arg(0).toInt();
    int cmdA = server.arg(1).toInt();
    int cmdB = server.arg(2).toInt();

    commandProcessing(cmdT, cmdA, cmdB);
  });

  server.on("/config/cmd", [](){
    int cmdT = server.arg(0).toInt();
    int cmdA = server.arg(1).toInt();
    int cmdB = server.arg(2).toInt();

    commandProcessing(cmdT, cmdA, cmdB);
  });

  // Start server
  server.begin();
  Serial.println("Server Starts.");
}


void webServerSetup(){
  webCtrlServer();
}


void getMAC(){
  WiFi.mode(WIFI_AP_STA);
  MAC_ADDRESS = WiFi.macAddress();
  Serial.print("MAC:");
  Serial.println(WiFi.macAddress());
}


void getIP(){
  IP_ADDRESS = WiFi.localIP();
}


void setAP(){
  WiFi.softAP(AP_SSID, AP_PWD);
  IPAddress myIP = WiFi.softAPIP();
  IP_ADDRESS = myIP;
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  WIFI_MODE = 1;
}


void setSTA(){
  WIFI_MODE = 3;
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PWD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  IP_ADDRESS = WiFi.localIP();
  Serial.print("STA IP address: ");
  Serial.println(IP_ADDRESS);
}


void getWifiStatus(){
  if(WiFi.status() == WL_CONNECTED){
    WIFI_MODE = 2;
    getIP();
    WIFI_RSSI = WiFi.RSSI();
  }
  else if(WiFi.status() == WL_CONNECTION_LOST && DEFAULT_WIFI_MODE == 2){
    WIFI_MODE = 3;
    // WiFi.disconnect();
    WiFi.reconnect();
  }
}


void wifiInit(){
  DEV_ROLE  = DEFAULT_ROLE;
  WIFI_MODE = DEFAULT_WIFI_MODE;
  if(WIFI_MODE == 1){setAP();}
  else if(WIFI_MODE == 2){setSTA();}
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(espNowMode == 2){
    memcpy(&myData, incomingData, sizeof(myData));

    posESP[0] = myData.S1_pos;
    posESP[1] = myData.S2_pos;
    posESP[2] = myData.S3_pos;
    posESP[3] = myData.S4_pos;
    posESP[4] = myData.S5_pos;

    st.SyncWritePosEx(ID_ESP, 5, posESP, spdESP, accESP);
  }
}


void espNowInit(){
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  MAC_ADDRESS = WiFi.macAddress();
  Serial.print("MAC:");
  Serial.println(WiFi.macAddress());
}


void getDevInfo(){
  jsonDevInfoSend["DevName"]  = DevName;
  jsonDevInfoSend["AP_SSID"]  = AP_SSID;
  jsonDevInfoSend["AP_PWD"]   = AP_PWD;
  jsonDevInfoSend["STA_SSID"] = STA_SSID;
  jsonDevInfoSend["STA_PWD"]  = STA_PWD;
  jsonDevInfoSend["MAC"] = WiFi.macAddress();

  char getDevInfoJson[300];
  serializeJson(jsonDevInfoSend, getDevInfoJson);
  Serial.println(getDevInfoJson);
}


void cmdProcess(){
  if(CtrlModeSelect != CTRL_VIA_SERIAL || espNowMode == 2){
    return;
  }

  switch(processType){
    case   EMERGENCY_STOP:emergencyStop();processType = -1;break;

    case       ANGLE_CTRL:jsonAngleCtrl(false);processType = -1;break;
    case  ANGLE_CTRL_INIT:jsonAngleCtrl(true);processType = -1;break;

    case       COORD_CTRL:jsonCoordCtrl(false);processType = -1;break;
    case  COORD_CTRL_INIT:jsonCoordCtrl(true);processType = -1;break;

    case      ST_POS_CTRL:jsonStPosCtrl(false);processType = -1;break;
    case ST_POS_CTRL_INIT:jsonStPosCtrl(true);processType = -1;break;

    case     GET_DEV_INFO:getDevInfo();processType = -1;break;
    case  GET_ANGTOR_INFO:getAngTorInfo();processType = -1;break;
    case  GET_INFO_BUFFER:getInfoBuffer();processType = -1;break;

    case    RECORD_REPLAY:recordReplay();processType = -1;break;
    case  ROARM_M1_CONFIG:configRoArm();processType = -1;break;
    case             HELP:getHelp();processType = -1;break;

    case               -1:delay(3);
  }
}


void jsonCtrl(String cmdJsonInput){
  deserializeJson(jsonCmdReceive, cmdJsonInput);
  cmdProcess();
}


void serialCtrl(){
  if (Serial.available()){
    DeserializationError err = deserializeJson(jsonCmdReceive, Serial);
    if (err == DeserializationError::Ok){
      CtrlModeSelect = CTRL_VIA_SERIAL;
      cmdHandler();
    }
    else{
      while (Serial.available() > 0){
        Serial.read();
      }
    }
  }
}


void espNowProcess(){
  Serial.println("ESP-NOW MODE:");
  Serial.println(espNowMode);
  if(espNowMode == 1){
    getFeedBackAll();
    myData.S1_pos = posRead[1];
    myData.S2_pos = posRead[2];
    myData.S3_pos = posRead[3];
    myData.S4_pos = posRead[4];
    myData.S5_pos = posRead[5];
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    Serial.println("ESP-NOW Send!");
  }
}



//   // Init ESP-NOW
//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   char str[] = "3F4C5D026611";

  
//   peerInfo.channel = 0;
//   peerInfo.encrypt = false;

//   char * ptr = str; // pointer to start of string
//   while( *ptr ) // while end of string not reached
//   {
//     uint64_t addr = strtoull( ptr, &ptr, 16 );
//     if ( *ptr ) ptr++; // if end of string not reached, increase pointer to skip ','
    
//     // split addr into bytes and store in peerInfo.peer_addr
//     for ( uint8_t i = 0; i < 6; i++ )
//     {
//       peerInfo.peer_addr[ i ] = ( addr >> ( ( 5 - i ) * 8 ) ) & 0xFF;
//     }
    
//     // try add peer
//     if ( esp_now_add_peer( &peerInfo ) != ESP_OK )
//     {
//       Serial.println( "Error adding peer" );
//     }
//   }
