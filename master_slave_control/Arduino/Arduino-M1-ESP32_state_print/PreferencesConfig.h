#include <Preferences.h>
#include <nvs_flash.h>

Preferences preferences;

// void preferencesSetup(){
  // preferences.begin("RoArm-M1", false);
// }

//void getTasksLength(){
//  
//}

void getFreeNvsSpace(){
  // Serial.println(preferences.freeEntries());
  // int nvsSpace = preferences.freeEntries();
  Serial.println(preferences.freeEntries());
  // Serial.println(preferences.freeEntries)
}


void nvsClear(){
  Serial.println("P1: NVS_CLEAR");
  nvs_flash_erase(); // erase the NVS partition and...
  nvs_flash_init(); // initialize the NVS partition.
}