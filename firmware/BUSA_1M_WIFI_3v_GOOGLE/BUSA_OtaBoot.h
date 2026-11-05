#pragma once
// Application self-test/confirmation. Boot selection belongs to the ESP-IDF
// updater/bootloader; never overwrite a separately staged next image here.
static bool busaOtaPendingVerification(){
  const esp_partition_t* running=esp_ota_get_running_partition();
  esp_ota_img_states_t state=ESP_OTA_IMG_UNDEFINED;
  return running&&esp_ota_get_state_partition(running,&state)==ESP_OK&&state==ESP_OTA_IMG_PENDING_VERIFY;
}
static void busaConfirmRunningImage(bool localReady){
  const esp_partition_t* running=esp_ota_get_running_partition();
  const esp_partition_t* boot=esp_ota_get_boot_partition();
  if(!running){busaOtaConfirmResult=ESP_ERR_NOT_FOUND;return;}
  busaOtaRunning=String(running->label);
  busaOtaBoot=boot?String(boot->label):String("unknown");
  esp_ota_img_states_t state=ESP_OTA_IMG_UNDEFINED;
  const esp_err_t rc=esp_ota_get_state_partition(running,&state);
  if(rc==ESP_OK&&state==ESP_OTA_IMG_PENDING_VERIFY){
    busaOtaConfirmResult=localReady?esp_ota_mark_app_valid_cancel_rollback():ESP_ERR_INVALID_STATE;
  }else{busaOtaConfirmResult=rc;}
  Serial0.printf("BUSA OTA running=%s boot=%s verify=0x%X; boot selection unchanged\n",
    busaOtaRunning.c_str(),busaOtaBoot.c_str(),(unsigned)busaOtaConfirmResult);
}
