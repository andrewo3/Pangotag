#include "libs.h"


FIL sdData;
FIL sdLog;

char data_path[256];
char log_path[256];

void log_printf(const char* fmt, ...) {
	char buf[1024];

	va_list args;

	va_start(args,fmt);
	vsprintf(buf,fmt,args);
	va_end(args);

	FRESULT open_success = f_open(&sdLog, log_path, FA_OPEN_APPEND|FA_WRITE);
	f_write(&sdLog, buf, strlen(buf), NULL);
	f_close(&sdLog);

	printf("%s",buf);
}

void initSD() {
    FRESULT mount_success = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
    if (mount_success != FR_OK) {
      printf("Failed to initialize SD Card: err %i\n",mount_success);
      Error_Handler();
    }
}

void initSDFiles() {
  FRESULT data_res;
	int data_num = 0;
	do {
		sprintf(data_path,"test_data%i.bin\0",data_num);
		//printf("Trying %s\r\n",log_path);
		data_res = f_open(&sdData, data_path, FA_CREATE_NEW | FA_WRITE);
		data_num++;

	} while (data_res == FR_EXIST); // continue until file doesnt exist
	if (data_res != FR_OK) {
		log_printf("ERR: Failed to create data file - error code: %i\r\n",data_res);
		Error_Handler();
	}
  f_close(&sdData);

  FRESULT log_res;
	int log_num = 0;
	do {
		sprintf(log_path,"test_log%i.txt\0",log_num);
		//printf("Trying %s\r\n",log_path);
		log_res = f_open(&sdLog, log_path, FA_CREATE_NEW | FA_WRITE);
		log_num++;

	} while (log_res == FR_EXIST); // continue until file doesnt exist
	if (log_res != FR_OK) {
		printf("ERR: Failed to create log - error code: %i\r\n",log_res);
		Error_Handler();
	}
	f_close(&sdLog);
  printf("Writing to %s and %s\n", data_path, log_path);
}

int writePacket(packet p) {
  int res = FR_OK;
  res = f_open(&sdData, data_path, FA_WRITE | FA_OPEN_APPEND);
  if (res != FR_OK) {
    return res;
  }
  res = f_write(&sdData, &p.type, sizeof(packet_type), nullptr);
  if (res != FR_OK) {
    return res;
  }
  res = f_write(&sdData, &p.timestamp, sizeof(uint32_t), nullptr);
  if (res != FR_OK) {
    return res;
  }
  res = f_write(&sdData, &p.len, sizeof(uint32_t), nullptr);
  if (res != FR_OK) {
    return res;
  }
  res = f_write(&sdData, p.payload, p.len * sizeof(uint8_t), nullptr);
  if (res != FR_OK) {
    return res;
  }
  res = f_close(&sdData);
  return res;
}

uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t status = SD_PRESENT;

  if (HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN)== 0x1) //SD switch is inverted
  {
    status = SD_NOT_PRESENT;
  }

  return status;
}