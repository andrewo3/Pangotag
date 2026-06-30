#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#define MAX_FIFO_FRAMES 128

#include "bmi270/bmi270.h"

extern uint32_t startTime;

extern struct bmi2_dev bmi270;
extern int16_t bmi2_data[MAX_FIFO_FRAMES * 3 * 2];
extern uint32_t payload_size;
extern uint8_t fifo_buffer[1024];
extern struct bmi2_fifo_frame fifo;

extern uint8_t saved_rmc[128];
extern uint8_t nmea_recv[128];
extern volatile uint8_t gps_char;
extern volatile uint16_t gps_idx;
extern volatile uint8_t rmc_ready;

extern uint32_t totalEnergyUsed;
extern uint16_t lastAcc;

int initBMI270();

void initTMP();

void readTMP();

void readGPS();
void GPS_Start();

void initBMIC();
void readBMIC();

void readBMI270();

void setupAccSleep();
void setupAccWake();

#endif