#ifndef SD_INTF_H
#define SD_INTF_H

#include <stdint.h>
#include "fatfs.h"

#define f_unmount(path) f_mount(0, path, 0)


extern FIL sdData;
extern FIL sdLog;

extern char data_path[256];
extern char log_path[256];

void log_printf(const char* fmt, ...);

typedef enum {
    PACKET_GPS, PACKET_TMP, PACKET_WAKE, PACKET_BMIC, PACKET_IMU
} packet_type;

typedef struct {
    packet_type type;
    uint32_t timestamp;
    uint32_t rtcTimestamp;
    uint32_t len;
    uint8_t* payload;
} packet;

void initSD();

void initSDFiles();

int writePacket(packet p);



#endif