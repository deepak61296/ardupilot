#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_COMPANION_HEALTH \
    LOG_CCH_MSG

// @LoggerMessage: CCH
// @Description: Companion Computer Health status
// @Field: TimeUS: Time since system startup
// @Field: State: Current failsafe state (0=Disconnected, 1=Healthy, 2=Degraded, 3=Critical)
// @Field: Svc: Services status bitmask
// @Field: Seq: Watchdog sequence number
// @Field: Temp: Board temperature
// @Field: Cpu: CPU load
// @Field: Mem: Memory usage
// @Field: Dsk: Disk usage
// @Field: Gpu: GPU load (255 if N/A)
// @Field: Flg: Status flags (0=Throttled, 1=Overheating, 2=Low Mem, 3=Low Disk)
struct PACKED log_CCH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t state;
    uint32_t services_status;
    uint16_t watchdog_seq;
    int16_t temperature;
    uint8_t cpu_load;
    uint8_t memory_used;
    uint8_t disk_used;
    uint8_t gpu_load;
    uint8_t status_flags;
};

#define LOG_STRUCTURE_FROM_COMPANION_HEALTH \
    { LOG_CCH_MSG, sizeof(log_CCH), \
        "CCH", "QBIHhBBBBB", "TimeUS,State,Svc,Seq,Temp,Cpu,Mem,Dsk,Gpu,Flg", "s--#O%%%%-", "F--0A0000-" },
