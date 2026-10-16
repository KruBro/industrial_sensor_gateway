#ifndef ALARM_H
#define ALARM_H

#include <stdint.h>
#include <assert.h>

typedef struct {
    char     tag[16];     // Offset 0..15  (16 bytes) - e.g., "TEMP", "PRESSURE"
    uint16_t value;       // Offset 16..17 (2 bytes)  - Raw scaled value (e.g., 1205 = 120.5 C)
    uint16_t limit;       // Offset 18..19 (2 bytes)  - Threshold in same scale (e.g., 1200)
    uint8_t  severity;    // Offset 20     (1 byte)   - 1 (WARN), 2 (HIGH), 3 (CRIT)
    uint8_t  _pad[3];     // Offset 21..23 (3 bytes)  - Explicit padding for 4-byte alignment
    uint32_t timestamp;   // Offset 24..27 (4 bytes)  - Epoch seconds (uint32_t)
} AlarmEvent;

typedef struct {
    const char *tag;        // Static Config: Name to put in AlarmEvent.tag (e.g., "TEMP")
    uint16_t    limit;      // Static Config: Threshold (e.g., 1200 for 120.0 C)
    uint16_t    hysteresis; // Static Config: Margin to re-arm (e.g., 20 for 2.0 C)
    int         in_alarm;   // Dynamic State: 0 = Normal (Armed), 1 = Currently in Alarm!
} AlarmChannel;

#define NUM_CHANNELS 3

static_assert(sizeof(AlarmEvent) == 28, "AlarmEvent must be exactly 28 bytes");

int alarm_init(const char *log_path);
void alarm_shutdown(void);
void alarm_check(const uint16_t *regs);

#endif