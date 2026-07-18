#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <stdbool.h>

// CRSF Protocol Constants
#define CRSF_SYNC_BYTE          0xC8
#define CRSF_MAX_FRAME_SIZE     64
#define CRSF_FRAME_LENGTH_OFFSET 1
#define CRSF_TYPE_OFFSET         2
#define CRSF_PAYLOAD_OFFSET      3

// CRSF Frame Types (Broadcast)
#define CRSF_FRAME_GPS                  0x02
#define CRSF_FRAME_GPS_TIME             0x03
#define CRSF_FRAME_GPS_EXTENDED          0x06
#define CRSF_FRAME_VARIOMEETER           0x07
#define CRSF_FRAME_BATTERY_SENSOR        0x08
#define CRSF_FRAME_BARO_ALTITUDE         0x09
#define CRSF_FRAME_AIRSPEED              0x0A
#define CRSF_FRAME_HEARTBEAT             0x0B
#define CRSF_FRAME_RPM                   0x0C
#define CRSF_FRAME_TEMPERATURE           0x0D
#define CRSF_FRAME_VOLTAGES              0x0E
#define CRSF_FRAME_VTX_TELEMETRY         0x10
#define CRSF_FRAME_BAROMETER             0x11
#define CRSF_FRAME_MAGNETOMETER          0x12
#define CRSF_FRAME_ACCEL_GYRO            0x13
#define CRSF_FRAME_LINK_STATISTICS       0x14
#define CRSF_FRAME_RC_CHANNELS_PACKED    0x16
#define CRSF_FRAME_LINK_STATISTICS_RX    0x1C
#define CRSF_FRAME_LINK_STATISTICS_TX    0x1D
#define CRSF_FRAME_ATTITUDE              0x1E
#define CRSF_FRAME_FLIGHT_MODE           0x21

// CRSF Device Addresses
#define CRSF_ADDRESS_BROADCAST           0x00
#define CRSF_ADDRESS_RC_RECEIVER          0xEC
#define CRSF_ADDRESS_FLIGHT_CONTROLLER   0xC8

// CRSF RC Channels Packed Payload
#define CRSF_RC_CHANNELS_PACKED_SIZE     22

// CRC8 Polynomial: x^8 + x^7 + x^6 + x^4 + x^2 + 1 (0xD5)
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);

// Frame Structure
typedef struct {
    uint8_t sync_byte;
    uint8_t frame_length;
    uint8_t frame_type;
    uint8_t payload[CRSF_MAX_FRAME_SIZE - 4]; // Max payload size
    uint8_t crc;
} crsf_frame_t;

// RC Channels Packed Payload (16 channels, 11 bits each)
typedef struct __attribute__((packed)) {
    unsigned int channel_01 : 11;
    unsigned int channel_02 : 11;
    unsigned int channel_03 : 11;
    unsigned int channel_04 : 11;
    unsigned int channel_05 : 11;
    unsigned int channel_06 : 11;
    unsigned int channel_07 : 11;
    unsigned int channel_08 : 11;
    unsigned int channel_09 : 11;
    unsigned int channel_10 : 11;
    unsigned int channel_11 : 11;
    unsigned int channel_12 : 11;
    unsigned int channel_13 : 11;
    unsigned int channel_14 : 11;
    unsigned int channel_15 : 11;
    unsigned int channel_16 : 11;
} crsf_rc_channels_packed_t;

// Function Prototypes
bool crsf_parse_frame(const uint8_t *buffer, uint8_t buffer_size, crsf_frame_t *frame);
bool crsf_extract_rc_channels(const crsf_frame_t *frame, crsf_rc_channels_packed_t *channels);
int16_t crsf_convert_channel_to_us(uint16_t channel_value);

#endif // CRSF_H