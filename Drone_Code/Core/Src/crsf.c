#include "crsf.h"
#include <stdint.h>
#include <stdbool.h>

// CRC8 Polynomial: x^8 + x^7 + x^6 + x^4 + x^2 + 1 (0xD5)
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);

// Function Prototypes
bool crsf_parse_frame(const uint8_t *buffer, uint8_t buffer_size, crsf_frame_t *frame);
bool crsf_extract_rc_channels(const crsf_frame_t *frame, crsf_rc_channels_packed_t *channels);
int16_t crsf_convert_channel_to_us(uint16_t channel_value);

// crsf.c
#include "crsf.h"

// CRC8 Lookup Table (Polynomial: 0xD5)
static const uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

// Calculate CRC8 for CRSF frame
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}

// Parse CRSF frame from buffer
bool crsf_parse_frame(const uint8_t *buffer, uint8_t buffer_size, crsf_frame_t *frame) {
    if (buffer_size < 4) return false; // Minimum frame size
    
    // Check sync byte
    if (buffer[0] != CRSF_SYNC_BYTE) return false;
    
    uint8_t frame_length = buffer[CRSF_FRAME_LENGTH_OFFSET];
    if (frame_length < 2 || frame_length > CRSF_MAX_FRAME_SIZE - 2) return false;
    
    // Check buffer size
    if (buffer_size < frame_length + 2) return false;
    
    // Calculate CRC (excluding the CRC byte itself, which is 1 byte)
    uint8_t crc = crsf_crc8(&buffer[CRSF_TYPE_OFFSET], frame_length - 1);

    // Verify against the CRC byte located at the end of the frame
    if (crc != buffer[frame_length + 1]) return false;

    // Calculate CRC (excluding sync byte and frame length)
    // uint8_t crc = crsf_crc8(&buffer[CRSF_TYPE_OFFSET], frame_length);
    // if (crc != buffer[frame_length + 1]) return false;
    
    // Copy frame data
    frame->sync_byte = buffer[0];
    frame->frame_length = frame_length;
    frame->frame_type = buffer[CRSF_TYPE_OFFSET];

    // Robust check: Ensure the payload length doesn't meet or exceed maximum allocation
    if ((frame_length - 1) >= (CRSF_MAX_FRAME_SIZE - 4)) {
        return false;
    }

    // Safely copy payload
    for (uint8_t i = 0; i < frame_length - 1; i++) {
        frame->payload[i] = buffer[CRSF_PAYLOAD_OFFSET + i];
    }

    frame->crc = crc;
    
    return true;
}

// Extract RC channels from CRSF frame
bool crsf_extract_rc_channels(const crsf_frame_t *frame, crsf_rc_channels_packed_t *channels) {
    if (frame->frame_type != CRSF_FRAME_RC_CHANNELS_PACKED) return false;
    if (frame->frame_length != 24) return false;
    
    const uint8_t *p = frame->payload;
    
    // Explicit bitfield decoding exactly as outlined in the TBS spec
    channels->channel_01 = (p[0]       | (p[1]  << 8)) & 0x07FF;
    channels->channel_02 = ((p[1] >> 3)  | (p[2]  << 5)) & 0x07FF;
    channels->channel_03 = ((p[2] >> 6)  | (p[3]  << 2) | (p[4] << 10)) & 0x07FF;
    channels->channel_04 = ((p[4] >> 1)  | (p[5]  << 7)) & 0x07FF;
    channels->channel_05 = ((p[5] >> 4)  | (p[6]  << 4)) & 0x07FF;
    channels->channel_06 = ((p[6] >> 7)  | (p[7]  << 1) | (p[8] << 9)) & 0x07FF;
    channels->channel_07 = ((p[8] >> 2)  | (p[9]  << 6)) & 0x07FF;
    channels->channel_08 = ((p[9] >> 5)  | (p[10] << 3)) & 0x07FF;
    channels->channel_09 = (p[11]      | (p[12] << 8)) & 0x07FF;
    channels->channel_10 = ((p[12] >> 3) | (p[13] << 5)) & 0x07FF;
    channels->channel_11 = ((p[13] >> 6) | (p[14] << 2) | (p[15] << 10)) & 0x07FF;
    channels->channel_12 = ((p[14] >> 1) | (p[15] << 7)) & 0x07FF;
    channels->channel_13 = ((p[15] >> 4) | (p[16] << 4)) & 0x07FF;
    channels->channel_14 = ((p[16] >> 7) | (p[17] << 1) | (p[18] << 9)) & 0x07FF;
    channels->channel_15 = ((p[18] >> 2) | (p[19] << 6)) & 0x07FF;
    channels->channel_16 = ((p[19] >> 5) | (p[20] << 3)) & 0x07FF;
    
    return true;
}

// Convert CRSF channel value (11-bit) to microseconds (1000-2000 range)
int16_t crsf_convert_channel_to_us(uint16_t channel_value) {
    // CRSF channel value is 11-bit (0-2047)
    // CRSF center = 992, 1500us = center
    // Formula: us = (channel_value - 992) * 5 / 8 + 1500
    return (int16_t)((channel_value - 992) * 5 / 8) + 1500;
}