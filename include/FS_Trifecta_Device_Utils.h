#pragma once

#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Interfaces.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define FS_CIRCULAR_BUFFER_SIZE FS_MAX_DATA_LENGTH // == 512

typedef struct
{
    uint8_t buffer[FS_CIRCULAR_BUFFER_SIZE];
    size_t head;  // write position
    size_t tail;  // read position
    size_t count; // number of bytes in buffer
} fs_circular_buffer_t;

void fs_cb_init(fs_circular_buffer_t *cb);
size_t fs_cb_push(fs_circular_buffer_t *cb, const uint8_t *data, size_t len);
size_t fs_cb_pop(fs_circular_buffer_t *cb, uint8_t *out, size_t len);
size_t fs_cb_peek(const fs_circular_buffer_t *cb, uint8_t *out, size_t len);
size_t fs_cb_available(const fs_circular_buffer_t *cb);
void fs_cb_clear(fs_circular_buffer_t *cb);

typedef enum 
{
    FS_SCANNER_IDLE,
    FS_SCANNER_COMMAND,
    FS_SCANNER_PACKET
} fs_scanner_state_t;

// int fs_enqueue_into_command_queue(fs_device_info *device_handle, char *str, size_t len);
int fs_segment_commands(fs_device_info *device_handle, const void *cmd_buf, size_t buf_len);

int fs_base64_encode(const void *data, size_t len, char *output_buffer, size_t buffer_size);
int fs_base64_decode(const char *input, void *output_buffer, size_t buffer_size, size_t *decoded_length);

int obtain_packet_length(int packet_type);
// int fs_enqueue_into_packet_queue(fs_device_info *device_handle, const fs_packet_union *packet, size_t len);
int segment_packets(fs_device_info *device_handle, const void *rx_buf, size_t rx_len);
int base64_to_packet(fs_device_info *device_handle, char *segment, size_t length);

typedef struct {
    int colon_index;     // Position of ':'
    int exclam_index;    // Position of '!'
    int semicolon_index; // Position of ';'
} fs_delimiter_indices_t;

fs_delimiter_indices_t fs_scan_delimiters(const uint8_t *buffer, size_t len);