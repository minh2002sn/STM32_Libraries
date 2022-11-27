#ifndef MIN_H
#define MIN_H

#include <stdint.h>
#include <stdbool.h>

#define TRANSPORT_FIFO_MAX_FRAMES           (1U << (4U))    // 2^4
#define MAX_PAYLOAD                         (255U)

typedef struct{
    uint32_t crc;
}  CRC32_HandleTypeDef;

typedef struct{
    uint8_t Rx_Payload_Buffer[MAX_PAYLOAD];
    uint32_t Rx_Frame_Checksum;
    CRC32_HandleTypeDef Rx_Checksum;
    CRC32_HandleTypeDef TX_Checksum;
    uint8_t Rx_State;
    uint8_t Rx_Control_ID;
    uint8_t Rx_Receiving_Index;
    uint8_t Rx_Sequence_Number;
    uint8_t Rx_Payload_Length;
    uint8_t Rx_Header_Byte_Counter;
    uint8_t Tx_Header_Byte_Counter;
    uint8_t port;
} MIN_HandleTypeDef;

void MIN_Init(MIN_HandleTypeDef *p_min, uint8_t p_port);
void MIN_Handle(MIN_HandleTypeDef *p_min, uint8_t const *p_buffer, uint32_t p_buffer_length);
void MIN_Send_Data(MIN_HandleTypeDef *p_min, uint8_t p_control_id, uint8_t const *p_payload, uint32_t p_payload_length);

// CALLBACK. Must return current time in milliseconds.
// Typically a tick timer interrupt will increment a 32-bit variable every 1ms (e.g. SysTick on Cortex M ARM devices).
uint32_t MIN_Get_Tick_ms();
// CALLBACK. Must return current buffer space in the given port. Used to check that a frame can be
uint16_t MIN_Transmit_Space(uint8_t p_port);

// CALLBACK. Send a byte on the given line.
void MIN_Transmit_Byte(uint8_t p_port, uint8_t p_byte);

// CALLBACK. Called when frame transmission is finished; useful for buffering bytes into a single serial call.
void MIN_Start_Transmit(uint8_t p_port);
void MIN_Finish_Transmit(uint8_t p_port);

// CALLBACK. Handle incoming MIN frame
void MIN_Application_Handle(uint8_t p_control_id, uint8_t const *p_payload, uint32_t p_payload_length, uint8_t p_port);

#endif
