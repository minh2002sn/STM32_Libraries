#include "min.h"

// Number of bytes needed for a frame with a given payload length, excluding stuff bytes
// 3 header bytes, ID/control byte, length byte, seq byte, 4 byte CRC, EOF byte
#define TRANSMIT_SIZE(p_data_size)          ((p_data_size) + 11)

enum{
    SEARCHING_FOR_START_OF_FRAME,
    RECEIVING_ID_CONTROL,
    RECEIVING_SEQUENCE_NUMBER,
    RECEIVING_LENGTH,
    RECEIVING_PAYLOAD,
    RECEIVING_CHECKSUM_3,
    RECEIVING_CHECKSUM_2,
    RECEIVING_CHECKSUM_1,
    RECEIVING_CHECKSUM_0,
    RECEIVING_END_OF_FRAME,
};

enum{
    HEADER_BYTE         =   0XAAU,
    STUFF_BYTE          =   0X55U,
    END_OF_FRAME_BYTE   =   0X55U,
};

enum{
    ACK     =   0xFF,
    RESET   =   0xFE,
};

static uint32_t Current_Time;

static void CRC32_Init(CRC32_HandleTypeDef *p_crc){
    p_crc->crc = 0xFFFFFFFFU;
}

static void CRC32_Step(CRC32_HandleTypeDef *p_crc, uint8_t p_byte){
    p_crc->crc ^= p_byte;
    for(int i = 0; i < 8; i++){
        uint32_t t_mask = (uint32_t) ~(p_crc->crc & 1U);
        p_crc->crc = (p_crc->crc >> 1) ^ (0xEDB88320U & t_mask);
    }
}

static uint32_t CRC32_Finalize(CRC32_HandleTypeDef *p_crc){
    return ~(p_crc->crc);
}

static void Stuffed_Transmit_Byte(MIN_HandleTypeDef *p_min, uint8_t p_byte, bool p_is_using_crc){
    MIN_Transmit_Byte(p_min->port, p_byte);

    if(p_is_using_crc){
        CRC32_Step(&(p_min->TX_Checksum), p_byte);
    }

    if(p_byte == HEADER_BYTE){
        if(--p_min->Tx_Header_Byte_Counter == 0){
            MIN_Transmit_Byte(p_min->port, STUFF_BYTE);
            p_min->Tx_Header_Byte_Counter = 2;
        }
    } else{
        p_min->Tx_Header_Byte_Counter = 2;
    }

}

static void Transmit_Data(MIN_HandleTypeDef *p_min, uint8_t p_control_id, uint8_t p_sequence_number, uint8_t const *p_payload, 
                    uint16_t p_payload_offset, uint16_t p_payload_mask, uint32_t p_payload_length){
    uint32_t checksum;
    
    p_min->Tx_Header_Byte_Counter = 2;
    CRC32_Init(&(p_min->TX_Checksum));

    MIN_Start_Transmit(p_min->port);

    MIN_Transmit_Byte(p_min->port, HEADER_BYTE);
    MIN_Transmit_Byte(p_min->port, HEADER_BYTE);
    MIN_Transmit_Byte(p_min->port, HEADER_BYTE);

    Stuffed_Transmit_Byte(p_min, p_control_id, true);

    if(p_control_id & 0x80){
        Stuffed_Transmit_Byte(p_min, p_sequence_number, true);
    }

    Stuffed_Transmit_Byte(p_min, p_payload_length, true);

    for(int n = p_payload_length; n > 0; n--){
        Stuffed_Transmit_Byte(p_min, p_payload[p_payload_offset], true);
        p_payload_offset++;
        p_payload_offset &= p_payload_mask;
    }

    checksum = CRC32_Finalize(&(p_min->TX_Checksum));

    Stuffed_Transmit_Byte(p_min, (uint8_t)((checksum >> 24) & 0xFF), false);
    Stuffed_Transmit_Byte(p_min, (uint8_t)((checksum >> 16) & 0xFF), false);
    Stuffed_Transmit_Byte(p_min, (uint8_t)((checksum >> 8) & 0xFF), false);
    Stuffed_Transmit_Byte(p_min, (uint8_t)((checksum >> 0) & 0xFF), false);

    MIN_Transmit_Byte(p_min->port, END_OF_FRAME_BYTE);

    MIN_Finish_Transmit(p_min->port);
}

static void Receive_Data(MIN_HandleTypeDef *p_min, uint8_t p_byte){
    uint32_t t_crc;

    if(p_min->Rx_Header_Byte_Counter == 2){
        p_min->Rx_Header_Byte_Counter = 0;
        if(p_byte == HEADER_BYTE){
            p_min->Rx_State = RECEIVING_ID_CONTROL;
            return;
        } else if(p_byte == STUFF_BYTE){
            return;
        } else{
            p_min->Rx_State = SEARCHING_FOR_START_OF_FRAME;
            return;
        }
    }

    if(p_byte == HEADER_BYTE){
        p_min->Rx_Header_Byte_Counter++;
    } else{
        p_min->Rx_Header_Byte_Counter = 0;
    }

    switch(p_min->Rx_State){
        case SEARCHING_FOR_START_OF_FRAME:
            break;
        case RECEIVING_ID_CONTROL:
            p_min->Rx_Control_ID = p_byte;
            p_min->Rx_Receiving_Index = 0;
            CRC32_Init(&(p_min->Rx_Checksum));
            CRC32_Step(&(p_min->Rx_Checksum), p_byte);
            if(p_byte == 0x80){
                p_min->Rx_State = RECEIVING_SEQUENCE_NUMBER;
            } else{
                p_min->Rx_Sequence_Number = 0;
                p_min->Rx_State = RECEIVING_LENGTH;
            }
            break;
        case RECEIVING_SEQUENCE_NUMBER:
            p_min->Rx_Sequence_Number = p_byte;
            CRC32_Step(&(p_min->Rx_Checksum), p_byte);
            p_min->Rx_State = RECEIVING_LENGTH;
            break;
        case RECEIVING_LENGTH:
            p_min->Rx_Payload_Length = p_byte;
            p_min->Rx_Receiving_Index = 0;
            CRC32_Step(&(p_min->Rx_Checksum), p_byte);
            if(p_min->Rx_Payload_Length > 0){
                if(p_min->Rx_Payload_Length < MAX_PAYLOAD){
                    p_min->Rx_State = RECEIVING_PAYLOAD;
                } else{
                    p_min->Rx_State = SEARCHING_FOR_START_OF_FRAME;
                }
            } else{
                p_min->Rx_State = RECEIVING_CHECKSUM_3;
            }
            break;
        case RECEIVING_PAYLOAD:
            p_min->Rx_Payload_Buffer[p_min->Rx_Receiving_Index++] = p_byte;
            CRC32_Step(&(p_min->Rx_Checksum), p_byte);
            if(p_min->Rx_Receiving_Index == p_min->Rx_Payload_Length){
                p_min->Rx_State = RECEIVING_CHECKSUM_3;
            }
            break;
        case RECEIVING_CHECKSUM_3:
            p_min->Rx_Frame_Checksum = ((uint32_t)p_byte) << 24;
            p_min->Rx_State = RECEIVING_CHECKSUM_2;
            break;
        case RECEIVING_CHECKSUM_2:
            p_min->Rx_Frame_Checksum |= ((uint32_t)p_byte) << 16;
            p_min->Rx_State = RECEIVING_CHECKSUM_1;
            break;
        case RECEIVING_CHECKSUM_1:
            p_min->Rx_Frame_Checksum |= ((uint32_t)p_byte) << 8;
            p_min->Rx_State = RECEIVING_CHECKSUM_0;
            break;
        case RECEIVING_CHECKSUM_0:
            p_min->Rx_Frame_Checksum |= p_byte;
            t_crc = CRC32_Finalize(&(p_min->Rx_Checksum));
            if(p_min->Rx_Frame_Checksum == t_crc){
                p_min->Rx_State = RECEIVING_END_OF_FRAME;
            } else{
                p_min->Rx_State = SEARCHING_FOR_START_OF_FRAME;
            }
            break;
        case RECEIVING_END_OF_FRAME:
            if(p_byte == END_OF_FRAME_BYTE){
                MIN_Application_Handle(p_min->Rx_Control_ID, p_min->Rx_Payload_Buffer, p_min->Rx_Payload_Length, p_min->port);
            } else{
                // Error handle
            }
            p_min->Rx_State = SEARCHING_FOR_START_OF_FRAME;
            break;
        default:
            p_min->Rx_State = SEARCHING_FOR_START_OF_FRAME;
            break;
    }

}

void MIN_Handle(MIN_HandleTypeDef *p_min, uint8_t const *p_buffer, uint32_t p_buffer_length){
    for(int i; i < p_buffer_length; i++){
        Receive_Data(p_min, p_buffer[i]);
    }
}

void MIN_Send_Data(MIN_HandleTypeDef *p_min, uint8_t p_control_id, uint8_t const *p_payload, uint32_t p_payload_length){
    if(TRANSMIT_SIZE(p_payload_length) <= MIN_Transmit_Space(p_min->port)){
        Transmit_Data(p_min, p_control_id & (uint8_t)0x3F, 0, p_payload, 0, 0xFFFFU, p_payload_length);
    }
}
