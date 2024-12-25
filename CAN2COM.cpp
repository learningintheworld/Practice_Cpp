#include <stdint.h>
#include<string.h>

#define ARM_MODULE_ID 0x12010102 
#define ZED_MODULE_ID 0x12020102
#define COM_HEAD 0x55
#define COM_LEN 0x10
#define COM_FRAME_ID 0x00
#define CAN_DATA_LEN 8

typedef struct CanTxMessage {
	uint32_t can_id;
    uint8_t motor_id;      	// Unsigned int8
    int32_t angle;      		// Signed int32
    uint16_t max_speed; 		// Unsigned int16
	uint8_t data[CAN_DATA_LEN];
} CanTxMessage;

typedef struct ComTxMessage {
	uint8_t Head;						// Fixed meter head 0x55
    uint8_t Len;      			// Com length
    uint8_t CANData[4 + CAN_DATA_LEN];    // CAN instruction data(4+8)
    uint8_t FrameID; 				// Fixed to 0
	uint8_t SumCheak;
	uint8_t data[8 + CAN_DATA_LEN];
} ComTxMessage;


uint8_t left_arm_motor_id[7] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
uint8_t right_arm_motor_id[7] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
CanTxMessage can_tx_message;
ComTxMessage com_tx_message;
double joint_angle[19];
double motor_angle[19];
double movmean_motor_angle[19];
uint16_t motor_max_speed = 0;


void encode_can_message(uint8_t motor_id, double angle, uint16_t max_speed, CanTxMessage* can_tx_message)
{
	can_tx_message->can_id = ARM_MODULE_ID;
	
	can_tx_message->motor_id = motor_id;
	memcpy(&can_tx_message->data[0], &can_tx_message->motor_id, sizeof(uint8_t));
	
	can_tx_message->angle = (int32_t)angle * 3600;
//	uart_printf("%d\r\n", can_tx_message->angle);
	memcpy(&can_tx_message->data[1], &can_tx_message->angle, sizeof(int32_t));
//	uart_printf("%d, %d, %d, %d\r\n", can_tx_message->data[1], can_tx_message->data[2], can_tx_message->data[3], can_tx_message->data[4]);
	
	can_tx_message->max_speed = (uint16_t)max_speed * 60;
	memcpy(&can_tx_message->data[5], &can_tx_message->max_speed, sizeof(uint16_t));
	
	memset(&can_tx_message->data[7], 0, sizeof(can_tx_message->data[7]));
}


unsigned char Sum_cheak(unsigned char* data)
{
  unsigned char cheak = 0x00;
  for(int i = 0; i < data[1] - 1; i++)
	{
		cheak += data[i];
	}
	return cheak;
}

void encode_com_message(CanTxMessage* can_tx_message, ComTxMessage* com_tx_message)
{
	com_tx_message->Head = COM_HEAD;
	com_tx_message->Len = COM_LEN;
	
	memcpy(&com_tx_message->CANData[0], &can_tx_message->can_id, sizeof(uint32_t));
	memcpy(&com_tx_message->CANData[4], &can_tx_message->data, CAN_DATA_LEN);
	
	com_tx_message->FrameID = COM_FRAME_ID;
	
	memcpy(&com_tx_message->data[0], &com_tx_message->Head, sizeof(uint8_t));
	memcpy(&com_tx_message->data[1], &com_tx_message->Len, sizeof(uint8_t));
	memcpy(&com_tx_message->data[2], &com_tx_message->CANData, (sizeof(uint32_t)+CAN_DATA_LEN));
	memcpy(&com_tx_message->data[14], &com_tx_message->FrameID, sizeof(uint8_t));
	
	uint8_t checksum = Sum_cheak(com_tx_message->data);
	com_tx_message->SumCheak = checksum;
	memcpy(&com_tx_message->data[15], &com_tx_message->SumCheak, sizeof(uint8_t));
}


// for(int i = 0; i < 7; i++)
// {
//     encode_can_message(right_arm_motor_id[i], movmean_motor_angle[i], motor_max_speed, &can_tx_message);
//     encode_com_message(&can_tx_message, &com_tx_message);
// //     HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
// }

// int main()
// {

// }
