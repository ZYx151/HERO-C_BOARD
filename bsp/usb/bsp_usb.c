#include "bsp_usb.h"

#include "VCOMCOMM.h"
/* 虚拟串口接收数据长度  */
#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 64-8   // byte
#define HEADER_SIZE 4         // byte

// 由于接收数据
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];
// 判断USB连接状态用到的一些变量
bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

__weak void Vision_TimestampReceive(uint8_t *data);
__weak void Vision_ColorReceive(uint8_t *data);
__weak void Vision_DataReceive(uint8_t *data);

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
void VCOMM_CallBack(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len) {
	if (id == 0) {
		switch (fun_code) {
			case VISION_TOMESTAMP_RECEIVE:
					Vision_TimestampReceive(data);
				 break;
			case VISION_COLOR_RECEIVE:
					  Vision_ColorReceive(data);
				break;
			case VISION_DATA_RECEIVE:
					Vision_DataReceive(data);
				break;
			default: 
				 break;
		}
	}
}

void UsbReceiveData(void)
{
	static uint32_t len = USB_RECEIVE_LEN;
	static uint8_t *rx_data_start_address = USB_RX_BUF;  // 接收数据包时存放于缓存区的起始位置
	static uint8_t *rx_data_end_address ; // 接收数据包时存放于缓存区的结束位置
	uint8_t *sof_address = USB_RX_BUF;
	
	// 计算数据包的结束位置
	rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
	// 读取数据
////    USB_Receive(rx_data_start_address, &len);
	
	while (sof_address <= rx_data_end_address) {  // 解析缓冲区数据
		// 寻找帧头位置
		while (*(sof_address) != RECEIVE_SOF && ( sof_address <= rx_data_end_address) ) {
			sof_address ++;
		}
		// 判断是否超出接收数据范围
		if (sof_address >= rx_data_end_address) {
			 break ; // 退出循环
		}
//  检查CRC8
//	  	bool crc8_ok = Verify_CRC8_Check_Sum(sof_address, HEADER_SIZE);
//        if (crc8_ok) {
		    uint8_t  code_id  = *((uint8_t *  )(sof_address + 1));
			uint16_t data_id  = *((uint16_t * )(sof_address + 2));
			uint16_t data_len = *((uint16_t * )(sof_address + 4));
			// 检查整包CRC16校验 4: header size, 2: crc16 size
            uint16_t crc16 = *((uint16_t * )(sof_address + 6 + data_len));
		    if (crc16  == Verify_CRC16_Check_Sum(sof_address + 6, data_len)) {
				if (code_id == 0) {
					switch (data_id) {
						case VISION_TOMESTAMP_RECEIVE:
                                Vision_TimestampReceive(sof_address + 7);
						     break;
					    case VISION_COLOR_RECEIVE:
								  Vision_ColorReceive(sof_address + 7);
						    break;
						case VISION_DATA_RECEIVE:
								Vision_DataReceive(sof_address + 7);
							break;
						default: 
						     break;
					}
					if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
						LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4])) ;
						RECEIVE_TIME = HAL_GetTick();
					}
				}
			}
			sof_address += (data_len + HEADER_SIZE + 2);
	}
	// 更新下一次接收数据的起始位置
	if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
		// 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = USB_RX_BUF;
	} else {
		uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = USB_RX_BUF + remaining_data_len ;
		// 将剩余数据转移到缓冲区的起始位置
		memcpy(USB_RX_BUF, sof_address, remaining_data_len);
	}
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/
/**
 * @brief USB发送数据
 * @param duration 发送周期
 */
void UsbTransmit(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len)
{
	if (len > 64 - 8)
		return;
	uint8_t buff[64] = {0x5a, fun_code};
	*((uint16_t * )(buff + 2)) = id;
	*((uint16_t * )(buff + 4)) = len;
	memcpy(buff + 6, data, len);
	*((uint16_t * )(buff + 6 + len)) = (len == 0) ? 0 : Verify_CRC16_Check_Sum(data, len);
    USB_Transmit((uint8_t *)&buff, sizeof(buff));
}


