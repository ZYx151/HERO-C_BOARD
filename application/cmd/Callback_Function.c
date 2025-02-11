//#include "Callback_Function.h"
//#include "bsp_can.h"
//#include "bsp_usart.h"
//#include "CANDrive.h"


//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	static uint32_t Can1_ID;
//	uint8_t Can1_Receivebuff[8];
//    Can1_ID = CAN_Receive_DataFrame(hcan, Can1_Receivebuff);
//    switch (Can1_ID) {
//		case 0x201 : DJIMotorReceive(loader, Can1_Receivebuff);
//			break;
//		case 0x202 : DJIMotorReceive(friction_l, Can1_Receivebuff);
//			break;
//		case 0x203 : DJIMotorReceive(friction_r, Can1_Receivebuff);
//			break;
//		case 0x204 : DJIMotorReceive(friction_u, Can1_Receivebuff);
//			break;
//		default : break;
//	}
//}

///**
// * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
// *
// * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
// */
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	static uint32_t Can2_ID;
//	uint8_t Can2_Receivebuff[8];
//    Can2_ID = CAN_Receive_DataFrame(hcan, Can2_Receivebuff);
//	switch (Can2_ID) {
//		case 0x205 : DJIMotorReceive(yaw_motor, Can2_Receivebuff);
//			break;
//		case 0x301 : DMMotorReceive(pitch_motor, Can2_Receivebuff);
//			break;
//		default : break;
//	}
//}






