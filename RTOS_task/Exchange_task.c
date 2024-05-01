#include "struct_typedef.h"
#include "Exchange_task.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "drv_can.h"
#include <string.h>
#include "miniPC_process.h"

extern INS_t INS;

static Vision_Recv_s *vision_recv_data;

ins_data_t ins_data;

void Exchange_task(void const * argument)
{
	Vision_Init_Config_s config = {
		.recv_config = {
			.header = VISION_RECV_HEADER,
		},
		.send_config = {
			.header = VISION_SEND_HEADER,
			.detect_color = VISION_DETECT_COLOR_BLUE,
			.reset_tracker = VISION_RESET_TRACKER_NO,
			.is_shoot = VISION_SHOOTING,
			.tail = VISION_SEND_TAIL,
		},
		.usart_config = {
			.recv_buff_size = VISION_RECV_SIZE,
			.usart_handle = &huart6,
		},
	};
	vision_recv_data = VisionInit(&config);

	while (1)
	{
		ins_data.angle[0] = INS.Yaw;
		ins_data.angle[1] = INS.Roll;
		ins_data.angle[2] = INS.Pitch;

		osDelay(1);
	}
}