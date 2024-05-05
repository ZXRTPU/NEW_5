#include "struct_typedef.h"
#include "cmsis_os.h"
#include "exchange.h"
#include "miniPC_process.h"
#include "remote_control.h"
#include "VideoTransmitter.h"
// #include "ins_task.h"
// extern INS_t INS;

// ins_data_t ins_data;
static RC_ctrl_t *rc_data;       // 遥控器数据,初始化时返回的指针

void Exchange_task(void const * argument)
{
	while(1){
  rc_data = RemoteControlInit(&huart3);
	osDelay(1);
	}
}