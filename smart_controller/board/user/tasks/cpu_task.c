#include "board.h"
#include "cmsis_os.h"
#include "cpu_utils.h"
#include "cpu_task.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[cpu_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


osThreadId   cpu_task_hdl;

static   task_msg_t scale_msg;
#define  SET_SENSOR_CMD_STR    "set sensor 1234"

void cpu_task(void const * argument)
{
 uint8_t cmd[20];
 uint8_t read_cnt;
 uint8_t has_data;
 
  /*等待任务同步*/
 xEventGroupSync(tasks_sync_evt_group_hdl,TASKS_SYNC_EVENT_CPU_TASK_RDY,TASKS_SYNC_EVENT_ALL_TASKS_RDY,osWaitForever);
 log_debug("cpu task sync ok.\r\n");
 
 while(1){
 log_one_line("cpu:%d%%.",osGetCPUUsage());
 bsp_sys_led_toggle();
 osDelay(250); 
 
 has_data = SEGGER_RTT_HasData(0);
 if(has_data > 0){
 read_cnt = SEGGER_RTT_Read(0,cmd,10);
 if(read_cnt == strlen(SET_SENSOR_CMD_STR) + 2){  
 if(strncmp((const char*)cmd,SET_SENSOR_CMD_STR,strlen(SET_SENSOR_CMD_STR)) == 0){
 scale_msg.type = REQ_SET_SENSOR;
 osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,5);  
 }  
 }
 } 
 
 
}
}  
  
 