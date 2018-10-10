#include "board.h"
#include "cmsis_os.h"
#include "serial.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "door_lock_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[door_lock]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


osThreadId   door_lock_task_hdl;
osMessageQId door_lock_task_msg_q_id;

void door_lock_task(void const * argument)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;

 osMessageQDef(door_lock_task_msg_q,2,uint32_t);
 door_lock_task_msg_q_id = osMessageCreate(osMessageQ(door_lock_task_msg_q),door_lock_task_hdl);
 log_assert(door_lock_task_msg_q_id); 
 
 /*等待任务同步*/
 xEventGroupSync(tasks_sync_evt_group_hdl,TASKS_SYNC_EVENT_DOOR_LOCK_TASK_RDY,TASKS_SYNC_EVENT_ALL_TASKS_RDY,osWaitForever);
 log_debug("door lock task sync ok.\r\n");
 
 while(1){
 os_msg = osMessageGet(door_lock_task_msg_q_id,DOOR_LOCK_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 
 if(msg->type == REQ_DOOR_STATUS){     
 }
 
 if(msg->type == REQ_LOCK_STATUS){     
 }
 
 if(msg->type == REQ_UNLOCK){     
 }
 
 if(msg->type == REQ_LOCK){     
 }
   
 }
 
 }
}