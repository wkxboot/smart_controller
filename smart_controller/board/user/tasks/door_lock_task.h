#ifndef  __DOOR_LOCK_TASK_H__
#define  __DOOR_LOCK_TASK_H__



extern osThreadId   door_lock_task_hdl;
extern osMessageQId door_lock_task_msg_q_id;
void door_lock_task(void const * argument);


#define  DOOR_LOCK_TASK_MSG_WAIT_TIMEOUT_VALUE  osWaitForever













#endif