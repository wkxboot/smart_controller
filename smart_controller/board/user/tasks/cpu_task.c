#include "board.h"
#include "cmsis_os.h"
#include "cpu_utils.h"
#include "cpu_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[cpu_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


osThreadId   cpu_task_hdl;



void cpu_task(void const * argument)
{
 while(1){
 log_one_line("cpu:%d%%.",osGetCPUUsage());
 bsp_sys_led_toggle();
 osDelay(400);   
 } 
}
   
  
 