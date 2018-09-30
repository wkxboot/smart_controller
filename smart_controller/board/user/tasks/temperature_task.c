#include "cmsis_os.h"
#include "tasks_init.h"
#include "adc_task.h"
#include "alarm_task.h"
#include "compressor_task.h"
#include "temperature_task.h"
#include "display_task.h"
#include "log.h"
#define  LOG_MODULE_LEVEL    LOG_LEVEL_DEBUG
#define  LOG_MODULE_NAME     "[t_task]"

osThreadId   temperature_task_hdl;
osMessageQId temperature_task_msg_q_id;

static task_msg_t *ptr_msg;
static task_msg_t  a_msg;
static task_msg_t  c_msg;
static task_msg_t  d_msg;
static task_msg_t  response_msg;

/*温度传感器型号：LAT5061G3839G B值：3839K*/
static int16_t const t_r_map[][2]={
  {-12,12224},{-11,11577},{-10,10968},{-9,10394},{-8,9854},{-7,9344},{-6,8864},{-5,8410},{-4,7983},{-3 ,7579},
  {-2 ,7199} ,{-1,6839}  ,{0,6499}   ,{1,6178 } ,{2,5875 },{3,5588 },{4,5317} ,{5,5060} ,{6,4817} ,{7,4587}  ,
  {8,4370}   ,{9,4164}   ,{10,3969}  ,{11,3784} ,{12,3608},{13,3442},{14,3284},{15,3135},{16,2993},{17,2858} ,
  {18,2730}  ,{19,2609}  ,{20,2494}  ,{21,2384} ,{22,2280},{23,2181},{24,2087},{25,1997},{26,1912},{27,1831} ,
  {28,1754}  ,{29,1680}  ,{30,1610}  ,{31,1543} ,{32,1480},{33,1419},{34,1361},{35,1306},{36,1254},{37,1204} ,
  {38,1156}  ,{39,1110}  ,{40,1067}  ,{41,1025} ,{42,985} ,{43,947} ,{44,911} ,{45,876} ,{46,843} ,{47,811}  ,
  {48,781}   ,{49,752}   ,{50,724}   ,{51,697}  ,{52,672} ,{53,647} ,{54,624} ,{55,602} ,{56,580} ,{57,559}
};


#define  TR_MAP_IDX_OVER_HIGH_ERR          0xff
#define  TR_MAP_IDX_OVER_LOW_ERR           0xfe


typedef enum
{
T_DIR_INIT=0,
T_DIR_UP,
T_DIR_DOWN
}temperature_dir_t;

typedef struct
{
int16_t           value;
temperature_dir_t dir;
uint32_t          time;
uint8_t           changed;
}temperature_t;



static temperature_t   temperature={
.value = 0,
.dir   = T_DIR_INIT,
.time =0,
.changed=0
};
               


static uint8_t seek_idex(uint32_t r)
{
 uint8_t mid=0;
 int low = TR_MAP_IDX_MIN;  
 int high =TR_MAP_IDX_MAX;  
 
 if(r < t_r_map[TR_MAP_IDX_MAX][1]){
 log_error("NTC 阻值超过最高温度范围！r=%d\r\n",r); 
 return TR_MAP_IDX_OVER_HIGH_ERR;
 }else if(r > t_r_map[TR_MAP_IDX_MIN][1]){
 log_error("NTC 阻值超过最低温度范围！r=%d\r\n",r); 
 return TR_MAP_IDX_OVER_LOW_ERR;
 }
 
  while (low <= high) {  
  mid = (low + high) / 2;  
  if(r > t_r_map[mid][1]){
    if(r <= t_r_map[mid-1][1]){
    return mid - 1;
    } else{
    high = mid - 1;  
    }
  } else {
  if(r > t_r_map[mid+1][1]){
   return mid;
  } else{
  low = mid + 1;   
  }
  }  
 }
 return 0; 
}




static uint32_t get_r(uint16_t adc)
{
 float t_sensor_r;
 t_sensor_r = (TEMPERATURE_SENSOR_SUPPLY_VOLTAGE*TEMPERATURE_SENSOR_ADC_VALUE_MAX*TEMPERATURE_SENSOR_BYPASS_RES_VALUE)/(adc*TEMPERATURE_SENSOR_REFERENCE_VOLTAGE)-TEMPERATURE_SENSOR_BYPASS_RES_VALUE;
 return (uint32_t)t_sensor_r;
}

int16_t get_t(uint16_t adc)
{
  uint8_t idx;
  uint32_t r; 
  if(adc == ADC_TASK_ADC_ERR_VALUE){
  return TEMPERATURE_ERR_VALUE_SENSOR;
  }
  r=get_r(adc);
  idx = seek_idex(r);
  if(idx == TR_MAP_IDX_OVER_HIGH_ERR){
  return TEMPERATURE_ERR_VALUE_OVER_HIGH;
  }else if(idx == TR_MAP_IDX_OVER_LOW_ERR){
  return TEMPERATURE_ERR_VALUE_OVER_LOW;
  }
  /*返回带有温度补偿值的温度*/
  return t_r_map[idx][0] + TEMPERATURE_COMPENSATION_VALUE;
}

void temperature_task(void const *argument)
{
  uint16_t bypass_r_adc;
  uint32_t cur_time;
  uint32_t delta_time;
  int16_t  t;
  osEvent  os_msg;
  osStatus status;
  osMessageQDef(temperature_msg_q,6,uint32_t);
  temperature_task_msg_q_id = osMessageCreate(osMessageQ(temperature_msg_q),temperature_task_hdl);
  log_assert(temperature_task_msg_q_id);
  
  /*等待任务同步*/
  xEventGroupSync(tasks_sync_evt_group_hdl,TASKS_SYNC_EVENT_TEMPERATURE_TASK_RDY,TASKS_SYNC_EVENT_ALL_TASKS_RDY,osWaitForever);
  log_debug("temperature task sync ok.\r\n");
  
  while(1){
  os_msg = osMessageGet(temperature_task_msg_q_id,TEMPERATURE_TASK_MSG_WAIT_TIMEOUT);
  if(os_msg.status == osEventMessage){
  ptr_msg = (task_msg_t *)os_msg.value.v;
  cur_time = osKernelSysTick(); 
  delta_time = cur_time-temperature.time;
  
  /*温度ADC转换完成消息处理*/
  if(ptr_msg->type == T_ADC_COMPLETED){

   bypass_r_adc = ptr_msg->adc;
   t =get_t(bypass_r_adc);
   /*温度值有变化*/
   if(t !=temperature.value){
   if(t == TEMPERATURE_ERR_VALUE_SENSOR    ||\
      t == TEMPERATURE_ERR_VALUE_OVER_HIGH ||\
      t == TEMPERATURE_ERR_VALUE_OVER_LOW ){
      temperature.dir=T_DIR_INIT;
      temperature.value=t;
      temperature.changed=TRUE;
      log_error("temperature err.code:0x%2x.\r\n",temperature.value);
   }else if(t > temperature.value && temperature.dir == T_DIR_UP    ||\
            t < temperature.value && temperature.dir == T_DIR_DOWN  ||\
            temperature.dir == T_DIR_INIT                           ||\
            delta_time >= TEMPERATURE_TASK_T_HOLD_TIME ){         
   temperature.dir = t > temperature.value?T_DIR_UP:T_DIR_DOWN;
   temperature.value = t;
   temperature.changed=TRUE;
   temperature.time =cur_time; 
   }
   }
   
   if(temperature.changed == TRUE){
   log_debug("teperature changed dir:%d value:%d C delta_time:%d ms.\r\n",temperature.dir,temperature.value,delta_time);
   temperature.changed=FALSE;
   
   a_msg.type = BROADCAST_TEMPERATURE_VALUE;
   a_msg.temperature= temperature.value;
   status = osMessagePut(alarm_task_msg_q_id,(uint32_t)&a_msg,TEMPERATURE_TASK_PUT_MSG_TIMEOUT);
   if(status !=osOK){
   log_error("put alarm broadcast t msg error:%d\r\n",status); 
   }
   
   c_msg.type = BROADCAST_TEMPERATURE_VALUE;
   c_msg.temperature= temperature.value;
   status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&c_msg,TEMPERATURE_TASK_PUT_MSG_TIMEOUT);
   if(status !=osOK){
   log_error("put compressor broadcast t msg error:%d\r\n",status); 
   }
   
   d_msg.type = BROADCAST_TEMPERATURE_VALUE;
   d_msg.temperature =  temperature.value;
   status = osMessagePut(display_task_msg_q_id,(uint32_t)&d_msg,TEMPERATURE_TASK_PUT_MSG_TIMEOUT);
   if(status !=osOK){
   log_error("put display broadcast t msg error:%d\r\n",status); 
   }
   }
  }
  /*主动请求温度消息处理*/
  if(ptr_msg->type == REQ_TEMPERATURE_VALUE){
   response_msg.type = RESPONSE_TEMPERATURE_VALUE;
   response_msg.temperature= temperature.value;
   status = osMessagePut(ptr_msg->req_q_id,(uint32_t)&response_msg,TEMPERATURE_TASK_PUT_MSG_TIMEOUT);
   if(status !=osOK){
   log_error("put response t msg error:%d\r\n",status); 
   }
  }
 }
 }
}