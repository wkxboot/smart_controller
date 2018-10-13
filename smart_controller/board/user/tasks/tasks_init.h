#ifndef  __TASKS_H__
#define  __TASKS_H__
#include "stdint.h"


#ifdef  __cplusplus
#define TASKS_BEGIN  extern "C" {
#define TASKS_END    }
#else
#define TASKS_BEGIN  
#define TASKS_END   
#endif

TASKS_BEGIN

extern EventGroupHandle_t tasks_sync_evt_group_hdl;
void tasks_init();


#define  TASKS_SYNC_EVENT_SCALE_TASK_RDY            (1<<0)
#define  TASKS_SYNC_EVENT_PROTOCOL_TASK_RDY         (1<<1)
#define  TASKS_SYNC_EVENT_DOOR_LOCK_TASK_RDY        (1<<2)
#define  TASKS_SYNC_EVENT_TEMPERATURE_TASK_RDY      (1<<3)
#define  TASKS_SYNC_EVENT_ADC_TASK_RDY              (1<<4)
#define  TASKS_SYNC_EVENT_CPU_TASK_RDY              (1<<5)
#define  TASKS_SYNC_EVENT_COMPRESSOR_TASK_RDY       (1<<6)
#define  TASKS_SYNC_EVENT_ALL_TASKS_RDY             ((1<<6)-1)


typedef enum
{
T_ADC_COMPLETED=0,
REQ_NET_WEIGHT,
REQ_DOOR_STATUS,
REQ_LOCK_STATUS,
REQ_TEMPERATURE,
REQ_MANUFACTURE_ID,
REQ_SCALE_CNT,
REQ_UNLOCK,
REQ_LOCK,
REQ_SET_SENSOR,
BROADCAST_TEMPERATURE,
RESPONSE_TEMPERATURE,
RESPONSE_NET_WEIGHT,
RESPONSE_DOOR_STATUS,
RESPONSE_LOCK_STATUS,
RESPONSE_MANUFACTURE_ID,
RESPONSE_SCALE_CNT,
RESPONSE_LOCK_RESULT,
RESPONSE_UNLOCK_RESULT,
RESPONSE_SET_SENSOR
}task_msg_type_t;


typedef struct
{
task_msg_type_t type;
union
{
int16_t        *net_weight;
uint8_t        door_status;
uint8_t        lock_status;
int8_t         temperature;
uint8_t        scale_cnt;
uint8_t        unlock_result;
uint8_t        lock_result;
uint8_t        sensor_value;
uint8_t        sensor_result;
uint16_t       manufacture_id;
uint16_t       adc;
osMessageQId   req_q_id;
uint32_t       reserved;
};
}task_msg_t;



#ifndef  NULL
#define  NULL         (void*)0
#endif


#ifndef  TRUE       
#define  TRUE         1
#endif

#ifndef  FALSE       
#define  FALSE        0
#endif








TASKS_END




#endif