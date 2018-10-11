#include "board.h"
#include "cmsis_os.h"
#include "serial.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "door_lock_task.h"
#include "protocol_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[protocol]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 

extern int protocol_serial_handle;
extern serial_hal_driver_t protocol_serial_driver;

osThreadId   protocol_task_hdl;
osMessageQId protocol_task_msg_q_id;

static task_msg_t scale_msg;
static task_msg_t door_lock_msg;


typedef enum
{
 PROTOCOL_TASK_ADU_STEP=0,
 PROTOCOL_TASK_CRC_STEP
}protocol_step_t;


/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

static uint16_t protocol_task_crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    uint32_t i; /* will index into CRC lookup */

   /* calculate the CRC  */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; 
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

static uint16_t protocol_task_prepare_crc16(uint8_t *send_buffer,uint16_t length_to_write)
{
uint16_t crc_calculated;
crc_calculated = protocol_task_crc16(send_buffer,length_to_write);

send_buffer[length_to_write++] = crc_calculated & 0xff;
send_buffer[length_to_write++] = crc_calculated >> 8;

return length_to_write;
}

static int protocol_task_get_net_weight(int16_t *net_weight)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;
 int        rc = -1;
 
 scale_msg.type = REQ_NET_WEIGHT;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
  return rc;
 }
 
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 if(msg->type == RESPONSE_NET_WEIGHT){
   for(uint8_t i=0;i<SCALE_TASK_SCALE_CNT;i++){
   *net_weight++ = *msg->net_weight++;
   }
   rc = 0;
 }
 }
 
 return rc;
}




static int protocol_task_get_door_status(uint8_t *door_status)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;
 int        rc = -1;
 
 door_lock_msg.type = REQ_DOOR_STATUS;
 status = osMessagePut(door_lock_task_msg_q_id,(uint32_t)&door_lock_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
  return rc;
 }
 
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 if(msg->type == RESPONSE_DOOR_STATUS){
   *door_status = msg->door_status;
   rc = 0;
 }
 }
 
 return rc;
}

static int protocol_task_get_lock_status(uint8_t *lock_status)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;
 int        rc = -1;
 
 scale_msg.type = REQ_LOCK_STATUS;
 status = osMessagePut(door_lock_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
  return rc;
 }
 
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 if(msg->type == RESPONSE_LOCK_STATUS){
   *lock_status = msg->lock_status;
   rc = 0;
 }
 }
 
 return rc;
}

static uint8_t protocol_task_get_temperature(int8_t *temperature)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;
 int        rc = -1;
 
 scale_msg.type = REQ_TEMPERATURE;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
  return rc;
 }
 
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 if(msg->type == RESPONSE_TEMPERATURE){
   *temperature = msg->temperature;
   rc = 0;
 }
 }
 
 return rc;
}

static int protocol_task_get_manufacture_id(uint16_t *id)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;
 int        rc = -1;
 
 scale_msg.type = REQ_MANUFACTURE_ID;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
  return rc;
 }
 
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 if(msg->type == RESPONSE_MANUFACTURE_ID){
   *id = msg->manufacture_id;
   rc = 0;
 }
 }
 
 return rc;
}

static int protocol_task_get_scale_cnt(uint8_t *scale_cnt)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;
 int        rc = -1;
 
 scale_msg.type = REQ_SCALE_CNT;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
  return rc;
 }
 
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 if(msg->type == RESPONSE_SCALE_CNT){
   *scale_cnt = msg->scale_cnt;
   rc = 0;
 }
 }
 
 return rc;
}

int protocol_task_unlock(uint8_t *result)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;
 int        rc = -1;
 
 door_lock_msg.type = REQ_UNLOCK;
 status = osMessagePut(door_lock_task_msg_q_id,(uint32_t)&door_lock_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
  return rc;
 }
 
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 if(msg->type == RESPONSE_UNLOCK_RESULT){
   *result = msg->unlock_result;
   rc = 0;
 }
 }
 
 return rc;
}

static int protocol_task_lock(uint8_t *result)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;
 int        rc = -1;
 
 door_lock_msg.type = REQ_LOCK;
 status = osMessagePut(door_lock_task_msg_q_id,(uint32_t)&door_lock_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
  return rc;
 }
 
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 if(msg->type == RESPONSE_LOCK_RESULT){
   *result = msg->lock_result;
   rc = 0;
 }
 }
 
 return rc;
}


void protocol_task(void const * argument)
{
 int rc; 
 int length_to_read,read_length=0;
 int length_to_write,write_length,remain_length;
 uint8_t  result;
 uint8_t  status;
 uint8_t  scale_cnt; 
 uint32_t timeout;
 uint16_t crc_calculated;
 uint16_t crc_received;
 int16_t  net_weight[SCALE_TASK_SCALE_CNT];
 uint16_t id;
 int8_t   temperature;
 protocol_step_t step;
 
 
 uint8_t recv_buffer[PROTOCOL_TASK_FRAME_SIZE_MAX];
 uint8_t send_buffer[PROTOCOL_TASK_FRAME_SIZE_MAX];
 
 osMessageQDef(protocol_task_msg_q,1,uint32_t);
 protocol_task_msg_q_id = osMessageCreate(osMessageQ(protocol_task_msg_q),protocol_task_hdl);
 log_assert(protocol_task_msg_q_id); 
 
 rc = serial_create(&protocol_serial_handle,PROTOCOL_TASK_RX_BUFFER_SIZE,PROTOCOL_TASK_TX_BUFFER_SIZE);
 log_assert(rc == 0);
 rc = serial_register_hal_driver(protocol_serial_handle,&protocol_serial_driver);
 log_assert(rc == 0);
 
 rc = serial_open(protocol_serial_handle,
                  PROTOCOL_TASK_SERIAL_PORT,
                  PROTOCOL_TASK_SERIAL_BAUDRATES,
                  PROTOCOL_TASK_SERIAL_DATABITS,
                  PROTOCOL_TASK_SERIAL_STOPBITS);
 
 log_assert(rc == 0); 
 
  /*等待任务同步*/
  xEventGroupSync(tasks_sync_evt_group_hdl,TASKS_SYNC_EVENT_PROTOCOL_TASK_RDY,TASKS_SYNC_EVENT_ALL_TASKS_RDY,osWaitForever);
  log_debug("protocol task sync ok.\r\n");
 
 serial_flush(protocol_serial_handle);
 
 while(1){
protocol_parse_start:

  timeout = PROTOCOL_TASK_FRAME_TIMEOUT_VALUE;
  length_to_read = 2;
  read_length =0;
  length_to_write = 0;
  step = PROTOCOL_TASK_ADU_STEP;
  
  while(length_to_read != 0){
  rc = serial_select(protocol_serial_handle,timeout);
  if(rc == -1){
   log_error("protocol select error.\r\n");
   goto protocol_parse_start;
  }
  if(rc == 0){
   log_error("protocol select timeout.\r\n");
   goto protocol_parse_start;
  }
  
   rc = serial_read(protocol_serial_handle,recv_buffer + read_length,length_to_read);
   if(rc == -1) {
    log_error("protocol read error.\r\n");
    goto protocol_parse_start;
   }

   for (int i=0; i < rc; i++){
   log_debug("<%2X>\r\n", recv_buffer[read_length + i]);
   }
   
   read_length +=rc;
   length_to_read -=rc;
   
   if(length_to_read == 0){
     switch(step){
     /*接收到了协议头和数据长度域*/
     case PROTOCOL_TASK_ADU_STEP:
       if(recv_buffer[PROTOCOL_TASK_CMD_ADDR_OFFSET] == PROTOCOL_TASK_CMD_ADDR_VALUE){
        step = PROTOCOL_TASK_CRC_STEP;      
        send_buffer[PROTOCOL_TASK_CMD_ADDR_OFFSET]=PROTOCOL_TASK_CMD_ADDR_VALUE;
        send_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET]=recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET];                                                              
        length_to_write = PROTOCOL_TASK_CMD_ADDR_LEN + PROTOCOL_TASK_CMD_CODE_LEN; 
              
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_NET_WEIGHT){
        length_to_read = PROTOCOL_TASK_CMD_NET_WEIGHT_PARAM_LEN + PROTOCOL_TASK_CMD_CRC_LEN;
        }
        
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_DOOR_STATUS){
        length_to_read = PROTOCOL_TASK_CMD_DOOR_STATUS_PARAM_LEN + PROTOCOL_TASK_CMD_CRC_LEN;
        }
                
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_LOCK_STATUS){
        length_to_read = PROTOCOL_TASK_CMD_LOCK_STATUS_PARAM_LEN + PROTOCOL_TASK_CMD_CRC_LEN;
        }
                       
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_TEMPERATURE){
        length_to_read = PROTOCOL_TASK_CMD_TEMPERATURE_PARAM_LEN + PROTOCOL_TASK_CMD_CRC_LEN;
        }
       
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_MANUFACTURE_ID){
        length_to_read = PROTOCOL_TASK_CMD_MANUFACTURE_ID_PARAM_LEN + PROTOCOL_TASK_CMD_CRC_LEN;
        }
        
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_SCALE_CNT){
        length_to_read = PROTOCOL_TASK_CMD_SCALE_CNT_PARAM_LEN + PROTOCOL_TASK_CMD_CRC_LEN;
        }
      
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_UNLOCK){
        length_to_read = PROTOCOL_TASK_CMD_UNLOCK_PARAM_LEN + PROTOCOL_TASK_CMD_CRC_LEN;
        }
              
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_LOCK){
        length_to_read = PROTOCOL_TASK_CMD_LOCK_PARAM_LEN + PROTOCOL_TASK_CMD_CRC_LEN;
        }
        
        if(length_to_read == 0){
        log_error("protocol err in code value:%d.\r\n",recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET]);
        goto protocol_parse_start;
        }
       }else{
         log_error("protocol err in addr value:%d.\r\n",recv_buffer[PROTOCOL_TASK_CMD_ADDR_OFFSET]);
        goto protocol_parse_start;
       } 
                                                                    
     break;
     /*接收完成了全部的数据*/
     case PROTOCOL_TASK_CRC_STEP:
       crc_calculated = protocol_task_crc16(recv_buffer,read_length - PROTOCOL_TASK_CMD_CRC_LEN);
       crc_received = recv_buffer[read_length-2]<< 8 | recv_buffer[read_length-1];
       if(crc_calculated != crc_received){
          log_error("protocol err in crc.recv:%d calculate:%d.\r\n",crc_received,crc_calculated);
          goto protocol_parse_start;
       }
       
       /*如果是读取净重值*/
       if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_NET_WEIGHT){
          rc = protocol_task_get_net_weight(net_weight);
          if(rc != 0){
          log_error("protocol get net weight err.\r\n");
          goto protocol_parse_start;
          }
          
         /*所有称的重量值*/
          if(recv_buffer[PROTOCOL_TASK_CMD_PARAM_OFFSET] == 0){
            for(uint8_t i=0;i<SCALE_TASK_SCALE_CNT;i++){
            send_buffer[length_to_write++] = net_weight[i] >> 8;
            send_buffer[length_to_write++] = net_weight[i] & 0xFF;
            } 
            for(uint8_t i=0;i<PROTOCOL_TASK_SCALE_CNT_MAX - SCALE_TASK_SCALE_CNT;i++){
            send_buffer[length_to_write++] = 0;
            send_buffer[length_to_write++] = 0;
            } 
            
          }else if(recv_buffer[PROTOCOL_TASK_CMD_PARAM_OFFSET] >= 1 && 
                   recv_buffer[PROTOCOL_TASK_CMD_PARAM_OFFSET] <= PROTOCOL_TASK_CMD_SCALE_CNT){     
          send_buffer[length_to_write++] = net_weight[recv_buffer[PROTOCOL_TASK_CMD_PARAM_OFFSET]] >> 8;
          send_buffer[length_to_write++] = net_weight[recv_buffer[PROTOCOL_TASK_CMD_PARAM_OFFSET]] & 0xFF;
          }else{
          log_error("protocol scale num err:%d.\r\n",recv_buffer[PROTOCOL_TASK_CMD_PARAM_OFFSET]);
          goto protocol_parse_start;
          }       
          
          length_to_write = protocol_task_prepare_crc16(send_buffer,length_to_write);         
       }
       
       /*如果是查询门状态*/
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_DOOR_STATUS){
         rc = protocol_task_get_door_status(&status);
         if(rc != 0){
         log_error("protocol task get door status err.\r\n");
         goto protocol_parse_start;
         }
         if(status == DOOR_LOCK_TASK_DOOR_OPEN_STATUS){
         result = PROTOCOL_TASK_DOOR_OPEN_STATUS;                
         }else{
         result = PROTOCOL_TASK_DOOR_CLOSE_STATUS; 
         }
         /*填充操作结果值*/
         send_buffer[length_to_write++] = result;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16(send_buffer,length_to_write); 
       }
            
       /*如果是查询锁状态*/
        if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_LOCK_STATUS){
         rc = protocol_task_get_lock_status(&status);
         if(rc != 0){
         log_error("protocol task get lock status err.\r\n");
         goto protocol_parse_start;
         }
         if(status == DOOR_LOCK_TASK_LOCK_OPEN_STATUS){
         result = PROTOCOL_TASK_LOCK_OPEN_STATUS;                
         }else{
         result = PROTOCOL_TASK_DOOR_CLOSE_STATUS; 
         }
         /*填充操作结果值*/
         send_buffer[length_to_write++] = result;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16(send_buffer,length_to_write); 
       }
       
       /*如果是查询温度值*/
       if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_TEMPERATURE){
         rc = protocol_task_get_temperature(&temperature);
         if(rc != 0){
         log_error("protocol task get temperature err.\r\n");
         goto protocol_parse_start;
         }

         /*填充操作结果值*/
         send_buffer[length_to_write++] = temperature;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16(send_buffer,length_to_write); 
       }
      
      /*如果是读取固件版本号*/
       if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_MANUFACTURE_ID){
         rc = protocol_task_get_manufacture_id(&id);
         if(rc != 0){
         log_error("protocol task get manufacture id err.\r\n");
         goto protocol_parse_start;
         }

         /*填充操作结果值*/
         send_buffer[length_to_write++] = id >> 8;
         send_buffer[length_to_write++] = id &  0xFF;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16(send_buffer,length_to_write); 
       }
      
      /*如果是获取称数量*/
       if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_SCALE_CNT){
         rc = protocol_task_get_scale_cnt(&scale_cnt);
         if(rc != 0){
         log_error("protocol task get scale cnt err.\r\n");
         goto protocol_parse_start;
         }

         /*填充操作结果值*/
         send_buffer[length_to_write++] = scale_cnt;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16(send_buffer,length_to_write); 
       }
     
       /*如果是开锁*/
       if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_UNLOCK){
         rc = protocol_task_unlock(&status);
         if(rc != 0){
         log_error("protocol task unlock err.\r\n");
         goto protocol_parse_start;
         }
         if(status == DOOR_LOCK_TASK_UNLOCK_SUCCESS){
         result = PROTOCOL_TASK_UNLOCK_SUCCESS;
         }else{
         result = PROTOCOL_TASK_UNLOCK_FAILURE;
         }
         /*填充操作结果值*/
         send_buffer[length_to_write++] = result;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16(send_buffer,length_to_write); 
       }
       /*如果是关锁*/
       if(recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET] == PROTOCOL_TASK_CMD_LOCK){
         rc = protocol_task_lock(&status);
         if(rc != 0){
         log_error("protocol task lock err.\r\n");
         goto protocol_parse_start;
         }
         if(status == DOOR_LOCK_TASK_LOCK_SUCCESS){
         result = PROTOCOL_TASK_LOCK_SUCCESS;
         }else{
         result = PROTOCOL_TASK_LOCK_FAILURE;
         }
         /*填充操作结果值*/
         send_buffer[length_to_write++] = result;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16(send_buffer,length_to_write); 
       }       
     break;
     default :
     log_error("protocol err in internal:%d.\r\n",recv_buffer[PROTOCOL_TASK_CMD_CODE_OFFSET]); 
     goto protocol_parse_start;
     }                     
    }
  
   if(length_to_read != 0){   
   timeout = PROTOCOL_TASK_CHARACTER_TIMEOUT_VALUE; 
   }       
   }
 
    /*解析完毕 回应操作结果*/  
    write_length = serial_write(protocol_serial_handle,send_buffer,length_to_write);
    for (int i=0; i < write_length; i++){
    log_debug("[%2X]\r\n", send_buffer[i]);
    }
    if(write_length != length_to_write){
    log_error("protocol err in  serial buffer write. expect:%d write:%d.\r\n",length_to_write,write_length); 
    goto protocol_parse_start;      
    }
    
    remain_length = serial_complete(protocol_serial_handle,PROTOCOL_TASK_SEND_TIMEOUT);
    if(remain_length != 0){
    log_error("protocol err in  serial send timeout.\r\n",); 
    goto protocol_parse_start;  
    }
     
 }
}
