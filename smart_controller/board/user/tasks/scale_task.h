#ifndef  __SCALE_TASK_H__
#define  __SCALE_TASK_H__



extern osThreadId   scale_task_hdl;
extern osMessageQId scale_task_msg_q_id;
void scale_task(void const * argument);


#define  SCALE_TASK_SCALE_CNT                 4
#define  SCALE_TASK_MANUFACTURE_ID            0x1100


#define  SCALE_TASK_FRAME_SIZE_MAX            32
#define  SCALE_TASK_RX_BUFFER_SIZE            32
#define  SCALE_TASK_TX_BUFFER_SIZE            32

#define  SCALE_TASK_SERIAL_PORT               2
#define  SCALE_TASK_SERIAL_BAUDRATES          38400
#define  SCALE_TASK_SERIAL_DATABITS           8
#define  SCALE_TASK_SERIAL_STOPBITS           1

#define  SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE        osWaitForever
#define  SCALE_TASK_MSG_PUT_TIMEOUT_VALUE         5
#define  SCALE_TASK_REQ_NET_WEIGHT_TIMEOUT_VALUE  700
#define  SCALE_TASK_CHARACTER_TIMEOUT_VALUE       3
#define  SCALE_TASK_SEND_TIMEOUT_VALUE            10





/*称的协议*/
#define  SOF_OFFSET                           0
#define  SOF_VALUE                            0x05
#define  LEN_OFFSET                           1
#define  ADDR_OFFSET                          2
#define  ADDR_VALUE                           0x00

#define  NET_WEIGHT_CMD0_OFFSET               3
#define  NET_WEIGHT_CMD0_VALUE                0xe2
#define  NET_WEIGHT_CMD1_OFFSET               4
#define  NET_WEIGHT_CMD1_VALUE                0x01
#define  NET_WEIGHT_OFFSET                    5
#define  NET_WEIGHT_STATUS_OFFSET             15
#define  NET_WEIGHT_STATUS_SUCCESS            0x00
#define  NET_WEIGHT_STATUS_FALIURE            0x01

#define  SCALE_TASK_REQ_NET_WEIGHT_LEN        7
#define  SCALE_TASK_RESPONSE_NET_WEIGHT_LEN   18

#endif