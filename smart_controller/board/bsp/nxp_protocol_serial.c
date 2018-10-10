#include "cmsis_os.h"   
#include "board.h"
#include "fsl_usart.h"
#include "fsl_clock.h"
#include "pin_mux.h"
#include "serial.h"

/*lpc54606在IAR freertos下的移植*/
int nxp_protocol_serial_init(uint8_t port,uint32_t bauds,uint8_t data_bit,uint8_t stop_bit);
int nxp_protocol_serial_deinit(uint8_t port);
void nxp_protocol_serial_enable_txe_int();
void nxp_protocol_serial_disable_txe_int();
void nxp_protocol_serial_enable_rxne_int();
void nxp_protocol_serial_disable_rxne_int();


serial_hal_driver_t protocol_serial_driver={
.init =nxp_protocol_serial_init,
.deinit = nxp_protocol_serial_deinit,
.enable_txe_int=nxp_protocol_serial_enable_txe_int,
.disable_txe_int =nxp_protocol_serial_disable_txe_int,
.enable_rxne_int =nxp_protocol_serial_enable_rxne_int,
.disable_rxne_int =nxp_protocol_serial_disable_rxne_int
};

static USART_Type *serial;
static IRQn_Type  serial_irq_num;
static clock_name_t clk_src;
int protocol_serial_handle;

int nxp_protocol_serial_init(uint8_t port,uint32_t bauds,uint8_t data_bit,uint8_t stop_bit)
{
    status_t status;
    usart_config_t config;

       
    if(port == 0){
    serial=USART0;
    serial_irq_num=FLEXCOMM0_IRQn;
    clk_src = kCLOCK_Flexcomm0;
    }else if(port == 1){
    serial=USART1;
    serial_irq_num=FLEXCOMM1_IRQn;
    clk_src = kCLOCK_Flexcomm1;
    }else if(port == 2){
    serial=USART2;
    serial_irq_num=FLEXCOMM2_IRQn;
    clk_src = kCLOCK_Flexcomm2;
    }else if(port == 3){
    serial=USART3;
    serial_irq_num=FLEXCOMM3_IRQn;
    clk_src = kCLOCK_Flexcomm3;
    }else{
    serial=USART0;
    serial_irq_num=FLEXCOMM0_IRQn;
    clk_src = kCLOCK_Flexcomm0;
    }
    
    config.baudRate_Bps = bauds;
    if(data_bit == 8){
    config.bitCountPerChar = kUSART_8BitsPerChar;
    }else{
    config.bitCountPerChar = kUSART_7BitsPerChar;
    }
    if(stop_bit ==1){
    config.stopBitCount = kUSART_OneStopBit;
    }else{
    config.stopBitCount = kUSART_TwoStopBit;
    }
    
    config.parityMode = kUSART_ParityDisabled;
   
    config.loopback = false;
    config.enableRx = true;
    config.enableTx = true;
    /* Initialize the USART with configuration. */
    status=USART_Init(serial, &config, CLOCK_GetFreq(clk_src));
 
  if (status != kStatus_Success){
    return -1;
  } 
  EnableIRQ(serial_irq_num);
  return 0;
}
int nxp_protocol_serial_deinit(uint8_t port)
{
  return 0;
}
void nxp_protocol_serial_enable_txe_int()
{
 //USART_EnableInterrupts(serial,USART_INTENSET_TXRDYEN_MASK);
}
void nxp_protocol_serial_disable_txe_int()
{
  //USART_DisableInterrupts(serial,USART_INTENSET_TXRDYEN_MASK);  
}

void nxp_protocol_serial_enable_rxne_int()
{
 //USART_EnableInterrupts(serial,USART_INTENSET_RXRDYEN_MASK);
}
void nxp_protocol_serial_disable_rxne_int()
{
 //USART_DisableInterrupts(serial,USART_INTENSET_RXRDYEN_MASK);
}


void nxp_protocol_serial_isr()
{
  int result;
  uint32_t tmp_flag = 0;
  uint8_t  send_byte,recv_byte;
  
  tmp_flag = USART_GetEnabledInterrupts(serial);
  //tmp_it_source =USART_GetStatusFlags(serial);
  
 /*接收中断处理*/
  if(tmp_flag & kUSART_RxFifoNotEmptyFlag){
      recv_byte=USART_ReadByte(serial);
      isr_serial_put_byte_from_recv(protocol_serial_handle,recv_byte);

  }
 /*发送中断处理*/
  if(tmp_flag & kUSART_TxFifoEmptyFlag){
  	 result =isr_serial_get_byte_to_send(protocol_serial_handle,&send_byte);
    if(result == 1) {
     USART_WriteByte(serial, send_byte);
     }
   }
}


/*串口中断处理*/
void FLEXCOMM0_IRQHandler()
{
 nxp_protocol_serial_isr();
}


