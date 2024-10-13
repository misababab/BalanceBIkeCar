#ifndef __USART2_H
#define __USART2_H

#include "stm32f10x.h"
#include "stm32f10x_usart.h"

extern u8 usart2_flag;
extern u8 usart2_CacheByte;

///////////////////////
#define CMD_BUFFER_LEN 100
extern int read_x2;
extern int read_y2;
extern uint8_t aRxBuffer2[20];
extern uint8_t RxCounter2;
extern uint8_t ReceiveState2;
extern uint8_t i2;
/////////////////////


/////////////////////
void usart2_Init(u32 bound);
void Usart2_send(u8 com);
void uart2_SendData(u8 ch);
void uart2_SendString(u8 *ch);
void printf2(char *fmt, ...);

void usart2receive(void);

#endif  

