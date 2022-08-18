#ifndef COMMON_PORTING_H__
#define COMMON_PORTING_H__
#include <stdio.h>
#include "stdint.h"


//#include "iica_user.c"
extern uint32_t g_ticks;
#define BUS_TIMEOUT             200
int8_t IIC11_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t IIC11_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void DelayUs(uint32_t Delay);
void sleep(uint32_t t_ms);
int64_t get_timestamp_us(void);

#endif
